//! `cargo xtask test`: build the workspace's tests once, then run every test binary at the same
//! time. `cargo test` runs them one after another, so one slow binary held up all the others;
//! the doc tests and any other `side` jobs run alongside on their own thread.

use std::collections::HashSet;
use std::process::{Command, Stdio};
use std::sync::mpsc;
use std::time::Instant;

use crate::{Backend, cargo, root};

/// Tests the OpenCASCADE backend is expected to fail, one libtest path per line (`#` comments);
/// `cargo xtask test` skips them and fails on any of them that passes.
pub const CADRUM_ALLOWLIST: &str = "tests/cadrum-allowlist.txt";
/// Tests that crash the OpenCASCADE backend's process (an access violation inside OpenCASCADE,
/// which no `Result` catches), in the allowlist's format: `cargo xtask test`
/// passes each as `--skip` to its binary, so the rest of that binary's tests still run.
pub const CADRUM_SKIPLIST: &str = "tests/cadrum-skiplist.txt";

/// A test executable from `cargo test --no-run`.
struct TestBin {
    label: String,
    exe: String,
    dir: String,
}

/// Build the tests with `cargo_args` (e.g. `--locked`), run each binary with `test_args`, and run
/// the `side` jobs one after another meanwhile. Each prints a line when it ends; a failure prints
/// its whole output. With the cadrum backend (the default) the tests listed in [`CADRUM_ALLOWLIST`] may fail
/// (a binary whose failures are all listed counts as passing), and one of them that passes fails
/// the run until its line goes: the list only ever shrinks.
pub fn run_backend(cargo_args: &[&str], test_args: &[&str], side: Vec<(String, Command)>, backend: Backend) -> Result<(), String> {
    let (allow, skip) = match backend {
        Backend::Truck => (HashSet::new(), HashSet::new()),
        Backend::Cadrum => (listed(CADRUM_ALLOWLIST)?, listed(CADRUM_SKIPLIST)?),
    };
    let bins = build(cargo_args)?;
    let start = Instant::now();
    let (tx, rx) = mpsc::channel::<(String, Result<String, String>, f64)>();
    let mut failed = Vec::new();
    let mut stale = Vec::new();
    let mut allowed = 0usize;
    std::thread::scope(|scope| {
        for b in &bins {
            let tx = tx.clone();
            let skips: Vec<String> = skip.iter().filter_map(|k| k.strip_prefix(&format!("{} :: ", b.label))).map(str::to_string).collect();
            scope.spawn(move || {
                let t0 = Instant::now();
                let mut c = Command::new(&b.exe);
                c.args(test_args).current_dir(&b.dir).env("CARGO_MANIFEST_DIR", &b.dir);
                for t in &skips {
                    c.arg("--skip").arg(t);
                }
                let _ = tx.send((b.label.clone(), finish(c), t0.elapsed().as_secs_f64()));
            });
        }
        let tx_side = tx.clone();
        scope.spawn(move || {
            for (label, c) in side {
                let t0 = Instant::now();
                let _ = tx_side.send((label, finish(c), t0.elapsed().as_secs_f64()));
            }
        });
        drop(tx);
        for (label, result, secs) in rx {
            match result {
                Ok(output) => {
                    let (passed, _) = outcomes(&output);
                    stale.extend(passed.iter().filter(|t| allow.contains(&key(&label, t))).map(|t| format!("{label} :: {t}")));
                    println!("ok    {label:<44} {secs:6.1}s  {}", summary(&output));
                }
                Err(output) => {
                    let (passed, failures) = outcomes(&output);
                    let unexpected: Vec<&String> = failures.iter().filter(|t| !allow.contains(&key(&label, t))).collect();
                    stale.extend(passed.iter().filter(|t| allow.contains(&key(&label, t))).map(|t| format!("{label} :: {t}")));
                    // A process that ended without its result line crashed (an abort or an access
                    // violation): the tests it never reached are unknown, so it fails whatever
                    // the allowlist says.
                    let crashed = !output.contains("test result:");
                    if crashed {
                        let last = output.lines().rev().find(|l| l.starts_with("test ")).unwrap_or("");
                        println!("FAIL  {label:<44} {secs:6.1}s\n{output}");
                        println!(
                            "FAIL  {label}: the test process ended without a result (crashed; the last test it reported: {last:?}); a crashing test goes into {CADRUM_SKIPLIST}"
                        );
                        failed.push(label);
                    } else if !allow.is_empty() && unexpected.is_empty() && !failures.is_empty() {
                        allowed += failures.len();
                        println!("ok    {label:<44} {secs:6.1}s  {} ({} allowlisted failure(s))", summary(&output), failures.len());
                    } else {
                        println!("FAIL  {label:<44} {secs:6.1}s\n{output}");
                        if !unexpected.is_empty() {
                            let names = unexpected.iter().map(|s| s.as_str()).collect::<Vec<_>>().join(", ");
                            match backend {
                                Backend::Cadrum => println!("FAIL  {label}: not in {CADRUM_ALLOWLIST}: {names}"),
                                Backend::Truck => println!("FAIL  {label}: {names}"),
                            }
                        }
                        failed.push(label);
                    }
                }
            }
        }
    });
    println!("test: {} binaries in {:.1}s", bins.len(), start.elapsed().as_secs_f64());
    if allowed > 0 {
        println!("allowlist: {allowed} failure(s) allowed by {CADRUM_ALLOWLIST}");
    }
    if !stale.is_empty() {
        println!(
            "now passing, remove from {CADRUM_ALLOWLIST}:
{}",
            stale.join(
                "
"
            )
        );
    }
    if !failed.is_empty() {
        return Err(format!("failed: {}", failed.join(", ")));
    }
    if !stale.is_empty() {
        return Err(format!("{} allowlisted test(s) now pass: remove them from {CADRUM_ALLOWLIST} (listed above)", stale.len()));
    }
    Ok(())
}

/// The allowlist's entries, `<binary label> :: <test path>` each.
fn listed(file: &str) -> Result<HashSet<String>, String> {
    let path = root().join(file);
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(text.lines().map(|l| l.split('#').next().unwrap_or("").trim().to_string()).filter(|l| !l.is_empty()).collect())
}

fn key(label: &str, test: &str) -> String {
    format!("{label} :: {test}")
}

/// The tests a binary's output reports as passed and as failed.
fn outcomes(output: &str) -> (Vec<String>, Vec<String>) {
    let (mut passed, mut failed) = (Vec::new(), Vec::new());
    for line in output.lines() {
        if let Some(rest) = line.strip_prefix("test ") {
            if let Some(name) = rest.strip_suffix(" ... ok") {
                passed.push(name.trim().to_string());
            } else if let Some(name) = rest.strip_suffix(" ... FAILED") {
                failed.push(name.trim().to_string());
            }
        }
    }
    (passed, failed)
}

/// The last `test result:` line of a binary's output.
fn summary(output: &str) -> String {
    output.lines().rev().find(|l| l.starts_with("test result:")).unwrap_or("").trim_start_matches("test result: ").to_string()
}

/// Run to the end: the output on success, the output and the exit status on failure.
fn finish(mut c: Command) -> Result<String, String> {
    let out = c.stdin(Stdio::null()).output().map_err(|e| format!("failed to spawn: {e}"))?;
    let stdout = String::from_utf8_lossy(&out.stdout);
    if out.status.success() { Ok(stdout.into_owned()) } else { Err(format!("{stdout}{}\n({})", String::from_utf8_lossy(&out.stderr), out.status)) }
}

/// `cargo test --workspace --profile ci --no-run`, and the executables it built.
fn build(cargo_args: &[&str]) -> Result<Vec<TestBin>, String> {
    let mut c = cargo();
    c.args(["test", "--workspace", "--profile", "ci", "--no-run", "--message-format=json-render-diagnostics"]).args(cargo_args);
    eprintln!("$ cargo test --workspace --profile ci --no-run");
    let out = c.stderr(Stdio::inherit()).output().map_err(|e| format!("cargo test --no-run: failed to spawn: {e}"))?;
    if !out.status.success() {
        return Err(format!("cargo test --no-run: exited with {}", out.status));
    }
    let mut bins: Vec<TestBin> = Vec::new();
    for line in String::from_utf8_lossy(&out.stdout).lines() {
        let Ok(m) = serde_json::from_str::<serde_json::Value>(line) else { continue };
        let Some(exe) = m["executable"].as_str().filter(|_| m["reason"] == "compiler-artifact" && m["profile"]["test"] == true) else {
            continue;
        };
        if bins.iter().any(|b| b.exe == exe) {
            continue;
        }
        let manifest = std::path::Path::new(m["manifest_path"].as_str().unwrap_or_default()).to_path_buf();
        let dir = manifest.parent().map(|d| d.to_path_buf()).unwrap_or_else(root);
        let pkg = dir.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
        let kind = m["target"]["kind"][0].as_str().unwrap_or("");
        let label = format!("{pkg} {kind} {}", m["target"]["name"].as_str().unwrap_or(""));
        bins.push(TestBin { label, exe: exe.to_string(), dir: dir.to_string_lossy().into_owned() });
    }
    if bins.is_empty() {
        return Err("cargo test --no-run built no test executables".into());
    }
    Ok(bins)
}

/// The doc tests (`cargo test --doc`), to run as a side job.
pub fn doc_tests(cargo_args: &[&str], test_args: &[&str]) -> (String, Command) {
    let mut c = cargo();
    c.args(["test", "--workspace", "--profile", "ci", "--doc"]).args(cargo_args).arg("--").args(test_args);
    ("doc tests".into(), c)
}

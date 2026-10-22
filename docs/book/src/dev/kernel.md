# The kernel boundary and its backends

`crates/kernel` is SolidCraft's B-rep kernel boundary: a façade that re-exports one geometry
backend, chosen by cargo feature. `cadrum` (the default) is `crates/kernel-cadrum`: OpenCASCADE
8 (LGPL-2.1 with the OCCT exception), prebuilt and statically linked through the
[cadrum](https://github.com/lzpel/cadrum) crate (MIT), which is the only crate allowed to touch
it. `truck` is `crates/kernel-truck`, the previous backend: [truck](https://github.com/ricosjp/truck)
(Apache-2.0) for topology, NURBS geometry, booleans, tessellation and STEP writing, plus our own
blends, shells and face edits on top; it stays until the migration's last phase removes it. What
both share lives in `crates/kernel-core`: `KernelError` and `guard`, the exact face and edge
history, face sources and paints, measures, mesh bodies, the Part 21 and IGES text readers, the
2-D offsetting of profiles. Callers get SolidCraft types (`Body`, face and edge indices, meshes,
measures) and a `KernelError` instead of backend types and panics, whichever backend is built
(`solidcraft_kernel::BACKEND` names it).

## Building and testing

The default build is the OpenCASCADE backend; the first build downloads cadrum's prebuilt
OpenCASCADE for the host. `cargo build -F solidcraft-kernel/truck` selects the truck backend on
workspace commands, `-F truck` on one app (`cargo run -p solidcraft -F truck`); the truck feature
wins when both are on, so nothing has to be switched off. `cargo xtask ci` (also `test`, `oracle`
and `wasm`) runs the gate against the default; `--backend truck` runs it against truck.

The default gate knows the backend's gaps, in three places:

- A test of a feature OpenCASCADE does not give us (sheet metal's sewing, the surface workspace,
  drafts, variable-radius and G2 fillets, emboss, split and delete face, mesh conversion, …) is
  marked `#[ignore = "needs OpenCASCADE: <feature> (lzpel/cadrum#317)"]` in the source, so plain
  `cargo test` is green and `cargo test -- --ignored` lists what is missing. When the feature
  lands, remove the attribute. Each such operation returns
  `KernelError::TooLarge("… on the OpenCASCADE backend")`, shown as "not supported yet".
- A test that fails for another reason (a measurement, a name or a pick that comes out
  differently on OpenCASCADE: a bug to fix in the backend) is listed in
  `tests/cadrum-allowlist.txt` (`<binary label> :: <test path>`). It may fail, and one that passes
  fails the run until its line goes, so the list only shrinks.
- A test that crashes the process inside OpenCASCADE (an access violation no `Result` catches)
  is skipped through `tests/cadrum-skiplist.txt`; the reproductions are in `tests/cadrum-crashes/`
  and each entry waits on a cadrum fix.

An oracle case the default backend fails is marked `known_failure` in its `expected.json`
(`known_failure_truck` for the truck backend); the robustness goldens are
`tests/robustness/goldens.json` for the default backend and `goldens-truck.json` for truck. What
cadrum itself still lacks is collected in one tracking issue,
[lzpel/cadrum#317](https://github.com/lzpel/cadrum/issues/317), re-edited as gaps turn up.

OpenCASCADE builds for wasm32 only with the wasi-sdk toolchain of cadrum's Docker image, so
`cargo xtask wasm` runs the check inside `ghcr.io/lzpel/cross-wasm32-unknown-unknown` (and says
it skipped it where docker is not available), and the web app is built with
`packaging/web/build-cadrum.sh`.

## What the cadrum backend keeps in a body

A `Body` holds its lumps (`cadrum::Solid`s) behind a mutex: cadrum's types are `Send` but not
`Sync`, and OpenCASCADE writes a shape's triangulation into the shape. The face and edge order is
read once when the body is made (OpenCASCADE's traversal order, lump by lump) and every index the
application sees refers to it; the shape ids never leave the crate. A cadrum `clone` is a deep
copy that changes every id, so bodies share their solids through `Arc` and only an operation that
consumes a solid (a transform) copies one.

The exact history every operation reports (which input face each output face came from, for
[persistent naming](naming.md)) is rebuilt from OpenCASCADE's own history where it has one
(booleans, fillets, chamfers, shells, offsets) and from geometry where it has none: a face kept
is found by its centre and area, a generated blend face by the picked edge nearest to it, the
sides and caps of an extrude or a revolve by the profile segment or end they lie on. Display
plans are mesh-only (OpenCASCADE's surfaces are not laid out for the GPU shader): the faces carry
their positions and normals, which the GPU draws as given.

## `guard`

A backend can panic on geometry it doesn't handle (truck does; cadrum's Rust side can too).
Every call into one goes through `solidcraft_kernel::guard`, which catches the panic and turns
it into `KernelError::Internal`, leaving the model unchanged:

```rust
pub fn guard<T>(what: &str, f: impl FnOnce() -> Result<T>) -> Result<T>
```

A failing feature shows an error on the timeline; it never takes the app down. An OpenCASCADE
exception inside a cadrum call that returns no `Result` ends the process instead; those are
tracked in lzpel/cadrum#317 and, until fixed, kept out of the test run by the skip list.

## The truck backend

truck has no fillets, chamfers, shells or face edits, so on that backend these are SolidCraft
code in `crates/kernel-truck`: constant, chord and variable-radius fillets with corner blends,
chamfers, shell and offset, Delete Face, Split Face, Replace Face, sewing, STEP and IGES reading
and writing beyond truck's parser, and face provenance.

Patched copies of `truck-shapeops`, `truck-stepio` and `truck-meshalgo` live in `vendor/`,
wired in with `[patch.crates-io]`. The patches fix robustness bugs found by our tests and the
STEP corpus. `vendor/` is excluded from our fmt and clippy rules.

Equal cylinders crossing each other (`tangent_cylinders.rs`) touch at two points, which truck's
mesh-based intersection cannot trace through. When a boolean of such a pair fails, the kernel
splits both cylinder faces along the line through the touching points and takes the boolean of
the split bodies; the shapeops patch that picks the corner of a face at a vertex several curves
end on makes the arms of the crossing connect.

Some truck assertions only run in debug builds and can fail on valid geometry, so tests and CI
run with `--profile ci` or `--release`.

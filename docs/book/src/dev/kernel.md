# The kernel boundary and its backend

`crates/kernel` is SolidCraft's B-rep kernel boundary: a façade that re-exports the geometry
backend, `crates/kernel-cadrum`: OpenCASCADE 8 (LGPL-2.1 with the OCCT exception), prebuilt and
statically linked through the [cadrum](https://github.com/lzpel/cadrum) crate (MIT), which is
the only crate allowed to touch it. What needs no geometry library lives in `crates/kernel-core`:
`KernelError` and `guard`, the exact face and edge history, face sources and paints, measures,
mesh bodies, the Part 21 and IGES text readers, the 2-D offsetting of profiles. Callers get
SolidCraft types (`Body`, face and edge indices, meshes, measures) and a `KernelError` instead of
backend types and panics (`solidcraft_kernel::BACKEND` names the backend, `"cadrum"`).

The pure-Rust truck backend SolidCraft started with (`crates/kernel-truck`, with our own blends,
shells, face edits and STEP reader on top of [truck](https://github.com/ricosjp/truck)) was
replaced in ten phases on this fork and removed in the last one
(lzpel/solidcraft-opencascade#10); its code is in the history before that merge.

## Building and testing

The first build downloads cadrum's prebuilt OpenCASCADE for the host. `cargo xtask ci` (also
`test`, `oracle` and `wasm`) runs the gate, which knows the backend's gaps, in three places:

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

An oracle case the backend fails is marked `known_failure` in its `expected.json`; the
robustness goldens are `tests/robustness/goldens.json`. What cadrum itself still lacks is
collected in one tracking issue, [lzpel/cadrum#317](https://github.com/lzpel/cadrum/issues/317),
re-edited as gaps turn up.

OpenCASCADE builds for wasm32 only with the wasi-sdk toolchain of cadrum's Docker image, so
`cargo xtask wasm` runs the check inside `ghcr.io/lzpel/cross-wasm32-unknown-unknown` (and says
it skipped it where docker is not available), and the web app is built with
`packaging/web/build-cadrum.sh`.

## What the backend keeps in a body

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

The backend can panic on geometry it doesn't handle (cadrum's Rust side can). Every call into it
goes through `solidcraft_kernel::guard`, which catches the panic and turns it into
`KernelError::Internal`, leaving the model unchanged:

```rust
pub fn guard<T>(what: &str, f: impl FnOnce() -> Result<T>) -> Result<T>
```

A failing feature shows an error on the timeline; it never takes the app down. An OpenCASCADE
exception inside a cadrum call that returns no `Result` ends the process instead; those are
tracked in lzpel/cadrum#317 and, until fixed, kept out of the test run by the skip list.

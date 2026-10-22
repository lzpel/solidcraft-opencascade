//! The OpenCASCADE backend of the kernel boundary, through the `cadrum` crate.
//!
//! Phase 2 of the truck → cadrum migration: primitives, measures, rigid transforms, the basic
//! booleans, tessellation and the face and edge queries work; every other operation keeps its
//! signature and answers `KernelError::TooLarge("… on the OpenCASCADE backend")`, which the
//! application shows as "not supported yet", until its phase lands.
//!
//! Every call into cadrum runs under [`guard`]: cadrum panics on a few degenerate inputs (a
//! zero rotation axis, a projection that throws), and the kernel validates those before the
//! call and catches what slips through. cadrum's types are `Send` but not `Sync` (OpenCASCADE
//! writes a shape's triangulation into the shape), so a body keeps its solids behind a mutex
//! and never clones them (a cadrum clone rebuilds the topology and changes every id).
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]
#![forbid(unsafe_code)]

mod body;
mod build;
pub mod fuzz;
pub mod loft_blend;
mod measure;
mod meshbody;
mod ops;
mod query;
mod sketch_profile;
mod step;
mod stubs;
mod tess;
#[cfg(test)]
mod tests;
mod types;

pub use solidcraft_kernel_core::par::{for_each_on_threads, map_on_threads, map_with_local_pools};
pub use solidcraft_kernel_core::{
    BodyMeasure, BooleanDiagnostics, BooleanFallback, BooleanStrategy, CapSignature, DISPLAY_ANGLE, EdgeInfo, FaceInfo, FacePaint, FaceSource,
    KernelError, MAX_TRIANGLES, MassMeasure, Paint, Result, TopoCounts, boolean_diagnostics, capture_boolean_diagnostics, guard, guarded, history,
    iges_in::iges_to_step, numbered, p21, profile, watch,
};

pub use body::{Body, DisplayShare};
pub use build::{
    box_solid, canonical_revolve_axis, cylinder, extrude, extrude_planar_regions, extrude_tapered, extrude_tapered_between, extrude_whole_sides,
    loft, loft_regions, loft_to_point, loft_with_rails, path_polyline, revolve, revolve_whole, sphere, sweep, sweep_helix, sweep_helix_mesh,
    sweep_path, sweep_path_with, sweep_with, torus,
};
pub use loft_blend::loft_blend;
pub use measure::{measure, measure_mass};
pub use meshbody::{convert_prismatic_mesh, mesh_body};
pub use ops::{
    boolean, can_join_all_touching_faces, can_join_touching_faces, compound_disjoint_extrusions, faceted_boolean, half_space, is_planar, join_apart,
    place_matrix, planar_boolean, planar_regions_boolean, split_by_plane, transform, transform_matrix, union_apart, with_attempt_budget,
};
pub use query::{
    certified_prism_equivalent, coaxial_candidate, coaxial_overlap, cylinder_end_at, cylinder_face, cylinder_face_at, edge_circle, face_surface,
    merged_topology, seam_flags, with_same_surface,
};
pub use step::{
    ExportBody, ExportProduct, ImportedBody, MAX_IGES_BYTES, StepImport, StepReading, decode_native_body, encode_native_body, iges_export_bodies,
    iges_import, native_preview_mesh, step_cache_alive, step_export, step_export_bodies, step_export_products, step_import, step_import_shared,
    step_orientation_errors, step_validate,
};
pub use stubs::*;
pub use types::*;

/// Which geometry backend this is (`solidcraft_kernel::BACKEND`).
pub const BACKEND: &str = "cadrum";
/// The producer tag of this backend's native B-rep cache (docs/design-container.md): a cache
/// another backend wrote is a miss, never misread.
pub const NATIVE_PRODUCER: &str = "solidcraft-cadrum-adapter/1";

/// One-time backend set-up before the first kernel call. On native targets cadrum needs
/// nothing; in the browser OpenCASCADE's C++ constructors must run first (phase 8 wires that
/// through the web app, which owns the one `unsafe` call it takes).
pub fn init() {}

/// The error an operation this backend does not implement yet returns: shown as "not
/// supported yet: … on the OpenCASCADE backend", like any feature that isn't there yet.
pub(crate) fn unsupported(what: &str) -> KernelError {
    KernelError::TooLarge(format!("{what} on the OpenCASCADE backend"))
}

/// A cadrum failure as the kernel's error: OpenCASCADE's own message, after the operation.
pub(crate) fn failed(what: &str, e: cadrum::Error) -> KernelError {
    KernelError::Failed(format!("{what}: {e}"))
}

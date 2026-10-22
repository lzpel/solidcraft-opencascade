//! The part of the kernel boundary that does not depend on a geometry backend: the error type
//! and the panic [`guard`], exact face and edge [`history`], face sources and paints, measures,
//! mesh bodies' triangles, the Part 21 ([`p21`]) and IGES text readers, threading and profiling
//! helpers. `solidcraft-kernel` re-exports all of it through whichever backend crate it selects
//! (`solidcraft-kernel-truck`, `solidcraft-kernel-cadrum`), so the rest of the workspace sees one
//! set of types whatever the backend.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]
#![forbid(unsafe_code)]

pub mod boolean_diagnostics;
mod error;
pub mod fuzz;
pub mod history;
pub mod iges_in;
pub mod measure;
pub mod meshbody;
pub mod native_meta;
pub mod numbered;
pub mod offset2d;
pub mod p21;
pub mod par;
pub mod profile;
pub mod source;
pub mod topo;
pub mod watch;

pub use boolean_diagnostics::{BooleanDiagnostics, BooleanFallback, BooleanStrategy, capture_boolean_diagnostics};
pub use error::{KernelError, Result, guard, guarded};
pub use history::Topology;
pub use measure::{BodyMeasure, MassMeasure};
pub use meshbody::{MAX_TRIANGLES, TriMesh};
pub use source::{CapSignature, DISPLAY_ANGLE, EdgeInfo, FaceInfo, FacePaint, FaceSource, Paint};
pub use topo::TopoCounts;

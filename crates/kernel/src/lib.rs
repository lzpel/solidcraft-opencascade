//! The B-rep kernel boundary.
//!
//! Everything SolidCraft does with solids goes through this crate: building bodies from planar
//! regions (extrude, revolve) and primitives, booleans, fillets and chamfers, tessellation,
//! measurement and STEP export and import. No backend type appears in the public API, so the
//! backend can be replaced without touching the rest of the workspace: this crate only selects
//! one by cargo feature and re-exports it.
//!
//! - `cadrum` (default): `solidcraft-kernel-cadrum`, OpenCASCADE through the `cadrum` crate
//!   (prebuilt, statically linked).
//! - `truck`: `solidcraft-kernel-truck`, the pure-Rust truck crates (Apache-2.0) plus our own
//!   operations. It wins when both features are enabled, so selecting it needs no
//!   `--no-default-features`.
//!
//! Both backends re-export `solidcraft-kernel-core` (errors, the panic [`guard`], history, face
//! sources, measures), so those types are the same whichever backend is built. Every call into
//! the third-party kernel runs under [`guard`]: a panic inside it becomes a
//! [`KernelError::Internal`] instead of taking the application down.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]
#![forbid(unsafe_code)]

#[cfg(feature = "truck")]
pub use solidcraft_kernel_truck::*;

#[cfg(all(feature = "cadrum", not(feature = "truck")))]
pub use solidcraft_kernel_cadrum::*;

#[cfg(not(any(feature = "truck", feature = "cadrum")))]
compile_error!("solidcraft-kernel needs a backend feature: `cadrum` (default) or `truck`");

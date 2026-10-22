//! The B-rep kernel boundary.
//!
//! Everything SolidCraft does with solids goes through this crate: building bodies from planar
//! regions (extrude, revolve) and primitives, booleans, fillets and chamfers, tessellation,
//! measurement and STEP export and import. No backend type appears in the public API, so the
//! backend can be replaced without touching the rest of the workspace: this crate only
//! re-exports it. The backend is `solidcraft-kernel-cadrum`: OpenCASCADE through the `cadrum`
//! crate (prebuilt, statically linked). The pure-Rust truck backend it replaced lived here until
//! the migration's last phase (lzpel/solidcraft-opencascade#10).
//!
//! The backend re-exports `solidcraft-kernel-core` (errors, the panic [`guard`], history, face
//! sources, measures), the part of the boundary that depends on no geometry library. Every call
//! into the third-party kernel runs under [`guard`]: a panic inside it becomes a
//! [`KernelError::Internal`] instead of taking the application down.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]
#![forbid(unsafe_code)]

pub use solidcraft_kernel_cadrum::*;

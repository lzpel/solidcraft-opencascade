//! This backend's view of the exact face and edge history (`solidcraft_kernel_core::history`).

pub use solidcraft_kernel_core::history::*;

/// The faces each edge separates, in the body's edge order.
pub(crate) fn edge_faces(solid: &crate::body::Solid) -> Vec<Vec<usize>> {
    crate::body::edge_order(solid.face_iter()).1
}

#[cfg(test)]
#[path = "history_tests.rs"]
mod tests;

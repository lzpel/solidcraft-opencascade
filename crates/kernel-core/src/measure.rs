//! What a body measures as: the records the backend's `measure` fills (the computation lives in each backend).

use serde::Serialize;
use solidcraft_geom::{Aabb3, Vec3};

use crate::topo::TopoCounts;

/// Measured properties of a body.
#[derive(Clone, Debug, Serialize)]
pub struct BodyMeasure {
    pub volume: f64,
    pub area: f64,
    pub centroid: Vec3,
    pub bbox: Aabb3,
    /// Raw kernel topology counts (closed curves and surfaces may be split in pieces).
    pub faces: usize,
    pub edges: usize,
    pub vertices: usize,
    pub shells: usize,
    /// Counts after merging split pieces of the same analytic surface/curve (comparable with
    /// systems that use periodic faces and edges).
    pub merged: TopoCounts,
}

/// Volume, area, centre and box of a body, as [`measure`] works them out, without the topology
/// counts (and the tessellation they take).
#[derive(Clone, Copy, Debug, Serialize)]
pub struct MassMeasure {
    pub volume: f64,
    pub area: f64,
    pub centroid: Vec3,
    pub bbox: Aabb3,
}

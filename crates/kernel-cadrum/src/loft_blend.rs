//! A loft between blend sections (surface workspace): not on this backend yet.

use solidcraft_geom::Vec3;

use crate::body::Body;
use crate::types::{Continuity, Frame};
use crate::{Result, unsupported};

/// Most sections a loft blend takes.
pub const MAX_BLEND_SECTIONS: usize = 50;
/// Fewest stations along a section.
pub const MIN_BLEND_STATIONS: usize = 2;
/// Most stations along a section.
pub const MAX_BLEND_STATIONS: usize = 257;

pub fn loft_blend(_sections: &[Vec<Frame>], _orders: [Continuity; 2], _crown: [f64; 2], _samples: usize) -> Result<Body> {
    Err(unsupported("a loft blend"))
}

/// The stations of a section along `edges`, leaving in `leave`.
pub fn section_frames(_body: &Body, _edges: &[Vec3], _leave: Vec3, _stations: usize) -> Result<Vec<Frame>> {
    Err(unsupported("a loft blend"))
}

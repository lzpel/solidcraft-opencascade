//! What the rest of the application sees of a body's faces and edges, whatever the backend:
//! where a face came from (its [`FaceSource`] certificate), how it is painted, and the plain
//! records of an edge or face.

use serde::Serialize;
use solidcraft_geom::Vec3;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum FaceSource {
    ExtrudeSide {
        segment: solidcraft_geom::Seg2,
        plane: Option<solidcraft_geom::Plane>,
        coalesced: bool,
    },
    CertifiedExtrudeSide {
        producer: u64,
        sketch: u64,
        curve: String,
        plane: Option<solidcraft_geom::Plane>,
    },
    ExtrudeCap {
        high: bool,
        region: solidcraft_geom::Region2,
        plane: solidcraft_geom::Plane,
    },
    CertifiedExtrudeCap {
        producer: u64,
        sketch: u64,
        high: bool,
        outer: Vec<String>,
        holes: Vec<Vec<String>>,
        plane: solidcraft_geom::Plane,
    },
    Box {
        producer: Option<u64>,
        axis: u8,
        high: bool,
        plane: solidcraft_geom::Plane,
    },
    /// A cylinder primitive's cap: `high` at the far end (its `top`), else at the base.
    CylinderCap {
        producer: u64,
        high: bool,
        plane: solidcraft_geom::Plane,
    },
    ShellPlanar {
        producer: Option<u64>,
        source: Box<FaceSource>,
        inner: bool,
        plane: solidcraft_geom::Plane,
    },
    Imported {
        item: u64,
        face: u64,
        payload_sha256: Option<String>,
        producer: Option<u64>,
    },
}

pub type CapSignature = (Vec<String>, Vec<Vec<String>>);

impl FaceSource {
    pub fn plane_mut(&mut self) -> Option<&mut solidcraft_geom::Plane> {
        match self {
            Self::ExtrudeSide { plane, .. } | Self::CertifiedExtrudeSide { plane, .. } => plane.as_mut(),
            Self::ExtrudeCap { plane, .. }
            | Self::CertifiedExtrudeCap { plane, .. }
            | Self::Box { plane, .. }
            | Self::CylinderCap { plane, .. }
            | Self::ShellPlanar { plane, .. } => Some(plane),
            Self::Imported { .. } => None,
        }
    }
}

/// How a body looks beyond its colour, for export: its opacity and its faces' own colours.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Paint {
    /// 1 = opaque.
    pub opacity: f32,
    pub faces: Vec<FacePaint>,
}

/// A face's own colour: the B-rep face index (`face_iter` order), sRGB as 0..1, opacity.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FacePaint {
    pub face: usize,
    pub color: [f32; 3],
    pub opacity: f32,
}

/// A B-rep edge as seen by the rest of the application.
#[derive(Clone, Debug, Serialize)]
pub struct EdgeInfo {
    pub index: usize,
    /// Polyline approximation.
    pub points: Vec<Vec3>,
    /// Point halfway along the polyline (used to re-find the edge).
    pub mid: Vec3,
    pub length: f64,
}

/// A B-rep face.
#[derive(Clone, Debug, Serialize)]
pub struct FaceInfo {
    pub index: usize,
    pub area: f64,
    pub centroid: Vec3,
    /// Unit normal if the face is planar.
    pub plane_normal: Option<Vec3>,
}

/// The most one facet of the display mesh turns (radians): a curved face or edge is divided
/// until no facet spans more of it than this, however small its radius is beside the body.
/// 20 degrees: a quarter turn is drawn with at least five facets, a full circle with at least 18.
pub const DISPLAY_ANGLE: f64 = std::f64::consts::PI / 9.0;

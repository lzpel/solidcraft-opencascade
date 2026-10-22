//! STEP and IGES exchange: the result and request types, and what this backend cannot do yet
//! (IGES export; OpenCASCADE's IGES writer is not reached through cadrum).

use crate::body::Body;
use crate::types::{StepHeader, StepNode};
use crate::{Result, unsupported};

pub use crate::step_in::{StepReading, iges_import, step_cache_alive, step_import, step_import_shared, step_orientation_errors, step_validate};

pub use solidcraft_kernel_core::iges_in::MAX_IGES_BYTES;

/// A body read from a STEP file.
#[derive(Clone, Debug)]
pub struct ImportedBody {
    pub source_item: u64,
    pub face_origin: Vec<u64>,
    pub name: String,
    pub body: Body,
    pub color: Option<[f32; 3]>,
    pub path: Vec<String>,
    pub instance: Option<String>,
    pub closed: bool,
    pub file_faces: usize,
}

/// Result of [`step_import`].
#[derive(Clone, Debug)]
pub struct StepImport {
    pub bodies: Vec<ImportedBody>,
    pub tree: Vec<StepNode>,
    pub warnings: Vec<String>,
    pub schema: String,
    pub system: String,
}

/// A body to export with its name and colour.
pub struct ExportBody<'a> {
    pub name: String,
    pub body: &'a Body,
    pub color: Option<[f32; 3]>,
}

/// A product to export: its bodies and child products with their placements.
pub struct ExportProduct<'a> {
    pub name: String,
    pub bodies: Vec<ExportBody<'a>>,
    pub children: Vec<(usize, [[f64; 4]; 4], String)>,
}

/// How many shared imports are alive, and how many bodies they hold.
pub fn iges_export_bodies(_bodies: &[ExportBody], _header: &StepHeader) -> Result<String> {
    Err(unsupported("IGES export"))
}

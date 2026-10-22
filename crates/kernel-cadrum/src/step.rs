//! STEP and IGES exchange and the native B-rep cache: phase 6. The types are here so the io
//! and engine crates compile; every function answers "not supported yet" until then.

use std::sync::Arc;

use solidcraft_geom::Mesh;

use crate::body::Body;
use crate::types::{StepHeader, StepNode};
use crate::{Result, unsupported};

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

/// A STEP file read one body at a time.
pub struct StepReading {
    done: bool,
}

impl StepReading {
    pub fn new(_text: &str) -> Result<StepReading> {
        Err(unsupported("STEP import"))
    }

    /// Read the next body; false when there is none.
    pub fn step(&mut self) -> bool {
        self.done = true;
        false
    }

    /// Bodies converted so far, and how many there are.
    pub fn progress(&self) -> (usize, usize) {
        (usize::from(self.done), 0)
    }

    /// The import, once every solid is read.
    pub fn finish(self) -> Result<StepImport> {
        Err(unsupported("STEP import"))
    }

    /// The import, shared with later readers of the same text.
    pub fn finish_shared(self) -> Result<Arc<StepImport>> {
        Err(unsupported("STEP import"))
    }
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

pub fn step_import(_text: &str) -> Result<StepImport> {
    Err(unsupported("STEP import"))
}

pub fn step_import_shared(_text: &str) -> Result<Arc<StepImport>> {
    Err(unsupported("STEP import"))
}

/// How many shared imports are alive, and how many bodies they hold.
pub fn step_cache_alive() -> (usize, usize) {
    (0, 0)
}

pub fn step_validate(_text: &str) -> std::result::Result<usize, String> {
    Err("not supported yet: STEP validation on the OpenCASCADE backend".into())
}

pub fn step_orientation_errors(_text: &str) -> std::result::Result<Vec<String>, String> {
    Err("not supported yet: STEP validation on the OpenCASCADE backend".into())
}

pub fn step_export(_bodies: &[&Body], _system: &str) -> Result<String> {
    Err(unsupported("STEP export"))
}

pub fn step_export_bodies(_bodies: &[ExportBody], _header: &StepHeader) -> Result<String> {
    Err(unsupported("STEP export"))
}

pub fn step_export_products(_products: &[ExportProduct], _root: usize, _header: &StepHeader) -> Result<String> {
    Err(unsupported("STEP export"))
}

pub fn iges_import(_text: &str) -> Result<StepImport> {
    Err(unsupported("IGES import"))
}

pub fn iges_export_bodies(_bodies: &[ExportBody], _header: &StepHeader) -> Result<String> {
    Err(unsupported("IGES export"))
}

pub fn encode_native_body(_body: &Body) -> Result<Vec<u8>> {
    Err(unsupported("the native B-rep cache"))
}

pub fn decode_native_body(_bytes: &[u8]) -> Result<Body> {
    Err(unsupported("the native B-rep cache"))
}

pub fn native_preview_mesh(_body: &Body, _detail: usize, _remaining: &mut usize) -> Result<Mesh> {
    Err(unsupported("the native B-rep cache"))
}

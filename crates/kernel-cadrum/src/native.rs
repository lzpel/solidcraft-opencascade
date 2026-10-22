//! SolidCraft's native, versioned binary geometry for this backend: OpenCASCADE's own binary
//! B-rep (`BinTools`) beside the body's metadata (colour, paint, split lines, face sources,
//! history) and the values it measured about itself. Exact: a decoded body is the encoded one,
//! number for number, for the same producer; the other backend's records are a cache miss.
//!
//! Layout: `SCBREP05`, the metadata JSON's length (u32 LE), the metadata JSON, the B-rep bytes.

use std::sync::Arc;

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Mesh, brep::envelope as w};
use solidcraft_kernel_core::native_meta::{self as meta, Fields, invalid};

use crate::body::Body;
use crate::{DISPLAY_ANGLE, Result, guard};

const MAGIC: &[u8; 8] = b"SCBREP05";
/// A body's own limits: one large part (an imported casting) fits, a design's cache holds many.
const MAX_BYTES: usize = 256 << 20;
const MAX_METADATA_BYTES: usize = 64 << 20;
const MAX_ITEMS: usize = 1_000_000;

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Saved {
    metadata: w::Metadata,
    runtime: Runtime,
}

/// What a body measured about itself that later operations read.
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Runtime {
    /// [`Body::size`] as stored (0: not measured).
    size: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    boundary_tol: Option<f64>,
}

/// Explicit topology, geometry, construction certificates and history, exactly. Failure is a
/// cache miss at the IO boundary, never a failed document save.
pub fn encode_native_body(body: &Body) -> Result<Vec<u8>> {
    guard("encode native B-rep", || {
        if body.is_mesh() {
            return Err(invalid("mesh bodies are source blobs, not B-rep"));
        }
        let inner = body.inner()?;
        if inner.faces.len() > MAX_ITEMS || inner.edges.len() > MAX_ITEMS {
            return Err(invalid("native geometry budget"));
        }
        let metadata = meta::encode(&Fields {
            color: body.color,
            paint: body.paint.as_deref().cloned(),
            splits: body.splits.as_deref().cloned(),
            face_sources: body.face_sources.as_deref().cloned(),
            history: body.history.as_deref().cloned(),
            preserve_boundaries: body.regularization_protected,
        })?;
        let saved = Saved { metadata, runtime: Runtime { size: body.size, boundary_tol: body.boundary_tol } };
        let json = serde_json::to_vec(&saved).map_err(invalid)?;
        if json.len() > MAX_METADATA_BYTES {
            return Err(invalid("metadata budget"));
        }
        // Written from fresh copies: a copy carries no triangulation (lzpel/cadrum#317 item
        // 24), so the bytes hold the geometry alone, the same whether or not the body was
        // meshed, and the decoded shape's box is not that of a display mesh.
        let brep = inner.with("encode native B-rep", |solids| {
            let fresh: Vec<cadrum::Solid> = solids.iter().map(crate::body::cadrum_handle).collect();
            let mut out = Vec::new();
            cadrum::Solid::write_brep(fresh.iter(), &mut out).map_err(invalid)?;
            Ok(out)
        })?;
        let mut out = MAGIC.to_vec();
        out.extend_from_slice(&u32::try_from(json.len()).map_err(invalid)?.to_le_bytes());
        out.extend_from_slice(&json);
        out.extend_from_slice(&brep);
        if out.len() > MAX_BYTES {
            return Err(invalid("encoding size"));
        }
        Ok(out)
    })
}

/// Decode bounded untrusted data: the metadata is validated against the geometry before the
/// body is made. Only a caller that has checked who produced the data may treat it as
/// evaluated geometry.
pub fn decode_native_body(bytes: &[u8]) -> Result<Body> {
    if bytes.len() > MAX_BYTES || bytes.get(..8) != Some(MAGIC.as_slice()) {
        return Err(invalid("version or size"));
    }
    guard("decode native B-rep", || {
        let len: [u8; 4] = bytes.get(8..12).and_then(|b| b.try_into().ok()).ok_or_else(|| invalid("header"))?;
        let len = usize::try_from(u32::from_le_bytes(len)).map_err(invalid)?;
        if len > MAX_METADATA_BYTES {
            return Err(invalid("metadata budget"));
        }
        let json = bytes.get(12..12 + len).ok_or_else(|| invalid("truncated metadata"))?;
        let brep = bytes.get(12 + len..).ok_or_else(|| invalid("truncated geometry"))?;
        if brep.is_empty() {
            return Err(invalid("no geometry"));
        }
        let saved: Saved = serde_json::from_slice(json).map_err(invalid)?;
        let fields = meta::decode(saved.metadata)?;
        let solids = cadrum::Solid::read_brep(&mut &*brep).map_err(invalid)?;
        if solids.is_empty() {
            return Err(invalid("no solids"));
        }
        let mut body = Body::from_solids(solids)?;
        let (faces, edges) = (body.face_count(), body.edge_count());
        if faces > MAX_ITEMS || edges > MAX_ITEMS {
            return Err(invalid("native geometry budget"));
        }
        if fields.face_sources.as_ref().is_some_and(|s| s.len() != faces) {
            return Err(invalid("face sources do not fit the faces"));
        }
        if let Some(h) = &fields.history
            && (h.faces.len() != faces || h.edges.len() != edges || h.edges.iter().any(|e| e.faces.iter().any(|f| *f >= faces)))
        {
            return Err(invalid("history does not fit the topology"));
        }
        if let Some(p) = &fields.paint
            && p.faces.iter().any(|f| f.face >= faces)
        {
            return Err(invalid("paint names a missing face"));
        }
        if saved.runtime.size.is_finite() && saved.runtime.size > 0.0 {
            body.size = saved.runtime.size;
        }
        body.boundary_tol = saved.runtime.boundary_tol.filter(|t| t.is_finite() && *t > 0.0);
        body.color = fields.color;
        body.paint = fields.paint.map(Arc::new);
        body.splits = fields.splits.map(Arc::new);
        body.face_sources = fields.face_sources.map(Arc::new);
        body.history = fields.history.map(Arc::new);
        body.regularization_protected = fields.preserve_boundaries;
        Ok(body)
    })
}

/// Approximate read-only display of a decoded body, within a triangle budget shared by the
/// bodies of a design: `detail` 1 (coarse) to 8 (fine); `remaining` is the budget left.
pub fn native_preview_mesh(body: &Body, detail: usize, remaining: &mut usize) -> Result<Mesh> {
    if body.is_mesh() {
        return Err(invalid("native preview requires B-rep"));
    }
    let detail = detail.clamp(1, 8) as f64;
    let tol = (body.size().max(1e-3) * 2e-2 / detail).max(1e-4);
    let mesh = crate::tess::mesh(body, tol, DISPLAY_ANGLE)?;
    let triangles = mesh.triangles.len();
    if triangles > *remaining {
        return Err(invalid("preview budget"));
    }
    *remaining -= triangles;
    Ok(Mesh { positions: mesh.positions.clone(), normals: mesh.normals.clone(), triangles: mesh.triangles.clone(), ..Default::default() })
}

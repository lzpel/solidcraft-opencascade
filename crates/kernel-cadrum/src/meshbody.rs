//! Mesh bodies: triangles without a B-rep (imported 3MF/STL), the same on every backend.

use std::collections::HashMap;

use solidcraft_geom::Vec3;
use solidcraft_kernel_core::meshbody::{MAX_TRIANGLES, TriMesh, is_closed};

use crate::body::Body;
use crate::{KernelError, Result, unsupported};

/// A mesh body from shared-vertex triangles. Exactly equal vertices are merged; an inside-out
/// closed mesh is turned right side out.
pub fn mesh_body(positions: &[Vec3], triangles: &[[u32; 3]]) -> Result<Body> {
    if triangles.len() > MAX_TRIANGLES {
        return Err(KernelError::Invalid(format!("mesh with {} triangles (limit {MAX_TRIANGLES})", triangles.len())));
    }
    if positions.iter().any(|p| !(p.is_finite() && p.x.abs() < 1e9 && p.y.abs() < 1e9 && p.z.abs() < 1e9)) {
        return Err(KernelError::Invalid("mesh with non-finite or huge coordinates".into()));
    }
    let mut map: HashMap<[u64; 3], u32> = HashMap::new();
    let mut remap: Vec<u32> = Vec::with_capacity(positions.len());
    let mut out = TriMesh::default();
    for p in positions {
        let key = [(p.x + 0.0).to_bits(), (p.y + 0.0).to_bits(), (p.z + 0.0).to_bits()];
        let next = u32::try_from(out.positions.len()).map_err(|_| KernelError::Invalid("too many mesh vertices".into()))?;
        let idx = *map.entry(key).or_insert_with(|| {
            out.positions.push(*p);
            next
        });
        remap.push(idx);
    }
    for t in triangles {
        let mut tri = [0u32; 3];
        for (slot, i) in tri.iter_mut().zip(t) {
            *slot = *remap.get(*i as usize).ok_or_else(|| KernelError::Invalid("mesh triangle index out of range".into()))?;
        }
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
            continue;
        }
        out.triangles.push(tri);
    }
    if out.triangles.is_empty() {
        return Err(KernelError::Invalid("mesh with no triangles".into()));
    }
    if out.volume6() < 0.0 && is_closed(&out) {
        for t in &mut out.triangles {
            t.swap(1, 2);
        }
    }
    Ok(Body::from_mesh(out))
}

impl Body {
    /// Shared-vertex triangles of a mesh body.
    pub fn triangle_mesh(&self) -> Option<(&[Vec3], &[[u32; 3]])> {
        self.tri_mesh().map(|m| (m.positions.as_slice(), m.triangles.as_slice()))
    }

    /// Does a mesh body enclose a volume (every side shared by two triangles)?
    pub fn is_closed_mesh(&self) -> bool {
        self.tri_mesh().is_some_and(|m| is_closed(m))
    }
}

/// A prismatic mesh body as a B-rep: not on this backend yet.
pub fn convert_prismatic_mesh(_body: &Body, _tolerance: f64) -> Result<Body> {
    Err(unsupported("converting a mesh body to a B-rep"))
}

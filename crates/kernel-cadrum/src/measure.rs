//! Measures: volume, area and centroid are OpenCASCADE's exact ones (global properties of the
//! B-rep), the box is the solids' bounding box, and the counts come from the body's snapshot.
//! OpenCASCADE keeps a cylinder's wall as one periodic face, so the raw counts already are
//! what the truck backend reported as `merged`.

use std::collections::BTreeMap;

use solidcraft_geom::{Aabb3, Vec3};

use crate::body::{Body, Shape, v3};
use crate::{BodyMeasure, KernelError, MassMeasure, Result, TopoCounts};

/// Measure a body.
pub fn measure(b: &Body) -> Result<BodyMeasure> {
    let (mass, raw, merged) = mass(b)?;
    Ok(BodyMeasure {
        volume: mass.volume,
        area: mass.area,
        centroid: mass.centroid,
        bbox: mass.bbox,
        faces: raw.0,
        edges: raw.1,
        vertices: raw.2,
        shells: b.shell_count(),
        merged,
    })
}

/// Volume, area, centre and box only.
pub fn measure_mass(b: &Body) -> Result<MassMeasure> {
    Ok(mass(b)?.0)
}

/// The mass measure, the raw (faces, edges, vertices) counts and the merged counts.
fn mass(b: &Body) -> Result<(MassMeasure, (usize, usize, usize), TopoCounts)> {
    match &b.shape {
        Shape::Mesh(m) => {
            let mesh = m.to_mesh();
            let mut bbox = Aabb3::EMPTY;
            for p in &mesh.positions {
                bbox.add(*p);
            }
            let origin = bbox.center();
            let (mut volume, mut area, mut moment) = (0.0, 0.0, Vec3::ZERO);
            for t in &mesh.triangles {
                let Some([p, q, r]) = mesh.tri(t) else { continue };
                let (a, bb, c) = (p - origin, q - origin, r - origin);
                let v6 = a.dot(bb.cross(c));
                volume += v6 / 6.0;
                moment += (a + bb + c) * (v6 / 24.0);
                area += (q - p).cross(r - p).len() * 0.5;
            }
            let centroid = if volume.abs() > 0.0 { origin + moment * (1.0 / volume) } else { origin };
            let counts = TopoCounts {
                faces: mesh.triangles.len(),
                edges: m.edge_count(),
                vertices: m.positions.len(),
                face_types: [("mesh".to_string(), mesh.triangles.len())].into_iter().collect(),
            };
            Ok((MassMeasure { volume: volume.abs(), area, centroid, bbox }, (counts.faces, counts.edges, counts.vertices), counts))
        }
        Shape::Solid(inner) => {
            let (volume, area, centroid, kinds) = inner.with("measure", |solids| {
                let (mut volume, mut area, mut weighted) = (0.0, 0.0, Vec3::ZERO);
                for s in solids {
                    let v = s.volume();
                    volume += v;
                    area += s.area();
                    weighted += v3(s.center()) * v;
                }
                let centroid = if volume > 0.0 { weighted * (1.0 / volume) } else { inner.bbox.center() };
                let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
                for i in 0..inner.faces.len() {
                    let kind = match inner.face(solids, i).and_then(|f| f.surface()).map(|s| s.kind) {
                        Some(cadrum::SurfaceKind::Plane) => "plane",
                        Some(cadrum::SurfaceKind::Cylinder { .. }) => "cylinder",
                        Some(cadrum::SurfaceKind::Cone { .. }) => "cone",
                        Some(cadrum::SurfaceKind::Sphere { .. }) => "sphere",
                        Some(cadrum::SurfaceKind::Torus { .. }) => "torus",
                        None => "other",
                    };
                    *kinds.entry(kind.to_string()).or_default() += 1;
                }
                Ok((volume, area, centroid, kinds))
            })?;
            if !(volume.is_finite() && area.is_finite() && centroid.is_finite()) {
                return Err(KernelError::Failed("the measure is not finite".into()));
            }
            let _ = inner.volume.set(volume);
            let mut counts = crate::query::merged_counts(b);
            counts.face_types = kinds;
            // OpenCASCADE's box around a curved face is loose (a trimmed quadric's reaches to its
            // full extent, a free-form face's to its control points): a body with one is
            // measured by its fine tessellation, when that fits the mesh budget.
            let curved = counts.face_types.iter().any(|(kind, n)| kind != "plane" && *n > 0);
            let bbox = if curved { tight_bbox(b, inner).unwrap_or(inner.bbox) } else { inner.bbox };
            Ok((MassMeasure { volume, area, centroid, bbox }, (inner.faces.len(), inner.edges.len(), inner.vertices), counts))
        }
    }
}

/// The body's box, each side OpenCASCADE's where that is tight (an untrimmed quadric's is
/// exact) and the fine tessellation's where OpenCASCADE's reaches beyond it (a trimmed
/// quadric's to its full extent, a free-form face's to its control points).
fn tight_bbox(b: &Body, inner: &crate::body::Inner) -> Result<Aabb3> {
    let size = if inner.bbox.is_empty() { 1.0 } else { inner.bbox.diagonal().max(1e-9) };
    let deflection = size * 1e-4;
    let m = crate::tess::mesh(b, deflection, crate::DISPLAY_ANGLE)?;
    let mut mesh = Aabb3::EMPTY;
    for p in &m.positions {
        mesh.add(*p);
    }
    if mesh.is_empty() {
        return Ok(inner.bbox);
    }
    let pick = |bnd: f64, tess: f64| if (bnd - tess).abs() <= 4.0 * deflection { bnd } else { tess };
    let side = |bnd: Vec3, tess: Vec3| Vec3::new(pick(bnd.x, tess.x), pick(bnd.y, tess.y), pick(bnd.z, tess.z));
    Ok(Aabb3 { min: side(inner.bbox.min, mesh.min), max: side(inner.bbox.max, mesh.max) })
}

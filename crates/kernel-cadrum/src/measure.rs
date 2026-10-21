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
    let (mass, counts) = mass(b)?;
    Ok(BodyMeasure {
        volume: mass.volume,
        area: mass.area,
        centroid: mass.centroid,
        bbox: mass.bbox,
        faces: counts.faces,
        edges: counts.edges,
        vertices: counts.vertices,
        shells: b.shell_count(),
        merged: counts,
    })
}

/// Volume, area, centre and box only.
pub fn measure_mass(b: &Body) -> Result<MassMeasure> {
    Ok(mass(b)?.0)
}

fn mass(b: &Body) -> Result<(MassMeasure, TopoCounts)> {
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
            Ok((MassMeasure { volume: volume.abs(), area, centroid, bbox }, counts))
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
            let counts = TopoCounts { faces: inner.faces.len(), edges: inner.edges.len(), vertices: inner.vertices, face_types: kinds };
            Ok((MassMeasure { volume, area, centroid, bbox: inner.bbox }, counts))
        }
    }
}

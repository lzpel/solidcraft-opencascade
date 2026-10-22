//! Tessellation: cadrum meshes the solids (OpenCASCADE's incremental mesher, one vertex set
//! per face with surface normals, which is the kernel mesh's contract already), and the kernel
//! adds what the application reads by index: the triangle's face, the edges as polylines in
//! edge order, which faces each edge bounds, seams and smooth joins.

use std::sync::Arc;

use solidcraft_geom::tess::{FacePlan, TessPlan};
use solidcraft_geom::{Mesh, Vec3};

use crate::body::{Body, Shape, dv, v3};
use crate::{DISPLAY_ANGLE, EdgeInfo, FaceInfo, KernelError, Result, failed};

/// The finest angle a display mesh is divided by (one degree).
const MIN_DISPLAY_ANGLE: f64 = std::f64::consts::PI / 180.0;
/// The angle a reference mesh (measures, contact tests) is divided by at most.
const REFERENCE_ANGLE: f64 = 0.5;
/// Most triangles one mesh may have.
const MAX_MESH_TRIANGLES: usize = 40_000_000;

fn tessellation(tol: f64, angle: f64) -> cadrum::Tessellation {
    cadrum::Tessellation { deflection_linear: tol, deflection_angular: angle, relative_linear: false }
}

/// A mesh of `body` within chord tolerance `tol` and `angle` per facet.
pub(crate) fn mesh(body: &Body, tol: f64, angle: f64) -> Result<Arc<Mesh>> {
    if !(tol.is_finite() && tol > 0.0) {
        return Err(KernelError::Invalid(format!("tessellation tolerance {tol}")));
    }
    let tol = tol.max(body.size() * 1e-7).max(1e-9);
    let angle = if angle.is_finite() && angle > 0.0 { angle.clamp(MIN_DISPLAY_ANGLE, std::f64::consts::PI) } else { REFERENCE_ANGLE };
    let inner = match &body.shape {
        Shape::Mesh(m) => return Ok(Arc::new(m.to_mesh())),
        Shape::Solid(inner) => inner,
    };
    let key = (tol.to_bits(), angle.to_bits());
    if let Some(m) = inner.memo(key) {
        return Ok(m);
    }
    let made = inner.with("tessellate", |solids| {
        let m = cadrum::Solid::mesh(solids.iter(), tessellation(tol, angle)).map_err(|e| failed("tessellate", e))?;
        if m.indices.len() / 3 > MAX_MESH_TRIANGLES {
            return Err(KernelError::TooLarge(format!("a mesh of {} triangles", m.indices.len() / 3)));
        }
        let mut out = Mesh {
            positions: m.vertices.iter().map(|v| v3(*v)).collect(),
            normals: m.normals.iter().map(|n| v3(*n)).collect(),
            ..Default::default()
        };
        if out.normals.len() != out.positions.len() {
            out.normals = vec![Vec3::Z; out.positions.len()];
        }
        let n = out.positions.len();
        for (k, &[a, b, c]) in m.indices.as_chunks::<3>().0.iter().enumerate() {
            if a >= n || b >= n || c >= n {
                continue;
            }
            let Some(face) = m.face_ids.get(k).and_then(|id| inner.face_index.get(id)) else { continue };
            let (Ok(a), Ok(b), Ok(c), Ok(f)) = (u32::try_from(a), u32::try_from(b), u32::try_from(c), u32::try_from(*face)) else { continue };
            out.triangles.push([a, b, c]);
            out.tri_face.push(f);
        }
        // Edges in the kernel's edge order, each as its own polyline.
        let mut by_id = std::collections::HashMap::new();
        for s in solids {
            for e in s.iter_edge() {
                by_id.insert(e.id(), e);
            }
        }
        let edge_tess = tessellation(tol, angle);
        for id in &inner.edges {
            let points: Vec<Vec3> = by_id.get(id).map(|e| e.approximation_segments(edge_tess).into_iter().map(v3).collect()).unwrap_or_default();
            out.edges.push(points);
        }
        out.edge_faces = inner.edge_faces.iter().map(|fs| fs.iter().filter_map(|f| u32::try_from(*f).ok()).collect()).collect();
        // A seam is an edge with one face on both sides (a cylinder's wall meets itself there); a
        // smooth join is where the two faces' normals agree.
        let cos_smooth = (1.0_f64).to_radians().cos();
        for (ei, fs) in inner.edge_faces.iter().enumerate() {
            let mid = out.edges.get(ei).and_then(|pts| polyline_mid(pts)).map(|(m, _)| m);
            let seam = fs.len() == 1;
            let smooth = match (fs.first(), fs.get(1), mid) {
                (Some(&fa), Some(&fb), Some(mid)) if fa != fb => {
                    let normal = |f: usize| inner.face(solids, f).map(|face| v3(face.project(dv(mid)).1)).and_then(|n| n.normalized());
                    match (normal(fa), normal(fb)) {
                        (Some(na), Some(nb)) => na.dot(nb) > cos_smooth,
                        _ => false,
                    }
                }
                _ => seam,
            };
            out.seams.push(seam);
            out.smooth_edges.push(smooth);
            out.sharp_edges.push(false);
        }
        Ok(Arc::new(out))
    })?;
    inner.remember(key, made.clone());
    Ok(made)
}

/// The point halfway along a polyline and its length.
pub(crate) fn polyline_mid(pts: &[Vec3]) -> Option<(Vec3, f64)> {
    let first = *pts.first()?;
    if pts.len() == 1 {
        return Some((first, 0.0));
    }
    let total: f64 = pts.windows(2).map(|w| (w[1] - w[0]).len()).sum();
    let mut run = 0.0;
    for w in pts.windows(2) {
        let l = (w[1] - w[0]).len();
        if run + l >= total / 2.0 {
            let t = if l > 0.0 { (total / 2.0 - run) / l } else { 0.0 };
            return Some((w[0] + (w[1] - w[0]) * t, total));
        }
        run += l;
    }
    Some((*pts.last()?, total))
}

/// A display plan that carries the mesh as it is: every position and normal given, nothing for
/// the GPU to evaluate (cadrum exposes no NURBS data yet).
pub(crate) fn plan_of(mesh: &Mesh, face_count: usize) -> TessPlan {
    let mut faces: Vec<FacePlan> =
        (0..face_count).map(|f| FacePlan { face: u32::try_from(f).unwrap_or(u32::MAX), surface: None, ..FacePlan::default() }).collect();
    let mut remap: Vec<std::collections::HashMap<u32, u32>> = vec![std::collections::HashMap::new(); face_count];
    for (t, f) in mesh.triangles.iter().zip(&mesh.tri_face) {
        let fi = *f as usize;
        let (Some(plan), Some(map)) = (faces.get_mut(fi), remap.get_mut(fi)) else { continue };
        let mut idx = [0u32; 3];
        for (slot, &v) in idx.iter_mut().zip(t) {
            let next = u32::try_from(plan.fixed.len()).unwrap_or(u32::MAX);
            let local = *map.entry(v).or_insert_with(|| {
                plan.fixed.push(mesh.positions.get(v as usize).copied().unwrap_or(Vec3::ZERO));
                plan.normals.push(mesh.normals.get(v as usize).copied().unwrap_or(Vec3::Z));
                plan.uv.push([0.0, 0.0]);
                next
            });
            *slot = local;
        }
        plan.triangles.push(idx);
    }
    TessPlan { surfaces: Vec::new(), faces, edges: mesh.edges.clone(), edge_faces: mesh.edge_faces.clone() }
}

impl Body {
    /// A mesh within chord tolerance `tol` (the reference mesh: measures, contact tests).
    pub fn tessellate(&self, tol: f64) -> Result<Mesh> {
        Ok((*mesh(self, tol, REFERENCE_ANGLE)?).clone())
    }

    /// [`Body::tessellate`]; cadrum meshes within the deflection already, so `refine` changes nothing.
    pub fn tessellate_with(&self, tol: f64, _refine: bool) -> Result<Mesh> {
        self.tessellate(tol)
    }

    /// The display plan at chord tolerance `tol` and the display angle.
    pub fn display_plan(&self, tol: f64) -> Result<TessPlan> {
        self.display_plan_angular(tol, DISPLAY_ANGLE)
    }

    pub fn display_plan_angular(&self, tol: f64, angle: f64) -> Result<TessPlan> {
        let m = mesh(self, tol, angle)?;
        Ok(plan_of(&m, self.face_count()))
    }

    /// Faces whose boundary loops cross themselves in the parameter plane: cadrum reports none.
    pub fn crossed_face_loops(&self) -> Result<usize> {
        Ok(0)
    }

    pub fn crossed_faces(&self) -> Result<Vec<Vec3>> {
        Ok(Vec::new())
    }

    /// Faces the mesher left without triangles.
    pub fn unmeshed_faces(&self) -> usize {
        let Ok(m) = mesh(self, (self.size() * 1e-3).max(1e-3), DISPLAY_ANGLE) else { return self.face_count() };
        let mut seen = vec![false; self.face_count()];
        for f in &m.tri_face {
            if let Some(s) = seen.get_mut(*f as usize) {
                *s = true;
            }
        }
        seen.iter().filter(|s| !**s).count()
    }

    /// The display mesh: chord tolerance `tol`, facets turning at most [`DISPLAY_ANGLE`].
    pub fn display_mesh(&self, tol: f64) -> Result<Mesh> {
        self.display_mesh_angular(tol, DISPLAY_ANGLE)
    }

    pub fn display_mesh_angular(&self, tol: f64, angle: f64) -> Result<Mesh> {
        if let Some((shared, t)) = self.display_of.as_deref()
            && let Ok(m) = shared.original.display_mesh_angular(tol, angle)
            && let Some(placed) = placed_mesh(&m, t)
        {
            return Ok(placed);
        }
        Ok((*mesh(self, tol, angle)?).clone())
    }

    /// The mesh a display plan evaluates to.
    pub fn display_mesh_of(&self, plan: &TessPlan) -> Result<Mesh> {
        Ok(plan.evaluate())
    }

    /// The display mesh made the reference way (no plan): the same mesh on this backend.
    pub fn reference_display_mesh_angular(&self, tol: f64, angle: f64) -> Result<Mesh> {
        Ok((*mesh(self, tol, angle)?).clone())
    }

    /// Edges as polylines within `tol`, with the point halfway along each.
    pub fn edges(&self, tol: f64) -> Result<Vec<EdgeInfo>> {
        if self.is_mesh() {
            return Ok(Vec::new());
        }
        let m = mesh(self, tol, DISPLAY_ANGLE)?;
        Ok(m.edges
            .iter()
            .enumerate()
            .map(|(index, points)| {
                let (mid, length) = polyline_mid(points).unwrap_or((Vec3::ZERO, 0.0));
                EdgeInfo { index, points: points.clone(), mid, length }
            })
            .collect())
    }

    /// Edges in `plane`'s coordinates, exactly: not read from cadrum yet, so none.
    pub fn planar_edges(&self, _plane: &solidcraft_geom::Plane, indices: &[usize]) -> Vec<Option<Vec<solidcraft_geom::Seg2>>> {
        vec![None; indices.len()]
    }

    /// Faces with area and centroid (exact), and the normal of a planar face.
    pub fn faces(&self, _tol: f64) -> Result<Vec<FaceInfo>> {
        let inner = match &self.shape {
            Shape::Mesh(m) => {
                let mesh = m.to_mesh();
                let (mut area, mut centroid) = (0.0, Vec3::ZERO);
                for t in &mesh.triangles {
                    if let Some([a, b, c]) = mesh.tri(t) {
                        let ar = (b - a).cross(c - a).len() * 0.5;
                        area += ar;
                        centroid += (a + b + c) * (ar / 3.0);
                    }
                }
                return Ok(vec![FaceInfo {
                    index: 0,
                    area,
                    centroid: if area > 0.0 { centroid * (1.0 / area) } else { Vec3::ZERO },
                    plane_normal: None,
                }]);
            }
            Shape::Solid(inner) => inner,
        };
        inner.with("faces", |solids| {
            Ok((0..inner.faces.len())
                .map(|index| {
                    let Some(face) = inner.face(solids, index) else {
                        return FaceInfo { index, area: 0.0, centroid: Vec3::ZERO, plane_normal: None };
                    };
                    let centroid = v3(face.center());
                    let plane_normal = match face.surface() {
                        Some(s) if s.kind == cadrum::SurfaceKind::Plane => v3(face.project(dv(centroid)).1).normalized(),
                        _ => None,
                    };
                    FaceInfo { index, area: face.area(), centroid, plane_normal }
                })
                .collect())
        })
    }

    /// The edge nearest `p` and its distance, over the edges' polylines within `tol`.
    pub fn nearest_edge(&self, p: Vec3, tol: f64) -> Result<Option<(usize, f64)>> {
        if self.is_mesh() || !p.is_finite() {
            return Ok(None);
        }
        let m = mesh(self, tol, DISPLAY_ANGLE)?;
        let mut best: Option<(usize, f64)> = None;
        for (i, pts) in m.edges.iter().enumerate() {
            let d = if pts.len() < 2 {
                pts.first().map(|q| (*q - p).len())
            } else {
                pts.windows(2).map(|w| segment_distance(p, w[0], w[1])).fold(None, |m: Option<f64>, d| Some(m.map_or(d, |m| m.min(d))))
            };
            if let Some(d) = d
                && best.is_none_or(|(_, b)| d < b)
            {
                best = Some((i, d));
            }
        }
        Ok(best)
    }
}

fn segment_distance(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let l2 = ab.dot(ab);
    let t = if l2 > 0.0 { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
    (a + ab * t - p).len()
}

/// `m` placed by the rigid matrix `t` (by columns, translation in `t[3]`): positions, normals
/// and edge polylines move, everything indexed stays. `None` for anything but a rotation and
/// translation.
pub(crate) fn placed_mesh(m: &Mesh, t: &[[f64; 4]; 4]) -> Option<Mesh> {
    let r = |v: Vec3, w: f64| {
        Vec3::new(
            t[0][0] * v.x + t[1][0] * v.y + t[2][0] * v.z + w * t[3][0],
            t[0][1] * v.x + t[1][1] * v.y + t[2][1] * v.z + w * t[3][1],
            t[0][2] * v.x + t[1][2] * v.y + t[2][2] * v.z + w * t[3][2],
        )
    };
    let det = t[0][0] * (t[1][1] * t[2][2] - t[1][2] * t[2][1]) - t[0][1] * (t[1][0] * t[2][2] - t[1][2] * t[2][0])
        + t[0][2] * (t[1][0] * t[2][1] - t[1][1] * t[2][0]);
    if !(det > 0.0 && (det - 1.0).abs() < 1e-6) || t.iter().flatten().any(|x| !x.is_finite()) {
        return None;
    }
    Some(Mesh {
        positions: m.positions.iter().map(|p| r(*p, 1.0)).collect(),
        normals: m.normals.iter().map(|n| r(*n, 0.0)).collect(),
        edges: m.edges.iter().map(|e| e.iter().map(|p| r(*p, 1.0)).collect()).collect(),
        ..m.clone()
    })
}

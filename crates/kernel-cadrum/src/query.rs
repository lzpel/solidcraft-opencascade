//! Questions about faces and edges: the elementary surface a face lies on, circular edges,
//! cylindrical faces, merged topology counts and seams.

use solidcraft_geom::{Mesh, UnitVec3, Vec3};

use crate::body::{Body, dv, v3};
use crate::types::{CoaxialOverlap, CylinderFace, EdgeCircle, FaceSurface};
use crate::{Result, TopoCounts};

/// The surface face `face` lies on, when it is an elementary one. A plane's normal points out
/// of the body.
pub fn face_surface(b: &Body, face: usize) -> Option<FaceSurface> {
    let inner = b.inner().ok()?;
    inner
        .with("face surface", |solids| {
            let Some(f) = inner.face(solids, face) else { return Ok(None) };
            let Some(s) = f.surface() else { return Ok(None) };
            let axis = UnitVec3::new(v3(s.axis_z));
            Ok(match s.kind {
                cadrum::SurfaceKind::Plane => {
                    let center = v3(f.center());
                    let n = v3(f.project(dv(center)).1);
                    UnitVec3::new(n).map(|normal| FaceSurface::Plane { normal, point: center })
                }
                cadrum::SurfaceKind::Cylinder { radius } => axis.map(|axis| FaceSurface::Cylinder { origin: v3(s.origin), axis, radius }),
                cadrum::SurfaceKind::Cone { radius, semi_angle } => axis.and_then(|axis| {
                    let t = semi_angle.tan();
                    if !(t.is_finite() && t.abs() > 1e-12) {
                        return None;
                    }
                    // The radius is measured where the cone crosses the plane through the
                    // origin; the apex lies `radius / tan` back along the axis.
                    Some(FaceSurface::Cone { apex: v3(s.origin) - axis.vec() * (radius / t), axis, half_angle: semi_angle.abs() })
                }),
                cadrum::SurfaceKind::Sphere { radius } => Some(FaceSurface::Sphere { center: v3(s.origin), radius }),
                cadrum::SurfaceKind::Torus { major_radius, minor_radius } => {
                    axis.map(|axis| FaceSurface::Torus { center: v3(s.origin), axis, major: major_radius, minor: minor_radius })
                }
            })
        })
        .ok()
        .flatten()
}

/// The circle edge `edge` lies on, when it does: fitted through its points.
pub fn edge_circle(b: &Body, edge: usize) -> Option<EdgeCircle> {
    let inner = b.inner().ok()?;
    let tol = (b.size() * 1e-6).max(1e-9);
    inner
        .with("edge circle", |solids| {
            let Some(e) = inner.edge(solids, edge) else { return Ok(None) };
            let tess = cadrum::Tessellation { deflection_linear: tol, deflection_angular: 0.1, relative_linear: false };
            let pts: Vec<Vec3> = e.approximation_segments(tess).into_iter().map(v3).collect();
            Ok(circle_through(&pts, tol * 100.0))
        })
        .ok()
        .flatten()
}

/// The circle through `pts` (all of them within `tol`), from the first, middle and last.
pub(crate) fn circle_through(pts: &[Vec3], tol: f64) -> Option<EdgeCircle> {
    if pts.len() < 3 {
        return None;
    }
    let (a, b, c) = (*pts.first()?, *pts.get(pts.len() / 2)?, *pts.last()?);
    let (ab, ac) = (b - a, c - a);
    let n = ab.cross(ac);
    let n2 = n.dot(n);
    if !(n2 > 1e-18) {
        // A closed circle: first == last; take thirds instead.
        if (a - c).len() < tol && pts.len() >= 4 {
            let (p, q, r) = (*pts.first()?, *pts.get(pts.len() / 3)?, *pts.get(2 * pts.len() / 3)?);
            let (pq, pr) = (q - p, r - p);
            let nn = pq.cross(pr);
            let nn2 = nn.dot(nn);
            if !(nn2 > 1e-18) {
                return None;
            }
            let center = p + (pr * pq.dot(pq) - pq * pr.dot(pr)).cross(nn) * (1.0 / (2.0 * nn2));
            let radius = (p - center).len();
            if pts.iter().any(|x| ((*x - center).len() - radius).abs() > tol) {
                return None;
            }
            return Some(EdgeCircle { center, axis: UnitVec3::new(nn)?, radius, start: a, end: c, angle: std::f64::consts::TAU });
        }
        return None;
    }
    let center = a + (ac * ab.dot(ab) - ab * ac.dot(ac)).cross(n) * (1.0 / (2.0 * n2));
    let radius = (a - center).len();
    if !(radius > tol) || pts.iter().any(|p| ((*p - center).len() - radius).abs() > tol) {
        return None;
    }
    let axis = UnitVec3::new(n)?;
    let (u, w) = ((a - center) * (1.0 / radius), (c - center) * (1.0 / radius));
    let mut angle = u.dot(w).clamp(-1.0, 1.0).acos();
    if axis.vec().dot(u.cross(w)) < 0.0 {
        angle = std::f64::consts::TAU - angle;
    }
    Some(EdgeCircle { center, axis, radius, start: a, end: c, angle })
}

/// Face `face` as a cylinder, with its extent along the axis.
pub fn cylinder_face(b: &Body, _mesh: &Mesh, face: usize) -> Option<CylinderFace> {
    cylinder_face_index(b, face)
}

fn cylinder_face_index(b: &Body, face: usize) -> Option<CylinderFace> {
    let FaceSurface::Cylinder { origin, axis, radius } = face_surface(b, face)? else { return None };
    let inner = b.inner().ok()?;
    inner
        .with("cylinder face", |solids| {
            let Some(f) = inner.face(solids, face) else { return Ok(None) };
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for e in f.iter_edge() {
                for p in [v3(e.start_point()), v3(e.end_point())] {
                    let t = (p - origin).dot(axis);
                    lo = lo.min(t);
                    hi = hi.max(t);
                }
            }
            if !(lo.is_finite() && hi.is_finite()) {
                return Ok(None);
            }
            let center = v3(f.center());
            let (q, n) = f.project(dv(center));
            let (q, n) = (v3(q), v3(n));
            let radial = q - origin - axis.vec() * (q - origin).dot(axis);
            let internal = n.dot(radial) < 0.0;
            Ok(Some(CylinderFace { axis_point: origin, axis, radius, start: lo, end: hi, internal }))
        })
        .ok()
        .flatten()
}

/// The cylindrical face `p` lies on, if any.
pub fn cylinder_face_at(b: &Body, p: Vec3) -> Option<CylinderFace> {
    if !p.is_finite() {
        return None;
    }
    let tol = (b.size() * 1e-5).max(1e-6);
    (0..b.face_count()).filter_map(|i| cylinder_face_index(b, i)).find(|c| c.off(p) < tol)
}

/// The cylinder a planar end face at `p` closes: not read on this backend yet.
pub fn cylinder_end_at(_b: &Body, _p: Vec3) -> Option<CylinderFace> {
    None
}

/// Topology counts after merging pieces of one surface or curve (comparable with the truck
/// backend's `merged` counts, and with files that keep periodic faces whole).
pub fn merged_topology(b: &Body, _mesh: &Mesh) -> Result<TopoCounts> {
    Ok(crate::measure(b)?.merged)
}

/// The merged counts of `b`'s topology: faces on one elementary surface that touch count as
/// one, the edges between them vanish, edges on one circle or line that continue one another
/// within a merged face's boundary count as one, and the vertices those joins used go.
pub(crate) fn merged_counts(b: &Body) -> TopoCounts {
    let Ok(inner) = b.inner() else {
        return TopoCounts { faces: b.face_count(), edges: b.edge_count(), vertices: b.vertex_count(), face_types: Default::default() };
    };
    let nf = inner.faces.len();
    let tol = (b.size() * 1e-6).max(1e-9);
    let surfaces: Vec<Option<FaceSurface>> = (0..nf).map(|i| face_surface(b, i)).collect();
    // Faces: union by edges whose two faces lie on one surface.
    let mut face_group: Vec<usize> = (0..nf).collect();
    for fs in &inner.edge_faces {
        if let [f, g] = fs.as_slice()
            && f != g
            && let (Some(Some(s)), Some(Some(t))) = (surfaces.get(*f), surfaces.get(*g))
            && same_surface(s, t, tol)
        {
            unite(&mut face_group, *f, *g);
        }
    }
    let face_groups = (0..nf).filter(|i| find(&mut face_group, *i) == *i).count();
    // Edges: those left between different groups, united along one curve through a shared end.
    let circles: Vec<Option<EdgeCircle>> = (0..inner.edges.len()).map(|e| edge_circle(b, e)).collect();
    let ends: Vec<Option<(Vec3, Vec3)>> = inner
        .with("edge ends", |solids| {
            Ok((0..inner.edges.len()).map(|e| inner.edge(solids, e).map(|edge| (v3(edge.start_point()), v3(edge.end_point())))).collect())
        })
        .unwrap_or_default();
    let kept: Vec<usize> = (0..inner.edges.len())
        .filter(|&e| {
            let groups: std::collections::BTreeSet<usize> =
                inner.edge_faces.get(e).map(|fs| fs.iter().map(|f| find(&mut face_group, *f)).collect()).unwrap_or_default();
            groups.len() != 1
        })
        .collect();
    let mut edge_group: Vec<usize> = (0..inner.edges.len()).collect();
    let bounds = |e: usize| -> std::collections::BTreeSet<usize> {
        let mut fg = face_group.clone();
        inner.edge_faces.get(e).map(|fs| fs.iter().map(|f| find(&mut fg, *f)).collect()).unwrap_or_default()
    };
    for (k, &e) in kept.iter().enumerate() {
        for &d in kept.iter().skip(k + 1) {
            let (Some(Some(pe)), Some(Some(pd))) = (ends.get(e), ends.get(d)) else { continue };
            let touch = [pe.0, pe.1].iter().any(|a| [pd.0, pd.1].iter().any(|c| (*a - *c).len() < tol));
            if !touch || bounds(e) != bounds(d) {
                continue;
            }
            let same_curve = match (circles.get(e), circles.get(d)) {
                (Some(Some(ce)), Some(Some(cd))) => {
                    (ce.center - cd.center).len() < tol && (ce.radius - cd.radius).abs() < tol && ce.axis.vec().dot(cd.axis.vec()).abs() > 1.0 - 1e-9
                }
                (Some(None), Some(None)) => {
                    let (u, w) = (pe.1 - pe.0, pd.1 - pd.0);
                    u.cross(w).len() < 1e-9 * u.len().max(1.0) * w.len().max(1.0) && (pd.0 - pe.0).cross(u).len() < tol * u.len().max(1.0)
                }
                _ => false,
            };
            if same_curve {
                unite(&mut edge_group, e, d);
            }
        }
    }
    let merged_edges = kept.iter().filter(|&&e| find(&mut edge_group, e) == e).count();
    // Vertices: the ends of the kept edges, less those that only join two edges merged into one.
    let mut points: Vec<(Vec3, usize, std::collections::BTreeSet<usize>)> = Vec::new();
    for &e in &kept {
        let Some(Some((a, c))) = ends.get(e) else { continue };
        let g = find(&mut edge_group, e);
        for p in [*a, *c] {
            match points.iter_mut().find(|(q, _, _)| (*q - p).len() < tol) {
                Some(slot) => {
                    slot.1 += 1;
                    slot.2.insert(g);
                }
                None => points.push((p, 1, [g].into_iter().collect())),
            }
        }
    }
    let vertices = points.iter().filter(|(_, n, groups)| !(*n == 2 && groups.len() == 1)).count();
    TopoCounts { faces: face_groups, edges: merged_edges, vertices, face_types: Default::default() }
}

fn find(parent: &mut [usize], i: usize) -> usize {
    let mut r = i;
    while parent.get(r).is_some_and(|p| *p != r) {
        r = parent[r];
    }
    let mut c = i;
    while parent.get(c).is_some_and(|p| *p != r) {
        let next = parent[c];
        parent[c] = r;
        c = next;
    }
    r
}

fn unite(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (find(parent, a), find(parent, b));
    if ra != rb && rb < parent.len() {
        parent[rb] = ra;
    }
}

/// Per edge: is it a seam of one face?
pub fn seam_flags(b: &Body, mesh: &Mesh) -> Vec<bool> {
    let n = b.edge_count();
    let mut out = mesh.seams.clone();
    out.resize(n, false);
    out
}

/// The chosen faces plus every face on the same surface as one of them.
pub fn with_same_surface(b: &Body, chosen: &[usize]) -> Vec<usize> {
    let surfaces: Vec<Option<FaceSurface>> = (0..b.face_count()).map(|i| face_surface(b, i)).collect();
    let tol = (b.size() * 1e-6).max(1e-9);
    let mut out: Vec<usize> = chosen.iter().copied().filter(|i| *i < surfaces.len()).collect();
    for &c in chosen {
        let Some(Some(s)) = surfaces.get(c) else { continue };
        for (i, t) in surfaces.iter().enumerate() {
            if let Some(t) = t
                && !out.contains(&i)
                && same_surface(s, t, tol)
            {
                out.push(i);
            }
        }
    }
    out.sort_unstable();
    out
}

fn same_surface(a: &FaceSurface, b: &FaceSurface, tol: f64) -> bool {
    let close = |p: Vec3, q: Vec3| (p - q).len() < tol;
    let parallel = |u: UnitVec3, v: UnitVec3| u.vec().dot(v.vec()).abs() > 1.0 - 1e-9;
    match (a, b) {
        (FaceSurface::Plane { normal: n1, point: p1 }, FaceSurface::Plane { normal: n2, point: p2 }) => {
            parallel(*n1, *n2) && (*p2 - *p1).dot(*n1).abs() < tol
        }
        (FaceSurface::Cylinder { origin: o1, axis: a1, radius: r1 }, FaceSurface::Cylinder { origin: o2, axis: a2, radius: r2 }) => {
            parallel(*a1, *a2) && (r1 - r2).abs() < tol && (*o2 - *o1).cross(a1.vec()).len() < tol
        }
        (FaceSurface::Sphere { center: c1, radius: r1 }, FaceSurface::Sphere { center: c2, radius: r2 }) => close(*c1, *c2) && (r1 - r2).abs() < tol,
        (FaceSurface::Torus { center: c1, axis: a1, major: m1, minor: n1 }, FaceSurface::Torus { center: c2, axis: a2, major: m2, minor: n2 }) => {
            close(*c1, *c2) && parallel(*a1, *a2) && (m1 - m2).abs() < tol && (n1 - n2).abs() < tol
        }
        (FaceSurface::Cone { apex: p1, axis: a1, half_angle: h1 }, FaceSurface::Cone { apex: p2, axis: a2, half_angle: h2 }) => {
            close(*p1, *p2) && parallel(*a1, *a2) && (h1 - h2).abs() < 1e-9
        }
        _ => false,
    }
}

pub fn coaxial_candidate(_a: &Body, _b: &Body) -> Option<bool> {
    None
}

pub fn coaxial_overlap(_a: &Body, _b: &Body) -> Option<CoaxialOverlap> {
    None
}

/// Are two prisms the same by their certificates? Not decided on this backend.
pub fn certified_prism_equivalent(_a: &Body, _b: &Body) -> Result<Option<bool>> {
    Ok(None)
}

//! Building bodies: primitives, and prisms and revolutions of sketch regions. Loft and sweep
//! arrive with phase 5.

use std::sync::Arc;

use solidcraft_geom::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};
use solidcraft_kernel_core::history::{Change, EdgeOrigin, EdgeProv, FaceOrigin, FaceProv, History, Side};

use crate::body::{Body, dv, v3};
use crate::sketch_profile::{self as profile, ProfileEdge};
use crate::types::PathSeg;
use crate::{FaceSource, KernelError, Result, failed, unsupported};

/// Largest coordinate or size a primitive takes (mm).
const MAX_COORD: f64 = 1e9;

fn finite(v: Vec3) -> bool {
    v.is_finite() && v.x.abs() < MAX_COORD && v.y.abs() < MAX_COORD && v.z.abs() < MAX_COORD
}

/// A box with opposite corners `a` and `b`. Its faces carry `FaceSource::Box` (axis and side).
pub fn box_solid(a: Vec3, b: Vec3) -> Result<Body> {
    if !(finite(a) && finite(b)) {
        return Err(KernelError::Invalid("box corners must be finite".into()));
    }
    let lo = Vec3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z));
    let hi = Vec3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z));
    let d = hi - lo;
    if !(d.x > 1e-9 && d.y > 1e-9 && d.z > 1e-9) {
        return Err(KernelError::Invalid("box with a zero side".into()));
    }
    let solid = crate::guard("box", || Ok(cadrum::Solid::cube(dv(lo), dv(hi))))?;
    let body = Body::from_solids(vec![solid])?;
    let sources = planar_sources(&body, |n, center| {
        let axis = if n.x.abs() >= n.y.abs() && n.x.abs() >= n.z.abs() {
            0
        } else if n.y.abs() >= n.z.abs() {
            1
        } else {
            2
        };
        let high = [n.x, n.y, n.z][axis] > 0.0;
        Plane::from_normal(center, n).map(|plane| FaceSource::Box { producer: None, axis: axis as u8, high, plane })
    })?;
    Ok(body.with_face_sources(Some(Arc::new(sources))))
}

/// A cylinder from `base` along `axis`. Its caps carry extrude-cap sources (the cylinder
/// command certifies them), its wall an extrude side of the full circle.
pub fn cylinder(base: Vec3, axis: Vec3, radius: f64, height: f64) -> Result<Body> {
    if !(finite(base) && finite(axis) && radius.is_finite() && height.is_finite()) {
        return Err(KernelError::Invalid("cylinder parameters must be finite".into()));
    }
    let Some(dir) = axis.normalized() else { return Err(KernelError::Invalid("cylinder axis must not be zero".into())) };
    if !(radius > 1e-9 && radius < MAX_COORD && height > 1e-9 && height < MAX_COORD) {
        return Err(KernelError::Invalid("cylinder radius and height must be positive".into()));
    }
    let solid = crate::guard("cylinder", || Ok(cadrum::Solid::cylinder(radius, dv(dir * height)).translate(dv(base))))?;
    let body = Body::from_solids(vec![solid])?;
    let circle = Region2 { outer: Loop2::circle(Vec2::ZERO, radius), holes: Vec::new() };
    let mut sources: Vec<Option<FaceSource>> = planar_sources(&body, |n, center| {
        let high = n.dot(dir) > 0.0;
        Plane::from_normal(center, n).map(|plane| FaceSource::ExtrudeCap { high, region: circle.clone(), plane })
    })?;
    for s in sources.iter_mut() {
        if s.is_none() {
            *s = Some(FaceSource::ExtrudeSide {
                segment: Seg2::Arc { center: Vec2::ZERO, radius, start: 0.0, sweep: std::f64::consts::TAU },
                plane: None,
                coalesced: true,
            });
        }
    }
    Ok(body.with_face_sources(Some(Arc::new(sources))))
}

pub fn sphere(center: Vec3, radius: f64) -> Result<Body> {
    if !(finite(center) && radius.is_finite() && radius > 1e-9 && radius < MAX_COORD) {
        return Err(KernelError::Invalid("sphere centre must be finite and its radius positive".into()));
    }
    let solid = crate::guard("sphere", || Ok(cadrum::Solid::sphere(radius).translate(dv(center))))?;
    Body::from_solids(vec![solid])
}

pub fn torus(center: Vec3, major: f64, minor: f64) -> Result<Body> {
    if !(finite(center) && major.is_finite() && minor.is_finite() && minor > 1e-9 && major > minor && major < MAX_COORD) {
        return Err(KernelError::Invalid("torus radii must be finite, the minor positive and smaller than the major".into()));
    }
    let solid = crate::guard("torus", || Ok(cadrum::Solid::torus(major, minor, cadrum::DVec3::Z).translate(dv(center))))?;
    Body::from_solids(vec![solid])
}

/// A source for each planar face of `body` from its outward normal and centre (`None` for the
/// others).
fn planar_sources(body: &Body, mut source: impl FnMut(Vec3, Vec3) -> Option<FaceSource>) -> Result<Vec<Option<FaceSource>>> {
    let inner = body.inner()?;
    inner.with("face sources", |solids| {
        Ok((0..inner.faces.len())
            .map(|i| {
                let face = inner.face(solids, i)?;
                let s = face.surface()?;
                if s.kind != cadrum::SurfaceKind::Plane {
                    return None;
                }
                let center = v3(face.center());
                let n = v3(face.project(dv(center)).1).normalized()?;
                source(n, center)
            })
            .collect())
    })
}

// ---- prisms ----

/// Regions on `plane` extruded from `start` to `end` along its normal, one body per region.
/// Each body's faces carry their sources (sides by sketch segment, caps by end) and a history
/// (`FaceOrigin::Profile` / `Cap`), so names and attachments follow them.
pub fn extrude(plane: &Plane, regions: &[Region2], start: f64, end: f64) -> Result<Vec<Body>> {
    extrude_with(plane, regions, start, end, false)
}

/// [`extrude`] with runs of collinear sketch lines merged into one side face each.
pub fn extrude_whole_sides(plane: &Plane, regions: &[Region2], start: f64, end: f64) -> Result<Vec<Body>> {
    extrude_with(plane, regions, start, end, true)
}

fn extrude_with(plane: &Plane, regions: &[Region2], start: f64, end: f64, whole_sides: bool) -> Result<Vec<Body>> {
    if !profile::frame_ok(plane) {
        return Err(KernelError::Invalid("extrude frame is not finite and orthonormal".into()));
    }
    let (start, end) = (profile::finite(start, "start")?, profile::finite(end, "end")?);
    let (lo, hi) = if end >= start { (start, end) } else { (end, start) };
    if hi - lo < 1e-6 {
        return Err(KernelError::Invalid("extrude distance is zero".into()));
    }
    if regions.is_empty() {
        return Err(KernelError::Invalid("no profile selected".into()));
    }
    let n = plane.normal();
    let base = plane.offset(lo);
    let mut out = Vec::with_capacity(regions.len());
    for r in regions {
        profile::region_ok(r)?;
        let numbered = profile::numbered(r, whole_sides)?;
        let loops = profile::edges(&base, &numbered)?;
        let flat: Vec<&cadrum::Edge> = loops.iter().flatten().map(|e| &e.edge).collect();
        let solid = crate::guard("extrude", || cadrum::Solid::extrude(flat.iter().copied(), dv(n * (hi - lo))).map_err(|e| failed("extrude", e)))?;
        let body = Body::from_solids(vec![solid])?;
        let edges: Vec<&ProfileEdge> = loops.iter().flatten().collect();
        let (sources, faces) = classify(
            &body,
            plane,
            &edges,
            |face, _, normal, center| {
                if (normal.dot(n).abs() - 1.0).abs() > 1e-6 || face.surface().map(|s| s.kind) != Some(cadrum::SurfaceKind::Plane) {
                    return None;
                }
                let h = (center - plane.origin).dot(n);
                let high = (h - hi).abs() < (h - lo).abs();
                let cap_plane = if high { plane.offset(hi) } else { Plane { origin: base.origin, x: plane.x, y: -plane.y } };
                Some((FaceSource::ExtrudeCap { high, region: r.clone(), plane: cap_plane }, high))
            },
            |edge_points| edge_points.iter().all(|p| ((*p - base.origin).dot(n)).abs() < 1e-6 * (hi - lo).max(1.0) + 1e-9),
            |p| plane.to_local(p - n * (p - plane.origin).dot(n)),
        )?;
        out.push(attach(body.with_face_sources(Some(Arc::new(sources))), faces)?);
    }
    Ok(out)
}

/// The prism over every region as one body (regions of one sketch that may touch).
pub fn extrude_planar_regions(plane: &Plane, regions: &[Region2], start: f64, end: f64) -> Result<Body> {
    let bodies = extrude(plane, regions, start, end)?;
    match bodies.len() {
        0 => Err(KernelError::Invalid("no profile selected".into())),
        1 => bodies.into_iter().next().ok_or_else(|| KernelError::Internal("prism".into())),
        _ => crate::ops::join_apart(&bodies)?.ok_or_else(|| KernelError::Failed("the prisms left nothing".into())),
    }
}

// ---- revolutions ----

/// The revolve axis in space, pointing so that the sweep turns the profile the positive way.
pub fn canonical_revolve_axis(plane: &Plane, region: &Region2, axis_origin: Vec2, axis_dir: Vec2) -> Result<Vec3> {
    if !axis_origin.is_finite() || !axis_dir.is_finite() || !profile::frame_ok(plane) {
        return Err(KernelError::Invalid("non-finite revolve frame".into()));
    }
    profile::region_ok(region)?;
    let direction = axis_dir.normalized().ok_or_else(|| KernelError::Invalid("revolve axis has no direction".into()))?;
    let axis = plane.dir_to_world(direction).normalized().ok_or_else(|| KernelError::Invalid("revolve frame has no axis".into()))?;
    let origin = plane.to_world(axis_origin);
    let center = plane.to_world(region.centroid());
    let radial = center - origin - axis * (center - origin).dot(axis);
    Ok(if axis.cross(radial).dot(plane.normal()) < 0. { -axis } else { axis })
}

/// Regions on `plane` turned by `angle` (radians) about the axis through `axis_origin` along
/// `axis_dir` (both in the plane), one body per region, with sources and a history.
pub fn revolve(plane: &Plane, regions: &[Region2], axis_origin: Vec2, axis_dir: Vec2, angle: f64) -> Result<Vec<Body>> {
    revolved(plane, regions, axis_origin, axis_dir, angle)
}

/// [`revolve`]; OpenCASCADE keeps a full turn's faces whole already.
pub fn revolve_whole(plane: &Plane, regions: &[Region2], axis_origin: Vec2, axis_dir: Vec2, angle: f64) -> Result<Vec<Body>> {
    revolved(plane, regions, axis_origin, axis_dir, angle)
}

fn revolved(plane: &Plane, regions: &[Region2], axis_origin: Vec2, axis_dir: Vec2, angle: f64) -> Result<Vec<Body>> {
    if !profile::frame_ok(plane) || !axis_origin.is_finite() {
        return Err(KernelError::Invalid("revolve frame is not finite and orthonormal".into()));
    }
    let angle = profile::finite(angle, "angle")?;
    if angle.abs() < 1e-6 {
        return Err(KernelError::Invalid("revolve angle is zero".into()));
    }
    let angle = angle.clamp(-std::f64::consts::TAU, std::f64::consts::TAU);
    let d = axis_dir.normalized().ok_or_else(|| KernelError::Invalid("revolve axis has no direction".into()))?;
    if regions.is_empty() {
        return Err(KernelError::Invalid("no profile selected".into()));
    }
    let o = plane.to_world(axis_origin);
    let axis = plane.dir_to_world(d).normalized().ok_or_else(|| KernelError::Invalid("revolve frame has no axis".into()))?;
    let n = plane.normal();
    let mut out = Vec::with_capacity(regions.len());
    for r in regions {
        profile::region_ok(r)?;
        let (mut neg, mut pos) = (false, false);
        for p in r.outer.polyline(1e-3) {
            let s = d.cross(p - axis_origin);
            neg |= s < -1e-7;
            pos |= s > 1e-7;
        }
        if neg && pos {
            return Err(KernelError::Invalid("the profile crosses the revolve axis".into()));
        }
        // Points within a hair of the axis go onto it, so no sliver face is turned.
        let snap = |p: Vec2| {
            let along = (p - axis_origin).dot(d);
            let q = axis_origin + d * along;
            if p.dist(q) < 1e-7 { q } else { p }
        };
        let numbered = profile::numbered(r, false)?.map_loops(|l| {
            l.map_segs(|s| match *s {
                Seg2::Line { a, b } => Seg2::Line { a: snap(a), b: snap(b) },
                other => other,
            })
        });
        let loops = profile::edges(plane, &numbered)?;
        let flat: Vec<&cadrum::Edge> = loops.iter().flatten().map(|e| &e.edge).collect();
        let c = profile::centroid(plane, &r.outer);
        let radial = (c - o) - axis * (c - o).dot(axis);
        let turn = if axis.cross(radial).dot(n) < 0.0 { -angle } else { angle };
        // The profile's side of the axis, in the plane.
        let side = radial.normalized().unwrap_or_else(|| n.cross(axis));
        let solid =
            crate::guard("revolve", || cadrum::Solid::revolve(flat.iter().copied(), dv(o), dv(axis), turn).map_err(|e| failed("revolve", e)))?;
        let body = Body::from_solids(vec![solid])?;
        let edges: Vec<&ProfileEdge> = loops.iter().flatten().collect();
        let size = body.size().max(1e-9);
        let (sources, faces) = classify(
            &body,
            plane,
            &edges,
            |face, _, normal, center| {
                // An angle cap is planar and contains the axis; the start cap lies in the sketch plane.
                if normal.dot(axis).abs() > 1e-6
                    || face.surface().map(|s| s.kind) != Some(cadrum::SurfaceKind::Plane)
                    || ((center - o).dot(normal)).abs() > size * 1e-6
                {
                    return None;
                }
                let start = (center - plane.origin).dot(n).abs() < size * 1e-6 && normal.dot(n).abs() > 1.0 - 1e-6;
                // Caps of a revolution carry no stored source (their plane depends on the angle).
                Some((
                    FaceSource::ExtrudeCap { high: !start, region: r.clone(), plane: Plane::from_normal(center, normal).unwrap_or(*plane) },
                    !start,
                ))
            },
            |edge_points| edge_points.iter().all(|p| ((*p - plane.origin).dot(n)).abs() < size * 1e-6 + 1e-9),
            |p| {
                // Turn the point back into the profile's half-plane: its height along the axis and
                // its distance from it, on the profile's side.
                let w = p - o;
                let h = w.dot(axis);
                let r = (w - axis * h).len();
                plane.to_local(o + axis * h + side * r)
            },
        )?;
        // A revolution's caps are not extrude caps: keep only the side sources.
        let sources: Vec<Option<FaceSource>> = sources.into_iter().map(|s| s.filter(|s| matches!(s, FaceSource::ExtrudeSide { .. }))).collect();
        out.push(attach(body.with_face_sources(Some(Arc::new(sources))), faces)?);
    }
    Ok(out)
}

/// Each face of a swept body by what made it: `cap` says whether a face is an end cap (and
/// which end), `in_base` whether an edge's points lie in the profile's plane; the other faces
/// are sides, matched to the profile segment their base edge runs along.
#[allow(clippy::type_complexity)]
pub(crate) fn classify(
    body: &Body,
    plane: &Plane,
    edges: &[&ProfileEdge],
    mut cap: impl FnMut(&cadrum::Face, usize, Vec3, Vec3) -> Option<(FaceSource, bool)>,
    in_base: impl Fn(&[Vec3]) -> bool,
    back: impl Fn(Vec3) -> Vec2,
) -> Result<(Vec<Option<FaceSource>>, Vec<FaceProv>)> {
    let inner = body.inner()?;
    let n = plane.normal();
    let segs: Vec<Seg2> = edges.iter().map(|e| e.segment).collect();
    inner.with("sweep source identities", |solids| {
        let mut sources = Vec::with_capacity(inner.faces.len());
        let mut faces = Vec::with_capacity(inner.faces.len());
        let tess = cadrum::Tessellation { deflection_linear: body.size().max(1e-9) * 1e-3, deflection_angular: 0.5, relative_linear: false };
        for i in 0..inner.faces.len() {
            let Some(face) = inner.face(solids, i) else {
                sources.push(None);
                faces.push(generated(Vec::new()));
                continue;
            };
            let center = v3(face.center());
            let normal = v3(face.project(dv(center)).1).normalized().unwrap_or(Vec3::Z);
            if let Some((source, end)) = cap(face, i, normal, center) {
                sources.push(Some(source));
                faces.push(generated(vec![FaceOrigin::Cap { end }]));
                continue;
            }
            // The side's base edge, one of its edges in the profile's plane, names its segment
            // exactly; a side without one (a ring a revolution turns from a radial line) is
            // matched by its points turned back into the profile's plane.
            let mut matched: Option<usize> = None;
            let mut samples: Vec<Vec3> = vec![center];
            for e in face.iter_edge() {
                let pts: Vec<Vec3> = e.approximation_segments(tess).into_iter().map(v3).collect();
                if pts.len() >= 2 && in_base(&pts) {
                    let local: Vec<Vec2> = pts.iter().map(|p| back(*p)).collect();
                    matched = profile::nearest_segment(&segs, &local);
                    if matched.is_some() {
                        break;
                    }
                }
                samples.extend(pts.iter().copied().take(8));
            }
            if matched.is_none() {
                let local: Vec<Vec2> = samples.iter().map(|p| back(*p)).collect();
                matched = profile::nearest_segment(&segs, &local);
            }
            match matched.and_then(|k| edges.get(k)) {
                Some(side) => {
                    let basis = match side.segment {
                        Seg2::Line { a, b } => (b - a).normalized().and_then(|direction| {
                            let foot = a - direction * a.dot(direction);
                            Plane::new(plane.to_world(foot), plane.dir_to_world(direction), n)
                        }),
                        _ => None,
                    };
                    sources.push(Some(FaceSource::ExtrudeSide { segment: side.segment, plane: basis, coalesced: false }));
                    let numbers: std::collections::BTreeSet<usize> = side.numbers.iter().copied().collect();
                    faces.push(generated(numbers.into_iter().map(|segment| FaceOrigin::Profile { segment }).collect()));
                }
                None => {
                    sources.push(None);
                    faces.push(generated(Vec::new()));
                }
            }
        }
        Ok((sources, faces))
    })
}

pub(crate) fn generated(origins: Vec<FaceOrigin>) -> FaceProv {
    FaceProv { origins, change: Change::Generated }
}

/// The origins of a new edge between result faces `between`: the meeting of the two faces,
/// when each comes from something and no origin is under both; else none.
pub(crate) fn meeting(between: &[usize], faces: &[FaceProv]) -> Vec<EdgeOrigin> {
    let side = |f: &usize| faces.get(*f).and_then(|p| Side::new(p.origins.iter().copied()));
    match between {
        [f, g] => match (side(f), side(g)) {
            (Some(a), Some(b)) if a.origins().iter().all(|o| !b.origins().contains(o)) => vec![EdgeOrigin::meet(a, b)],
            _ => Vec::new(),
        },
        _ => Vec::new(),
    }
}

/// `body` with the history of a construction from nothing: `faces` in face order, each edge
/// where its faces meet.
pub(crate) fn attach(body: Body, faces: Vec<FaceProv>) -> Result<Body> {
    let inner = body.inner()?;
    let edges = inner.edge_faces.iter().map(|between| EdgeProv { origins: meeting(between, &faces), faces: between.clone() }).collect();
    let history = History { operands: 0, faces, edges };
    body.with_history(history, &[])
}

pub fn path_polyline(path: &[PathSeg]) -> Result<Vec<Vec3>> {
    let mut out = Vec::new();
    for (i, seg) in path.iter().enumerate() {
        match *seg {
            PathSeg::Line { a, b } => {
                if !(finite(a) && finite(b)) {
                    return Err(KernelError::Invalid("path points must be finite".into()));
                }
                if i == 0 {
                    out.push(a);
                }
                out.push(b);
            }
            PathSeg::Arc { a, center, axis, angle } => {
                if !(finite(a) && finite(center) && finite(axis) && angle.is_finite()) {
                    return Err(KernelError::Invalid("path arcs must be finite".into()));
                }
                let Some(k) = axis.normalized() else { return Err(KernelError::Invalid("path arc axis must not be zero".into())) };
                if i == 0 {
                    out.push(a);
                }
                let n = ((angle.abs() / 0.1).ceil() as usize).clamp(1, 512);
                for s in 1..=n {
                    out.push(rotate_about(a, center, k, angle * s as f64 / n as f64));
                }
            }
        }
    }
    Ok(out)
}

pub(crate) fn rotate_about(p: Vec3, center: Vec3, k: Vec3, angle: f64) -> Vec3 {
    let v = p - center;
    let (s, c) = angle.sin_cos();
    center + v * c + k.cross(v) * s + k * (k.dot(v) * (1.0 - c))
}

pub fn sweep_helix_mesh(
    _plane: &Plane,
    _polygon: &[Vec2],
    _axis_origin: Vec3,
    _axis: Vec3,
    _pitch: f64,
    _turns: f64,
    _stations: usize,
) -> Result<solidcraft_geom::Mesh> {
    Err(unsupported("helical sweep"))
}

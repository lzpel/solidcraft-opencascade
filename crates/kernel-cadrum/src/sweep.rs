//! Sweeps, lofts and helical sweeps through OpenCASCADE's pipe-shell and skinning builders.
//! The faces of a result are classified by the profile geometry, as an extrude's are: a side
//! by the profile segment its base edge runs along, a cap by the plane it lies in. The
//! builders report no history (lzpel/cadrum#317 item 5), so that classification is the history.

use std::f64::consts::TAU;

use solidcraft_geom::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};
use solidcraft_kernel_core::history::FaceOrigin;

use crate::body::{Body, dv, v3};
use crate::build::{attach, classify, generated, path_polyline, rotate_about};
use crate::sketch_profile::{self as profile, ProfileEdge};
use crate::types::{PathSeg, RailFit, SweepControl};
use crate::{FaceSource, KernelError, Result, failed, guard, unsupported};

const MAX_PATH: usize = 1000;
const MAX_SECTIONS: usize = 64;

/// Where a sweep ends: a point on the end cap's plane and that plane's normal.
#[derive(Clone, Copy)]
struct End {
    point: Vec3,
    normal: Vec3,
}

/// One closed loop as cadrum edges, swept along `spine`.
fn pipe(what: &str, lp: &[ProfileEdge], spine: &[cadrum::Edge], orient: cadrum::ProfileOrient<'static>) -> Result<cadrum::Solid> {
    guard(what, || cadrum::Solid::sweep(lp.iter().map(|e| &e.edge), spine.iter(), orient).map_err(|e| failed(what, e)))
}

/// `a` without the solids `tools`, as one solid (the largest, when the cut leaves several).
fn without(what: &str, a: cadrum::Solid, tools: &[cadrum::Solid]) -> Result<cadrum::Solid> {
    if tools.is_empty() {
        return Ok(a);
    }
    guard(what, || {
        let mut expr = cadrum::Boolean::from(&a);
        for t in tools {
            expr = expr - t;
        }
        let parts = expr.build_vec().map_err(|e| failed(what, e))?;
        parts
            .into_iter()
            .filter(|s| s.volume() > 0.0)
            .max_by(|x, y| x.volume().total_cmp(&y.volume()))
            .ok_or_else(|| KernelError::Failed(format!("{what}: the holes removed everything")))
    })
}

/// `solid`, built from the loops of `region` on `plane`, with every face placed: a side on
/// its profile segment, a cap at the start (on `plane`) or the end.
fn placed(what: &str, solid: cadrum::Solid, plane: &Plane, region: &Region2, loops: &[Vec<ProfileEdge>], end: Option<End>) -> Result<Body> {
    let body = Body::from_solids(vec![solid])?;
    let size = body.size().max(1e-9);
    let tol = size * 1e-5 + 1e-9;
    let n = plane.normal();
    let edges: Vec<&ProfileEdge> = loops.iter().flatten().collect();
    let on =
        |center: Vec3, normal: Vec3, origin: Vec3, axis: Vec3| (normal.dot(axis).abs() - 1.0).abs() < 1e-6 && (center - origin).dot(axis).abs() < tol;
    let (_, faces) = classify(
        &body,
        plane,
        &edges,
        |face, _, normal, center| {
            if face.surface().map(|s| s.kind) != Some(cadrum::SurfaceKind::Plane) {
                return None;
            }
            if on(center, normal, plane.origin, n) {
                return Some((FaceSource::ExtrudeCap { high: false, region: region.clone(), plane: *plane }, false));
            }
            match end {
                Some(e) if on(center, normal, e.point, e.normal) => {
                    Some((FaceSource::ExtrudeCap { high: true, region: region.clone(), plane: *plane }, true))
                }
                _ => None,
            }
        },
        |pts| pts.iter().all(|p| (*p - plane.origin).dot(n).abs() < tol),
        |p| plane.to_local(p - n * (p - plane.origin).dot(n)),
    )?;
    guard(what, || attach(body, faces))
}

/// The profile's loops (outer first) as cadrum edges on `plane`.
fn loops_on(plane: &Plane, region: &Region2) -> Result<Vec<Vec<ProfileEdge>>> {
    if !profile::frame_ok(plane) {
        return Err(KernelError::Invalid("profile frame is not finite and orthonormal".into()));
    }
    profile::region_ok(region)?;
    profile::edges(plane, &profile::numbered(region, false)?)
}

/// `region` swept along `spine`: the outer loop's pipe without the holes' pipes.
fn swept(
    what: &str,
    plane: &Plane,
    region: &Region2,
    spine: &[cadrum::Edge],
    orient: cadrum::ProfileOrient<'static>,
    end: Option<End>,
) -> Result<Body> {
    let loops = loops_on(plane, region)?;
    let (Some(outer), holes) = (loops.first(), loops.get(1..).unwrap_or_default()) else {
        return Err(KernelError::Invalid("profile has no outer loop".into()));
    };
    let solid = pipe(what, outer, spine, orient)?;
    let tools: Vec<cadrum::Solid> = holes.iter().map(|h| pipe(what, h, spine, orient)).collect::<Result<_>>()?;
    let solid = without(what, solid, &tools)?;
    placed(what, solid, plane, region, &loops, end)
}

/// A full turn of arcs about one axis: a revolution of the profile.
fn full_circle(path: &[PathSeg]) -> Option<(Vec3, Vec3, f64)> {
    let PathSeg::Arc { a: first, center, axis, .. } = *path.first()? else { return None };
    let axis = axis.normalized()?;
    let mut end = first;
    let mut total = 0.0;
    for seg in path {
        let PathSeg::Arc { a, center: c, axis: k, angle } = *seg else { return None };
        if a.dist(end) > 1e-7 || c.dist(center) > 1e-7 || k.normalized()?.dist(axis) > 1e-9 || !angle.is_finite() {
            return None;
        }
        total += angle;
        end = rotate_about(a, c, axis, angle);
    }
    ((total.abs() - TAU).abs() < 1e-9).then_some((center, axis, total))
}

/// The profile turned about `axis` through `center` by `angle` (a sweep along a full circle, or
/// a helix of no pitch): no caps.
fn revolved(what: &str, plane: &Plane, region: &Region2, center: Vec3, axis: Vec3, angle: f64) -> Result<Body> {
    let loops = loops_on(plane, region)?;
    let flat: Vec<&cadrum::Edge> = loops.iter().flatten().map(|e| &e.edge).collect();
    let solid = guard(what, || cadrum::Solid::revolve(flat.iter().copied(), dv(center), dv(axis), angle).map_err(|e| failed(what, e)))?;
    placed(what, solid, plane, region, &loops, None)
}

/// The spine of a path of lines and arcs, and where it ends.
fn spine_of(path: &[PathSeg]) -> Result<(Vec<cadrum::Edge>, End)> {
    let mut spine = Vec::with_capacity(path.len());
    let mut end: Option<End> = None;
    for seg in path {
        match *seg {
            PathSeg::Line { a, b } => {
                let Some(dir) = (b - a).normalized() else { continue };
                spine.push(guard("sweep path", || cadrum::Edge::line(dv(a), dv(b)).map_err(|e| failed("sweep path", e)))?);
                end = Some(End { point: b, normal: dir });
            }
            PathSeg::Arc { a, center, axis, angle } => {
                let k = axis.normalized().ok_or_else(|| KernelError::Invalid("path arc axis must not be zero".into()))?;
                if angle.abs() < 1e-9 {
                    continue;
                }
                if angle.abs() > TAU - 1e-9 {
                    return Err(KernelError::Invalid("a path arc must be less than a full turn".into()));
                }
                let mid = rotate_about(a, center, k, angle * 0.5);
                let b = rotate_about(a, center, k, angle);
                spine.push(guard("sweep path", || cadrum::Edge::arc_3pts(dv(a), dv(mid), dv(b)).map_err(|e| failed("sweep path", e)))?);
                let tangent = k.cross(b - center).normalized().map(|t| if angle < 0.0 { -t } else { t }).unwrap_or(k);
                end = Some(End { point: b, normal: tangent });
            }
        }
    }
    match end {
        Some(e) if !spine.is_empty() => Ok((spine, e)),
        _ => Err(KernelError::Invalid("the path has no length".into())),
    }
}

/// How the profile rides a smooth path of arcs: its binormal held to the first arc's axis, so
/// it stays perpendicular to the path without turning about it.
fn orientation(axes: &[Vec3]) -> cadrum::ProfileOrient<'static> {
    match axes.first() {
        Some(k) => cadrum::ProfileOrient::Up(dv(*k)),
        None => cadrum::ProfileOrient::Fixed,
    }
}

/// Sweep a planar region along a chain of path segments, keeping the profile perpendicular to
/// the path (lines translate it, arcs rotate it). A path closing as one full circle is a
/// revolution.
pub fn sweep(plane: &Plane, region: &Region2, path: &[PathSeg]) -> Result<Body> {
    if path.is_empty() || path.len() > MAX_PATH {
        return Err(KernelError::Invalid(format!("the path needs 1…{MAX_PATH} segments")));
    }
    path_polyline(path)?;
    if let Some((center, axis, angle)) = full_circle(path) {
        return revolved("sweep", plane, region, center, axis, angle);
    }
    // A path with sharp corners is swept as a polyline, mitred at the corners.
    let tangents: Vec<(Vec3, Vec3)> = path.iter().filter_map(tangents_of).collect();
    let corner = tangents.windows(2).any(|w| w[0].1.dot(w[1].0) < (1e-3f64).cos());
    let axes: Vec<Vec3> = path.iter().filter_map(|s| if let PathSeg::Arc { axis, .. } = s { axis.normalized() } else { None }).collect();
    if corner || axes.is_empty() {
        return mitred("sweep", plane, region, &path_polyline(path)?);
    }
    let (spine, end) = spine_of(path)?;
    swept("sweep", plane, region, &spine, orientation(&axes), Some(end))
}

/// A segment's direction at its start and its end.
fn tangents_of(seg: &PathSeg) -> Option<(Vec3, Vec3)> {
    match *seg {
        PathSeg::Line { a, b } => (b - a).normalized().map(|d| (d, d)),
        PathSeg::Arc { a, center, axis, angle } => {
            let k = axis.normalized()?;
            let sign = if angle < 0.0 { -1.0 } else { 1.0 };
            let start = (k.cross(a - center) * sign).normalized()?;
            let end = (k.cross(rotate_about(a, center, k, angle) - center) * sign).normalized()?;
            Some((start, end))
        }
    }
}

/// [`sweep`] with sweep options: only the default options are on this backend yet.
pub fn sweep_with(plane: &Plane, region: &Region2, path: &[PathSeg], ctl: &SweepControl) -> Result<Body> {
    if !ctl.is_default() {
        return Err(unsupported("sweep options (orientation, twist, scale, rails)"));
    }
    sweep(plane, region, path)
}

/// The distinct points of a polyline path, in order.
fn polyline_points(path: &[Vec3]) -> Result<Vec<Vec3>> {
    if path.len() < 2 || path.len() > MAX_PATH {
        return Err(KernelError::Invalid(format!("the path needs 2…{MAX_PATH} points")));
    }
    if path.iter().any(|p| !p.is_finite()) {
        return Err(KernelError::Invalid("path points must be finite".into()));
    }
    let mut pts: Vec<Vec3> = Vec::with_capacity(path.len());
    for p in path {
        if pts.last().is_none_or(|q| q.dist(*p) > 1e-9) {
            pts.push(*p);
        }
    }
    if pts.len() < 2 {
        return Err(KernelError::Invalid("the path has no length".into()));
    }
    Ok(pts)
}

/// Sweep a planar region along a 3D polyline path. The profile keeps its relation to the path
/// (it need not touch it); the path should start near the profile.
pub fn sweep_path(plane: &Plane, region: &Region2, path: &[Vec3]) -> Result<Body> {
    let pts = polyline_points(path)?;
    mitred("sweep", plane, region, &pts)
}

/// [`sweep_path`] with sweep options: only the default options are on this backend yet.
pub fn sweep_path_with(plane: &Plane, region: &Region2, path: &[Vec3], ctl: &SweepControl) -> Result<Body> {
    if !ctl.is_default() {
        return Err(unsupported("sweep options (orientation, twist, scale, rails)"));
    }
    sweep_path(plane, region, path)
}

/// Sweep a planar region along a helix about `axis` through `axis_origin`: `turns` revolutions
/// (right-handed about `axis`), rising `pitch` per turn. The profile keeps its attitude relative
/// to the axis, as a screw motion does.
pub fn sweep_helix(plane: &Plane, region: &Region2, axis_origin: Vec3, axis: Vec3, pitch: f64, turns: f64) -> Result<Body> {
    let k = axis.normalized().ok_or_else(|| KernelError::Invalid("helix axis".into()))?;
    if !(pitch.is_finite() && turns.is_finite() && turns > 0.0 && turns <= 200.0 && pitch.abs() < 1e7 && axis_origin.is_finite()) {
        return Err(KernelError::Invalid("helix: turns must be in (0, 200] and the pitch finite".into()));
    }
    if !profile::frame_ok(plane) {
        return Err(KernelError::Invalid("profile frame is not finite and orthonormal".into()));
    }
    profile::region_ok(region)?;
    if region.area().abs() < 1e-9 {
        return Err(KernelError::Invalid("profile has no area".into()));
    }
    let along: Vec<f64> = region
        .outer
        .segs
        .iter()
        .flat_map(|s| (0..8).map(move |i| s.point_at(i as f64 / 8.0)))
        .map(|q| (plane.to_world(q) - axis_origin).dot(k))
        .collect();
    let extent = along.iter().copied().fold(f64::NEG_INFINITY, f64::max) - along.iter().copied().fold(f64::INFINITY, f64::min);
    if turns > 1.0 && pitch.abs() <= extent {
        return Err(KernelError::Invalid(format!("the coil runs into itself: the pitch must exceed the section ({extent:.3} mm)")));
    }
    let total = TAU * turns;
    if pitch.abs() < 1e-9 {
        let angle = if (turns - 1.0).abs() < 1e-12 { TAU } else { total };
        return revolved("helix sweep", plane, region, axis_origin, k, angle);
    }
    let centroid = profile::centroid(plane, &region.outer);
    let radial = centroid - axis_origin;
    let lift = radial.dot(k);
    let x_ref = radial - k * lift;
    let radius = x_ref.len();
    if radius < 1e-9 {
        return Err(KernelError::Invalid("the profile must lie off the helix axis".into()));
    }
    let helix = guard("helix", || {
        cadrum::Edge::helix(radius, pitch, pitch * turns, dv(k), dv(x_ref))
            .map(|e| e.translate(dv(axis_origin + k * lift)))
            .map_err(|e| failed("helix", e))
    })?;
    let n = plane.normal();
    let end = End { point: rotate_about(plane.origin, axis_origin, k, total) + k * (pitch * turns), normal: rotate_about(n, Vec3::ZERO, k, total) };
    swept("helix sweep", plane, region, std::slice::from_ref(&helix), cadrum::ProfileOrient::Up(dv(k)), Some(end))
}

/// Ruled loft through two planar sections, smooth through more (outer loops only).
pub fn loft(sections: &[(Plane, Loop2)]) -> Result<Body> {
    let regions: Vec<(Plane, Region2)> = sections.iter().map(|(p, lp)| (*p, Region2 { outer: lp.clone(), holes: Vec::new() })).collect();
    loft_regions(&regions)
}

/// [`loft`] guided by rails: not on this backend yet.
pub fn loft_with_rails(sections: &[(Plane, Loop2)], rails: &[Vec<Vec3>], _fit: RailFit) -> Result<Body> {
    if !rails.is_empty() {
        return Err(unsupported("a loft with rails"));
    }
    loft(sections)
}

/// A loft through regions with matching hole counts: the outer loops' loft without the lofts
/// of each hole (holes paired by their order in each section).
pub fn loft_regions(sections: &[(Plane, Region2)]) -> Result<Body> {
    if sections.len() < 2 || sections.len() > MAX_SECTIONS {
        return Err(KernelError::Invalid(format!("a loft needs 2…{MAX_SECTIONS} sections")));
    }
    let (Some((first_plane, first_region)), Some((last_plane, last_region))) = (sections.first(), sections.last()) else {
        return Err(KernelError::Invalid("loft sections".into()));
    };
    let direction = profile::centroid(last_plane, &last_region.outer) - profile::centroid(first_plane, &first_region.outer);
    let mut loops: Vec<Vec<Vec<ProfileEdge>>> = Vec::with_capacity(sections.len());
    for (plane, region) in sections {
        if !profile::frame_ok(plane) {
            return Err(KernelError::Invalid("a section's frame is not finite and orthonormal".into()));
        }
        profile::region_ok(region)?;
        // Every section turns the same way seen along the loft, whichever way its plane faces.
        let numbered = profile::numbered(region, false)?;
        let numbered = if plane.normal().dot(direction) < 0.0 { numbered.map_loops(|l| l.reversed()) } else { numbered };
        loops.push(profile::edges(plane, &numbered)?);
    }
    let holes = loops.first().map(|l| l.len()).unwrap_or(0);
    if loops.iter().any(|l| l.len() != holes) {
        return Err(KernelError::Invalid("each section of a loft needs the same number of holes".into()));
    }
    let ruled = sections.len() == 2;
    let skin = |k: usize| -> Result<cadrum::Solid> {
        let wires: Vec<Vec<&cadrum::Edge>> =
            loops.iter().map(|l| l.get(k).map(|lp| lp.iter().map(|e| &e.edge).collect()).unwrap_or_default()).collect();
        guard("loft", || cadrum::Solid::loft(wires.iter().map(|w| w.iter().copied()), ruled).map_err(|e| failed("loft", e)))
    };
    let solid = skin(0)?;
    let tools: Vec<cadrum::Solid> = (1..holes).map(skin).collect::<Result<_>>()?;
    let solid = without("loft", solid, &tools)?;
    let end = End { point: last_plane.origin, normal: last_plane.normal() };
    let Some(first_loops) = loops.first() else { return Err(KernelError::Invalid("loft sections".into())) };
    placed("loft", solid, first_plane, first_region, first_loops, Some(end))
}

/// A solid from a planar loop to an apex point: not on this backend yet (OpenCASCADE skins to
/// a vertex, which cadrum does not expose; lzpel/cadrum#317).
pub fn loft_to_point(_plane: &Plane, _lp: &Loop2, _apex: Vec3) -> Result<Body> {
    Err(unsupported("a loft to a point"))
}

/// A station of a polyline sweep: the profile's frame at the start of a segment and the
/// segment's direction.
struct Station {
    frame: Plane,
    dir: Vec3,
}

/// The distinct corners of a polyline, runs of collinear points merged.
fn corners(pts: &[Vec3]) -> Vec<Vec3> {
    let mut out: Vec<Vec3> = Vec::with_capacity(pts.len());
    for p in pts {
        match (out.len(), out.last().copied()) {
            (0, _) => out.push(*p),
            (_, Some(last)) if last.dist(*p) <= 1e-9 => {}
            (1, _) => out.push(*p),
            (n, Some(last)) => {
                let prev = out.get(n - 2).copied().unwrap_or(last);
                let (a, b) = ((last - prev).normalized(), (*p - last).normalized());
                match (a, b) {
                    (Some(a), Some(b)) if a.cross(b).len() < 1e-9 && a.dot(b) > 0.0 => {
                        if let Some(slot) = out.last_mut() {
                            *slot = *p;
                        }
                    }
                    _ => out.push(*p),
                }
            }
            _ => out.push(*p),
        }
    }
    out
}

/// The part of `solid` on the side of the plane through `origin` its `normal` points to.
fn clipped(what: &str, solid: cadrum::Solid, origin: Vec3, normal: Vec3) -> Result<cadrum::Solid> {
    guard(what, || {
        let tool = cadrum::Solid::half_space(dv(origin), dv(normal));
        let parts = (cadrum::Boolean::from(&solid) * &tool).build_vec().map_err(|e| failed(what, e))?;
        parts
            .into_iter()
            .filter(|s| s.volume() > 0.0)
            .max_by(|x, y| x.volume().total_cmp(&y.volume()))
            .ok_or_else(|| KernelError::Failed(format!("{what}: a mitre cut away a whole segment")))
    })
}

/// Sweep `region` along the polyline `pts`: a prism per segment, the profile turned at each
/// corner as the path turns, the prisms mitred where they meet (cut at the plane bisecting
/// the corner) and joined.
fn mitred(what: &str, plane: &Plane, region: &Region2, pts: &[Vec3]) -> Result<Body> {
    let loops = loops_on(plane, region)?;
    let numbered = profile::numbered(region, false)?;
    let points = corners(pts);
    if points.len() < 2 {
        return Err(KernelError::Invalid("the path has no length".into()));
    }
    let closed = points.first().zip(points.last()).is_some_and(|(a, b)| a.dist(*b) <= 1e-9) && points.len() > 3;
    let dirs: Vec<Vec3> = points.windows(2).map(|w| (w[1] - w[0]).normalized().unwrap_or(Vec3::Z)).collect();
    let segs = dirs.len();
    // The profile's frame at each segment: turned at every corner as the path turns.
    let mut stations: Vec<Station> = Vec::with_capacity(segs);
    let mut frame = *plane;
    for (i, dir) in dirs.iter().enumerate() {
        if i > 0 {
            // Carried along the segment before, then turned at its corner.
            frame.origin += points[i] - points[i - 1];
            let (a, b) = (dirs[i - 1], *dir);
            let axis = a.cross(b);
            if let Some(k) = axis.normalized() {
                let angle = axis.len().atan2(a.dot(b));
                let corner = points[i];
                frame = Plane {
                    origin: rotate_about(frame.origin, corner, k, angle),
                    x: rotate_about(frame.x, Vec3::ZERO, k, angle),
                    y: rotate_about(frame.y, Vec3::ZERO, k, angle),
                };
            }
        }
        stations.push(Station { frame, dir: *dir });
    }
    // How far a prism reaches past its corner before the mitre: the profile's extent plus
    // the path's offset from it.
    let reach = region
        .outer
        .segs
        .iter()
        .flat_map(|s| (0..8).map(move |i| s.point_at(i as f64 / 8.0)))
        .map(|q| (plane.to_world(q) - plane.origin).len())
        .fold(0.0, f64::max);
    let ext = 2.0 * (reach + (plane.origin - points[0]).len()) + 1.0;
    let bisector = |i: usize| -> Result<Vec3> {
        let (a, b) = (dirs[(i + segs - 1) % segs], dirs[i % segs]);
        (a + b).normalized().ok_or_else(|| KernelError::Invalid("the path turns back on itself".into()))
    };
    let mut prisms = Vec::with_capacity(segs);
    for (i, st) in stations.iter().enumerate() {
        let (cut_start, cut_end) = (i > 0 || closed, i + 1 < segs || closed);
        let (start_ext, end_ext) = (if cut_start { ext } else { 0.0 }, if cut_end { ext } else { 0.0 });
        let shifted = Plane { origin: st.frame.origin - st.dir * start_ext, x: st.frame.x, y: st.frame.y };
        let edges = profile::edges(&shifted, &numbered)?;
        let flat: Vec<&cadrum::Edge> = edges.iter().flatten().map(|e| &e.edge).collect();
        let length = (points[i + 1] - points[i]).len() + start_ext + end_ext;
        let mut solid = guard(what, || cadrum::Solid::extrude(flat.iter().copied(), dv(st.dir * length)).map_err(|e| failed(what, e)))?;
        if cut_start {
            solid = clipped(what, solid, points[i], bisector(i)?)?;
        }
        if cut_end {
            solid = clipped(what, solid, points[i + 1], -bisector(i + 1)?)?;
        }
        prisms.push(solid);
    }
    let solid = guard(what, || {
        let Some(first) = prisms.first() else { return Err(KernelError::Invalid("the path has no length".into())) };
        let mut expr = cadrum::Boolean::from(first);
        for p in prisms.iter().skip(1) {
            expr = expr + p;
        }
        let parts = expr.build_vec().map_err(|e| failed(what, e))?;
        let joined = parts
            .into_iter()
            .filter(|s| s.volume() > 0.0)
            .max_by(|x, y| x.volume().total_cmp(&y.volume()))
            .ok_or_else(|| KernelError::Failed(format!("{what}: nothing swept")))?;
        Ok(joined.clean().unwrap_or(joined))
    })?;
    let end = (!closed).then(|| {
        let last = stations.last().map(|s| s.dir).unwrap_or(Vec3::Z);
        End { point: points[segs], normal: last }
    });
    let along: Vec<(Plane, Vec3)> = stations.iter().map(|s| (s.frame, s.dir)).collect();
    placed_along(what, solid, plane, &loops, &along, end)
}

/// `solid` swept through `stations`, with every face placed: a side on the profile segment it
/// runs along (its points carried back along its station's direction onto the profile), a cap
/// at the start (on `plane`) or the end.
fn placed_along(
    what: &str,
    solid: cadrum::Solid,
    plane: &Plane,
    loops: &[Vec<ProfileEdge>],
    stations: &[(Plane, Vec3)],
    end: Option<End>,
) -> Result<Body> {
    let body = Body::from_solids(vec![solid])?;
    let size = body.size().max(1e-9);
    let tol = size * 1e-5 + 1e-9;
    let n = plane.normal();
    let edges: Vec<&ProfileEdge> = loops.iter().flatten().collect();
    let segs: Vec<Seg2> = edges.iter().map(|e| e.segment).collect();
    let on =
        |center: Vec3, normal: Vec3, origin: Vec3, axis: Vec3| (normal.dot(axis).abs() - 1.0).abs() < 1e-6 && (center - origin).dot(axis).abs() < tol;
    let distance = |q: Vec2| segs.iter().map(|s| (s.point_at(s.closest_param(q)) - q).len()).fold(f64::INFINITY, f64::min);
    let nearest = |q: Vec2| {
        segs.iter().enumerate().map(|(k, s)| (k, (s.point_at(s.closest_param(q)) - q).len())).min_by(|a, b| a.1.total_cmp(&b.1)).map(|(k, _)| k)
    };
    let inner = body.inner()?;
    let tess = cadrum::Tessellation { deflection_linear: size * 1e-3, deflection_angular: 0.5, relative_linear: false };
    let faces = inner.with(what, |solids| {
        let mut faces = Vec::with_capacity(inner.faces.len());
        for i in 0..inner.faces.len() {
            let Some(face) = inner.face(solids, i) else {
                faces.push(generated(Vec::new()));
                continue;
            };
            let center = v3(face.center());
            let normal = v3(face.project(dv(center)).1).normalized().unwrap_or(Vec3::Z);
            let planar = face.surface().map(|s| s.kind) == Some(cadrum::SurfaceKind::Plane);
            if planar && on(center, normal, plane.origin, n) {
                faces.push(generated(vec![FaceOrigin::Cap { end: false }]));
                continue;
            }
            if planar && end.is_some_and(|e| on(center, normal, e.point, e.normal)) {
                faces.push(generated(vec![FaceOrigin::Cap { end: true }]));
                continue;
            }
            let mut samples = vec![center];
            for e in face.iter_edge() {
                samples.extend(e.approximation_segments(tess).into_iter().map(v3).take(16));
            }
            // The station whose direction carries the face's points onto the profile outline.
            let mut best: Option<(f64, usize)> = None;
            for (frame, dir) in stations {
                let fnorm = frame.normal();
                let denom = dir.dot(fnorm);
                if denom.abs() < 1e-9 {
                    continue;
                }
                let back = |p: Vec3| frame.to_local(p + *dir * ((frame.origin - p).dot(fnorm) / denom));
                let worst = samples.iter().map(|p| distance(back(*p))).fold(0.0, f64::max);
                if best.is_none_or(|b| worst < b.0)
                    && let Some(k) = nearest(back(center))
                {
                    best = Some((worst, k));
                }
            }
            match best {
                Some((worst, k)) if worst <= size * 1e-3 + 1e-6 => {
                    let numbers: std::collections::BTreeSet<usize> = edges.get(k).map(|e| e.numbers.iter().copied().collect()).unwrap_or_default();
                    faces.push(generated(numbers.into_iter().map(|segment| FaceOrigin::Profile { segment }).collect()));
                }
                _ => faces.push(generated(Vec::new())),
            }
        }
        Ok(faces)
    })?;
    guard(what, || attach(body, faces))
}

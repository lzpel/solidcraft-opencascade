//! Fillets, chamfers, shells and face offsets: OpenCASCADE's local operations through cadrum,
//! with the history read back from the pairs they report. The builders report modified faces
//! only (lzpel/cadrum#317 item 15), so a face new to the result is placed by geometry: a blend
//! face on the picked edge nearest its centre, a shell's inner wall on the input face it lines
//! (at the wall's thickness from it), a shell's rim on the open face it lies in.

use std::sync::Arc;

use solidcraft_geom::Vec3;
use solidcraft_kernel_core::history::{Change, EdgeOrigin, FaceOrigin, History, Side};

use crate::body::{Body, Inner, v3};
use crate::build::meeting;
use crate::record::{Operand, Recorded, with_history_or_log};
use crate::{DISPLAY_ANGLE, FaceSource, KernelError, Result, guard, unsupported};

const MAX_PICKS: usize = 1000;

/// A local operation on the one solid of `body`: the result and its history over `[body]` as
/// far as the pairs OpenCASCADE reported take it (a face it did not report is generated
/// without origins, for the caller to place).
fn rebuilt(
    body: &Body,
    what: &str,
    build: impl FnOnce(&cadrum::Solid, &Inner, &[cadrum::Solid]) -> Result<cadrum::Solid>,
) -> Result<(Body, Result<History>)> {
    body.require_brep(what)?;
    let inner = body.inner()?;
    if inner.lumps != 1 {
        return Err(unsupported(&format!("{what} on a body of several lumps")));
    }
    let size = body.size();
    let (out, operand, rec) = inner.with(what, |solids| {
        let solid = solids.first().ok_or_else(|| KernelError::Failed(format!("{what}: empty body")))?;
        let out = guard(what, || build(solid, inner, solids))?;
        let pairs: Vec<[u64; 2]> = out.iter_history().collect();
        Ok((out, Operand::from_solids(inner, solids), Recorded { pairs, cleaned: Vec::new() }))
    })?;
    let volume = guard(what, || Ok(out.volume()))?;
    if !(volume.is_finite() && volume > 0.0) {
        return Err(KernelError::Failed(format!("{what} left no volume")));
    }
    let result = Inner::snapshot(vec![out])?;
    let history = crate::record::history(&result, std::slice::from_ref(&operand), &rec, size);
    Ok((Body::from_inner(result), history))
}

/// `result` with `history` over `[input]` and the face sources the history carries over.
fn finish(result: Body, history: Result<History>, input: &Body) -> Body {
    let result = match &history {
        Ok(h) => {
            let sources = h.face_sources(&[input]);
            result.with_face_sources(sources.iter().any(Option::is_some).then(|| Arc::new(sources)))
        }
        Err(_) => result,
    };
    with_history_or_log(result, history, &[input])
}

/// The edges of `body` nearest the given points, each once. A seam under a point (the line a
/// wall closes on) is taken only when no other edge is there.
fn pick_edges(body: &Body, points: &[Vec3], what: &str) -> Result<Vec<usize>> {
    body.require_brep(what)?;
    if points.is_empty() || points.len() > MAX_PICKS {
        return Err(KernelError::Invalid(format!("select 1…{MAX_PICKS} edges")));
    }
    let size = body.size();
    let m = crate::tess::mesh(body, (size * 1e-3).max(1e-3), DISPLAY_ANGLE)?;
    let accept = size * 1e-2 + 1e-3;
    let seam = |i: usize| m.seams.get(i).copied().unwrap_or(false);
    let mut out = Vec::with_capacity(points.len());
    for p in points {
        if !p.is_finite() {
            return Err(KernelError::Invalid("edge points must be finite".into()));
        }
        let mut best: Option<(usize, f64)> = None;
        for (i, pts) in m.edges.iter().enumerate() {
            let Some(d) = polyline_distance(*p, pts) else { continue };
            if d > accept {
                continue;
            }
            let better = match best {
                None => true,
                Some((j, bd)) => (seam(j) && !seam(i)) || (seam(j) == seam(i) && d < bd),
            };
            if better {
                best = Some((i, d));
            }
        }
        match best {
            Some((i, _)) => {
                if !out.contains(&i) {
                    out.push(i);
                }
            }
            None => return Err(KernelError::Invalid(format!("no edge of the body near [{}, {}, {}]", p.x, p.y, p.z))),
        }
    }
    Ok(out)
}

/// The faces of `body` at the given points (through its display mesh), each once.
fn pick_faces(body: &Body, points: &[Vec3], what: &str) -> Result<Vec<usize>> {
    body.require_brep(what)?;
    if points.is_empty() || points.len() > MAX_PICKS {
        return Err(KernelError::Invalid(format!("select 1…{MAX_PICKS} faces")));
    }
    let size = body.size();
    let m = crate::tess::mesh(body, (size * 1e-3).max(1e-3), DISPLAY_ANGLE)?;
    let mut out = Vec::with_capacity(points.len());
    for p in points {
        if !p.is_finite() {
            return Err(KernelError::Invalid("face points must be finite".into()));
        }
        let near = m
            .triangles
            .iter()
            .zip(&m.tri_face)
            .filter_map(|(t, f)| m.tri(t).map(|[a, b, c]| (point_triangle(*p, a, b, c), *f as usize)))
            .min_by(|a, b| a.0.total_cmp(&b.0));
        match near {
            Some((d, f)) if d <= size * 1e-2 + 1e-3 => {
                if !out.contains(&f) {
                    out.push(f);
                }
            }
            _ => return Err(KernelError::Invalid(format!("no face of the body at [{}, {}, {}]", p.x, p.y, p.z))),
        }
    }
    Ok(out)
}

/// The distance from `p` to the triangle `abc`.
fn point_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f64 {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let n = ab.cross(ac);
    let n2 = n.dot(n);
    if n2 > 1e-30 {
        // Inside the triangle's projection: the distance to its plane.
        let w = ap.cross(ac).dot(n) / n2;
        let v = ab.cross(ap).dot(n) / n2;
        if w >= 0.0 && v >= 0.0 && w + v <= 1.0 {
            return (ap.dot(n) / n2.sqrt()).abs();
        }
    }
    [(a, b), (b, c), (c, a)].into_iter().map(|(s, e)| segment_distance(p, s, e)).fold(f64::INFINITY, f64::min)
}

fn segment_distance(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let l2 = ab.dot(ab);
    let t = if l2 > 0.0 { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
    (a + ab * t - p).len()
}

fn polyline_distance(p: Vec3, pts: &[Vec3]) -> Option<f64> {
    match pts {
        [] => None,
        [one] => Some((*one - p).len()),
        _ => pts.windows(2).map(|w| segment_distance(p, w[0], w[1])).reduce(f64::min),
    }
}

/// The centre of every face of `body`, in face order.
fn centres(body: &Body) -> Result<Vec<Vec3>> {
    let inner = body.inner()?;
    inner.with("face centres", |solids| {
        Ok((0..inner.faces.len()).map(|i| inner.face(solids, i).map(|f| v3(f.center())).unwrap_or(Vec3::ZERO)).collect())
    })
}

/// Each face of `h` new to the operation (generated, no origins) placed on the picked edge of
/// `input` nearest its centre: a blend made in that edge's place.
fn place_on_edges(h: &mut History, result: &Body, input: &Body, picked: &[usize]) -> Result<()> {
    let size = input.size();
    let tess = cadrum::Tessellation { deflection_linear: (size * 1e-3).max(1e-6), deflection_angular: 0.3, relative_linear: false };
    let inner = input.inner()?;
    let lines: Vec<(usize, Vec<Vec3>)> = inner.with("blend edges", |solids| {
        Ok(picked
            .iter()
            .filter_map(|&e| inner.edge(solids, e).map(|edge| (e, edge.approximation_segments(tess).into_iter().map(v3).collect())))
            .collect())
    })?;
    let centres = centres(result)?;
    for (i, f) in h.faces.iter_mut().enumerate() {
        if f.change != Change::Generated || !f.origins.is_empty() {
            continue;
        }
        let Some(c) = centres.get(i) else { continue };
        let best = lines.iter().filter_map(|(e, pts)| polyline_distance(*c, pts).map(|d| (*e, d))).min_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((e, _)) = best {
            f.origins = vec![FaceOrigin::Edge { operand: 0, edge: e }];
        }
    }
    Ok(())
}

/// Each edge of `h` without origins (between faces that had none when the history was read)
/// takes the meeting of its faces now that they are placed.
fn settle_edges(h: &mut History) {
    for e in &mut h.edges {
        if e.origins.is_empty() {
            e.origins = meeting(&e.faces, &h.faces);
        }
    }
}

fn blend(body: &Body, picked: &[usize], what: &str, op: impl FnOnce(&cadrum::Solid, Vec<&cadrum::Edge>) -> Result<cadrum::Solid>) -> Result<Body> {
    let (result, history) = rebuilt(body, what, |solid, inner, solids| {
        let es: Vec<&cadrum::Edge> = picked.iter().filter_map(|&i| inner.edge(solids, i)).collect();
        if es.len() != picked.len() {
            return Err(KernelError::Failed(format!("{what}: an edge is missing")));
        }
        op(solid, es)
    })?;
    let history = history.and_then(|mut h| {
        place_on_edges(&mut h, &result, body, picked)?;
        settle_edges(&mut h);
        Ok(h)
    });
    Ok(finish(result, history, body))
}

fn too_large(what: &str, size: f64, e: cadrum::Error) -> KernelError {
    KernelError::TooLarge(format!("a {what} of {size} mm does not fit the faces around the edges ({e})"))
}

/// Constant-radius fillet of the edges nearest to the given points. A radius too large for the
/// faces around the edges is refused with [`KernelError::TooLarge`] ([`fillet_max`] finds the
/// largest that works).
pub fn fillet(body: &Body, edges: &[Vec3], radius: f64) -> Result<Body> {
    body.require_brep("fillet")?;
    if !(radius.is_finite() && radius > 0.0 && radius < 1e6) {
        return Err(KernelError::Invalid("the blend radius must be finite and positive".into()));
    }
    let picked = pick_edges(body, edges, "fillet")?;
    blend(body, &picked, "fillet", |solid, es| solid.fillet_edges(radius, es).map_err(|e| too_large("fillet", radius, e)))
}

/// Equal-distance chamfer of the edges nearest to the given points.
pub fn chamfer(body: &Body, edges: &[Vec3], distance: f64) -> Result<Body> {
    body.require_brep("chamfer")?;
    if !(distance.is_finite() && distance > 1e-6 && distance < 1e6) {
        return Err(KernelError::Invalid("chamfer size must be positive".into()));
    }
    let picked = pick_edges(body, edges, "chamfer")?;
    blend(body, &picked, "chamfer", |solid, es| solid.chamfer_edges(distance, es).map_err(|e| too_large("chamfer", distance, e)))
}

/// The step of the radius search: three significant digits of the requested radius.
fn radius_step(radius: f64) -> f64 {
    10f64.powf(radius.log10().floor() - 3.0)
}

/// The largest radius below `radius` (to three significant digits) at which the fillet of
/// these edges works, with its result; `None` when none does. Each candidate is a real fillet
/// of the edges (up to 16 of them), so the radius returned is one that was built.
pub fn fillet_max(body: &Body, edges: &[Vec3], radius: f64) -> Option<(f64, Body)> {
    if !(radius.is_finite() && radius > 0.0 && radius < 1e6) || body.is_mesh() {
        return None;
    }
    let step = radius_step(radius);
    let (mut lo, mut hi) = (0.0, radius);
    let mut best = None;
    for _ in 0..16 {
        let mid = ((lo + hi) * 0.5 / step).floor() * step;
        if mid <= lo || mid <= 0.0 {
            break;
        }
        match fillet(body, edges, mid) {
            Ok(b) => {
                best = Some((mid, b));
                lo = mid;
            }
            Err(_) => hi = mid,
        }
        if hi - lo <= step {
            break;
        }
    }
    best
}

/// [`fillet_max`] with the radius repeated per edge: this backend finds one radius for all the
/// edges (OpenCASCADE builds them together).
pub fn fillet_max_each(body: &Body, edges: &[Vec3], radius: f64) -> Option<(Vec<f64>, Body)> {
    fillet_max(body, edges, radius).map(|(r, b)| (vec![r; edges.len()], b))
}

/// The fillet of each edge at its own radius (`radii` in `edges`' order): the edges of each
/// radius together, the largest first, with the history composed over the input.
pub fn fillet_each(body: &Body, edges: &[Vec3], radii: &[f64]) -> Result<Body> {
    body.require_brep("fillet")?;
    if edges.len() != radii.len() || edges.is_empty() {
        return Err(KernelError::Invalid("one radius per edge".into()));
    }
    if radii.iter().any(|r| !(r.is_finite() && *r > 0.0 && *r < 1e6)) {
        return Err(KernelError::Invalid("the blend radius must be finite and positive".into()));
    }
    let mut distinct: Vec<f64> = radii.to_vec();
    distinct.sort_by(|a, b| b.total_cmp(a));
    distinct.dedup();
    if distinct.len() == 1 {
        return fillet(body, edges, radii.first().copied().unwrap_or(0.0));
    }
    let mut cur = body.clone();
    let mut composed: Option<History> = None;
    for r in distinct {
        let group: Vec<Vec3> = edges.iter().zip(radii).filter(|(_, rr)| **rr == r).map(|(p, _)| *p).collect();
        let next = fillet(&cur, &group, r)?;
        composed = match (next.history(), composed) {
            (Some(outer), Some(inner)) => Some(compose(outer, &inner)),
            (Some(outer), None) => Some(outer.clone()),
            (None, _) => None,
        };
        cur = next;
    }
    Ok(match composed {
        Some(h) => finish(cur, Ok(h), body),
        None => cur,
    })
}

/// The fillet of each edge at the radius given with it.
pub fn fillet_radii(body: &Body, edges: &[(Vec3, f64)]) -> Result<Body> {
    let (points, radii): (Vec<Vec3>, Vec<f64>) = edges.iter().copied().unzip();
    fillet_each(body, &points, &radii)
}

/// `outer` (over the result of `inner`) over `inner`'s operand: each origin of `outer` that
/// names a face or edge of the intermediate body replaced by that face's or edge's origins.
fn compose(outer: &History, inner: &History) -> History {
    let face_origins = |o: &FaceOrigin| -> Vec<FaceOrigin> {
        match *o {
            FaceOrigin::Face { operand: 0, face } => inner.faces.get(face).map(|f| f.origins.clone()).unwrap_or_default(),
            FaceOrigin::Edge { operand: 0, edge } => inner
                .edges
                .get(edge)
                .map(|e| {
                    e.origins
                        .iter()
                        .filter_map(|eo| match eo {
                            EdgeOrigin::Edge { operand, edge } => Some(FaceOrigin::Edge { operand: *operand, edge: *edge }),
                            EdgeOrigin::Meet { .. } => None,
                        })
                        .collect()
                })
                .unwrap_or_default(),
            FaceOrigin::Offset { operand: 0, face } => inner
                .faces
                .get(face)
                .map(|f| {
                    f.origins
                        .iter()
                        .filter_map(|fo| match fo {
                            FaceOrigin::Face { operand, face } => Some(FaceOrigin::Offset { operand: *operand, face: *face }),
                            _ => None,
                        })
                        .collect()
                })
                .unwrap_or_default(),
            other => vec![other],
        }
    };
    let faces = outer
        .faces
        .iter()
        .map(|f| {
            let mut origins: Vec<FaceOrigin> = f.origins.iter().flat_map(face_origins).collect();
            origins.sort_unstable();
            origins.dedup();
            let change = match (f.change, f.origins.as_slice()) {
                (Change::Kept, [FaceOrigin::Face { operand: 0, face }]) => inner.faces.get(*face).map(|g| g.change).unwrap_or(Change::Kept),
                (c, _) => c,
            };
            solidcraft_kernel_core::history::FaceProv { origins, change }
        })
        .collect();
    let side = |s: &Side| Side::new(s.origins().iter().flat_map(face_origins));
    let edges = outer
        .edges
        .iter()
        .map(|e| {
            let origins = e
                .origins
                .iter()
                .flat_map(|o| match o {
                    EdgeOrigin::Edge { operand: 0, edge } => inner.edges.get(*edge).map(|x| x.origins.clone()).unwrap_or_default(),
                    EdgeOrigin::Meet { a, b } => match (side(a), side(b)) {
                        (Some(a), Some(b)) => vec![EdgeOrigin::meet(a, b)],
                        _ => Vec::new(),
                    },
                    other => vec![other.clone()],
                })
                .collect();
            solidcraft_kernel_core::history::EdgeProv { faces: e.faces.clone(), origins }
        })
        .collect();
    History { operands: inner.operands, faces, edges }
}

/// Each face of `h` new to a shell placed by where it lies: at the wall's thickness from an
/// input face, it is that face's offset (an inner wall); in an input face, it is part of that
/// face (the rim of an opening).
fn place_shell_faces(h: &mut History, result: &Body, input: &Body, thickness: f64) -> Result<()> {
    let size = input.size();
    let m = crate::tess::mesh(input, (size * 1e-3).max(1e-3), DISPLAY_ANGLE)?;
    let centres = centres(result)?;
    let tol = size * 1e-3 + 1e-6;
    for (i, f) in h.faces.iter_mut().enumerate() {
        if f.change != Change::Generated || !f.origins.is_empty() {
            continue;
        }
        let Some(c) = centres.get(i) else { continue };
        let near = m
            .triangles
            .iter()
            .zip(&m.tri_face)
            .filter_map(|(t, face)| m.tri(t).map(|[a, b, cc]| (point_triangle(*c, a, b, cc), *face as usize)))
            .min_by(|a, b| a.0.total_cmp(&b.0));
        let Some((d, face)) = near else { continue };
        if d <= tol {
            f.origins = vec![FaceOrigin::Face { operand: 0, face }];
            f.change = Change::Modified;
        } else if (d - thickness).abs() <= thickness * 0.25 + tol {
            f.origins = vec![FaceOrigin::Offset { operand: 0, face }];
        }
    }
    Ok(())
}

/// Hollow `b`: walls of `thickness` inside, the faces at `open` removed. The result's history
/// is over `[b]`: the inner walls are offsets of the faces they line, the rims parts of the
/// faces removed. A planar wall lining a face with a planar source takes a shell source over
/// it, so sketches attach to it exactly.
pub fn shell(b: &Body, open: &[Vec3], thickness: f64) -> Result<Body> {
    b.require_brep("shell")?;
    if !(thickness.is_finite() && thickness > 1e-6 && thickness < 1e6) {
        return Err(KernelError::Invalid("shell thickness must be positive".into()));
    }
    if open.is_empty() {
        return Err(KernelError::Invalid("select at least one face to remove".into()));
    }
    let picked = crate::query::with_same_surface(b, &pick_faces(b, open, "shell")?);
    let (result, history) = rebuilt(b, "shell", |solid, inner, solids| {
        let faces: Vec<&cadrum::Face> = picked.iter().filter_map(|&i| inner.face(solids, i)).collect();
        if faces.len() != picked.len() {
            return Err(KernelError::Failed("shell: a face is missing".into()));
        }
        // The cavity: every face but the open ones moved in by the thickness (sharp where
        // walls meet, as the truck backend builds it), taken out of the body. The thick-solid
        // builder (rounded joins) stands in where the offset fails.
        let keep: Vec<&cadrum::Face> = (0..inner.faces.len()).filter(|i| !picked.contains(i)).filter_map(|i| inner.face(solids, i)).collect();
        let hollowed = solid.offset(-thickness, keep).map_err(|e| KernelError::Failed(format!("shell: {e}"))).and_then(|cavity| {
            let parts = (cadrum::Boolean::from(solid) - &cavity).build_vec().map_err(|e| KernelError::Failed(format!("shell: {e}")))?;
            parts
                .into_iter()
                .filter(|s| s.volume() > 0.0)
                .max_by(|x, y| x.volume().total_cmp(&y.volume()))
                .ok_or_else(|| KernelError::Failed("shell: the walls removed everything".into()))
        });
        match hollowed {
            Ok(out) => Ok(out),
            Err(e) => {
                log::debug!("shell by offset failed, thick solid instead: {e}");
                solid.shell(-thickness, faces).map_err(|e| KernelError::Failed(format!("shell: {e}")))
            }
        }
    })?;
    let history = history.and_then(|mut h| {
        place_shell_faces(&mut h, &result, b, thickness)?;
        settle_edges(&mut h);
        Ok(h)
    });
    let result = match &history {
        Ok(h) => {
            let input_sources = b.face_sources().unwrap_or_default();
            let mut sources = h.face_sources(&[b]);
            for (i, slot) in sources.iter_mut().enumerate() {
                let Some([FaceOrigin::Offset { operand: 0, face }]) = h.faces.get(i).map(|f| f.origins.as_slice()) else { continue };
                let (Some(Some(source)), Some(plane)) = (input_sources.get(*face), result.face_plane(i)) else { continue };
                if source.clone().plane_mut().is_none() {
                    continue;
                }
                *slot = Some(FaceSource::ShellPlanar { producer: None, source: Box::new(source.clone()), inner: true, plane });
            }
            result.with_face_sources(sources.iter().any(Option::is_some).then(|| Arc::new(sources)))
        }
        Err(_) => result,
    };
    Ok(with_history_or_log(result, history, &[b]))
}

/// Move the faces at the given points along their normals by `distance` (negative: into the
/// body). The body's other faces follow (its topology is kept); a moved planar face's source
/// moves with it.
pub fn offset_faces(b: &Body, at: &[Vec3], distance: f64) -> Result<Body> {
    b.require_brep("offset faces")?;
    if !distance.is_finite() || distance.abs() > 1e6 {
        return Err(KernelError::Invalid("offset distance".into()));
    }
    if distance.abs() < 1e-9 {
        return Ok(b.clone());
    }
    let picked = crate::query::with_same_surface(b, &pick_faces(b, at, "offset faces")?);
    let (result, history) = rebuilt(b, "offset faces", |solid, inner, solids| {
        let faces: Vec<&cadrum::Face> = picked.iter().filter_map(|&i| inner.face(solids, i)).collect();
        if faces.len() != picked.len() {
            return Err(KernelError::Failed("offset faces: a face is missing".into()));
        }
        solid.offset(distance, faces).map_err(|e| KernelError::Failed(format!("offset faces: {e}")))
    })?;
    let history = history.map(|mut h| {
        settle_edges(&mut h);
        h
    });
    let result = match &history {
        Ok(h) => {
            let mut sources = h.face_sources(&[b]);
            for (i, slot) in sources.iter_mut().enumerate() {
                let Some([FaceOrigin::Face { operand: 0, face }]) = h.faces.get(i).map(|f| f.origins.as_slice()) else { continue };
                if !picked.contains(face) {
                    continue;
                }
                if let (Some(source), Some(plane)) = (slot.as_mut(), result.face_plane(i))
                    && let Some(p) = source.plane_mut()
                {
                    *p = plane;
                }
            }
            result.with_face_sources(sources.iter().any(Option::is_some).then(|| Arc::new(sources)))
        }
        Err(_) => result,
    };
    Ok(with_history_or_log(result, history, &[b]))
}

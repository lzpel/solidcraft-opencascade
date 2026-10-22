//! Booleans and transforms over cadrum solids.

use solidcraft_geom::{Plane, Region2, Vec3};
use solidcraft_kernel_core::history::History;

use crate::body::{Body, Inner, cadrum_handle, dv};
use crate::types::BoolOp;
use crate::{KernelError, Result, failed, unsupported};

/// A boolean of two bodies: `None` where nothing is left. Histories and face sources arrive
/// with phase 4; until then the result carries neither.
pub fn boolean(a: &Body, b: &Body, op: BoolOp) -> Result<Option<Body>> {
    a.require_brep("boolean")?;
    b.require_brep("boolean")?;
    let (ia, ib) = (a.inner()?, b.inner()?);
    let what = match op {
        BoolOp::Union => "union",
        BoolOp::Cut => "cut",
        BoolOp::Intersect => "intersection",
    };
    let results = Inner::with2(ia, ib, what, |sa, sb| {
        let (Some(a0), Some(b0)) = (sa.first(), sb.first()) else { return Err(KernelError::Failed("empty operand".into())) };
        let mut out = Vec::new();
        match op {
            BoolOp::Union => {
                let mut expr = cadrum::Boolean::from(a0);
                for s in sa.iter().skip(1).chain(sb.iter()) {
                    expr = expr + s;
                }
                out.extend(expr.build_vec().map_err(|e| failed(what, e))?);
            }
            BoolOp::Cut => {
                for s in sa {
                    let mut expr = cadrum::Boolean::from(s);
                    for t in sb {
                        expr = expr - t;
                    }
                    out.extend(expr.build_vec().map_err(|e| failed(what, e))?);
                }
            }
            BoolOp::Intersect => {
                let _ = b0;
                for s in sa {
                    for t in sb {
                        out.extend((s * t).build_vec().map_err(|e| failed(what, e))?);
                    }
                }
            }
        }
        Ok(out)
    })?;
    let results: Vec<cadrum::Solid> = results.into_iter().filter(|s| s.volume() > 0.0).collect();
    if results.is_empty() {
        return Ok(None);
    }
    Ok(Some(Body::from_solids(results)?))
}

/// Does every face of the body lie in a plane?
pub fn is_planar(body: &Body) -> bool {
    let Ok(inner) = body.inner() else { return false };
    inner
        .with("planar", |solids| {
            Ok((0..inner.faces.len()).all(|i| inner.face(solids, i).and_then(|f| f.surface()).is_some_and(|s| s.kind == cadrum::SurfaceKind::Plane)))
        })
        .unwrap_or(false)
}

/// Run `f` with at most `attempts` boolean retries: this backend retries nothing, so `f` runs
/// as it is.
pub fn with_attempt_budget<T>(_attempts: usize, f: impl FnOnce() -> T) -> T {
    f()
}

pub fn can_join_touching_faces(_a: &Body, _b: &Body) -> bool {
    false
}

pub fn can_join_all_touching_faces(_a: &Body, _b: &Body) -> bool {
    false
}

pub fn compound_disjoint_extrusions(_tools: &[Body]) -> Result<Option<Body>> {
    Ok(None)
}

/// The union of bodies that do not touch, as one body of several lumps.
pub fn union_apart(bodies: &[Body]) -> Result<Option<Body>> {
    let mut lumps = Vec::new();
    for b in bodies {
        b.require_brep("union")?;
        let inner = b.inner()?;
        lumps.extend(inner.with("union apart", |solids| Ok(solids.iter().map(cadrum_handle).collect::<Vec<_>>()))?);
    }
    if lumps.is_empty() {
        return Ok(None);
    }
    Ok(Some(Body::from_solids(lumps)?))
}

/// The union of bodies that may touch.
pub fn join_apart(bodies: &[Body]) -> Result<Option<Body>> {
    let mut acc: Option<Body> = None;
    for b in bodies {
        acc = match acc {
            None => Some(b.clone()),
            Some(a) => boolean(&a, b, BoolOp::Union)?,
        };
    }
    Ok(acc)
}

/// The body moved by `m` (column-major, translation in `m[3]`): rigid moves, mirrors and
/// uniform scales; anything else is refused.
pub fn transform_matrix(body: &Body, m: [[f64; 4]; 4]) -> Result<Body> {
    if m.iter().flatten().any(|x| !x.is_finite() || x.abs() >= 1e12) {
        return Err(KernelError::Invalid("transform with non-finite or huge entries".into()));
    }
    let moved = match &body.shape {
        crate::body::Shape::Mesh(mesh) => {
            let apply = |p: Vec3| point(&m, p);
            let moved = solidcraft_kernel_core::meshbody::TriMesh {
                positions: mesh.positions.iter().map(|p| apply(*p)).collect(),
                triangles: mesh.triangles.clone(),
            };
            let mut out = Body::from_mesh(moved);
            out.color = body.color;
            out.paint = body.paint.clone();
            return Ok(out);
        }
        crate::body::Shape::Solid(inner) => {
            let steps = decompose(&m)?;
            let solids = inner.with("transform", |solids| {
                Ok(solids
                    .iter()
                    .map(|s| {
                        let mut s = cadrum_handle(s);
                        for step in &steps {
                            s = match *step {
                                Step::Mirror(n) => s.mirror(cadrum::DVec3::ZERO, dv(n)),
                                Step::Scale(k) => s.scale(cadrum::DVec3::ZERO, k),
                                Step::Rotate(axis, angle) => s.rotate(cadrum::DVec3::ZERO, dv(axis), angle),
                                Step::Translate(t) => s.translate(dv(t)),
                            };
                        }
                        s
                    })
                    .collect::<Vec<_>>())
            })?;
            Body::from_solids(solids)?
        }
    };
    let rigid = decompose(&m).map(|steps| steps.iter().all(|s| matches!(s, Step::Rotate(..) | Step::Translate(_)))).unwrap_or(false);
    let moved = moved.with_boundary_tol(if rigid { body.boundary_tol } else { None }).with_face_sources(body.transformed_sources(m)?);
    let mut moved = moved.with_history(History::identity(body), &[body])?;
    moved.color = body.color;
    moved.paint = body.paint.clone();
    moved.regularization_protected = body.regularization_protected;
    Ok(moved)
}

/// The body placed by `m`: a moved copy that shows the original's display mesh.
pub fn place_matrix(body: &Body, m: [[f64; 4]; 4]) -> Result<Body> {
    if is_identity(&m) {
        return Ok(body.clone());
    }
    let moved = transform_matrix(body, m)?;
    Ok(match body.display_share() {
        Some((share, t)) => moved.with_display_share(&share, multiply(&m, &t)),
        None if rigid_placement(&m) && !body.is_mesh() && body.splits.is_none() => {
            moved.with_display_share(&crate::body::DisplayShare::new(body.clone()), m)
        }
        None => moved,
    })
}

/// The body turned by `angle` about the axis through `origin`, then moved by `translate`.
pub fn transform(body: &Body, translate: Vec3, origin: Vec3, axis: Vec3, angle: f64) -> Result<Body> {
    let huge = |v: Vec3| v.x.abs() >= 1e9 || v.y.abs() >= 1e9 || v.z.abs() >= 1e9;
    if !(translate.is_finite() && origin.is_finite() && axis.is_finite() && angle.is_finite()) || huge(translate) || huge(origin) {
        return Err(KernelError::Invalid("transform parameters must be finite".into()));
    }
    let m = match axis.normalized() {
        Some(k) if angle != 0.0 => {
            let (s, c) = angle.sin_cos();
            let t = 1.0 - c;
            let r = [
                [t * k.x * k.x + c, t * k.x * k.y + s * k.z, t * k.x * k.z - s * k.y],
                [t * k.x * k.y - s * k.z, t * k.y * k.y + c, t * k.y * k.z + s * k.x],
                [t * k.x * k.z + s * k.y, t * k.y * k.z - s * k.x, t * k.z * k.z + c],
            ];
            let rotated_origin = Vec3::new(
                r[0][0] * origin.x + r[1][0] * origin.y + r[2][0] * origin.z,
                r[0][1] * origin.x + r[1][1] * origin.y + r[2][1] * origin.z,
                r[0][2] * origin.x + r[1][2] * origin.y + r[2][2] * origin.z,
            );
            let d = origin - rotated_origin + translate;
            [[r[0][0], r[0][1], r[0][2], 0.0], [r[1][0], r[1][1], r[1][2], 0.0], [r[2][0], r[2][1], r[2][2], 0.0], [d.x, d.y, d.z, 1.0]]
        }
        _ => [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [translate.x, translate.y, translate.z, 1.0]],
    };
    transform_matrix(body, m)
}

/// The body split by `plane`: the pieces on each side (one body where the plane misses it).
pub fn split_by_plane(body: &Body, plane: &Plane) -> Result<Vec<Body>> {
    body.require_brep("split")?;
    let n = plane.normal();
    if !(plane.origin.is_finite() && n.is_finite() && n.len() > 0.5) {
        return Err(KernelError::Invalid("split plane must be finite".into()));
    }
    let inner = body.inner()?;
    let (above, below) = inner.with("split by plane", |solids| {
        let mut above = Vec::new();
        let mut below = Vec::new();
        for s in solids {
            let half = cadrum::Solid::half_space(dv(plane.origin), dv(n));
            above.extend((s * &half).build_vec().map_err(|e| failed("split", e))?);
            below.extend((s - &half).build_vec().map_err(|e| failed("split", e))?);
        }
        Ok((above, below))
    })?;
    let mut out = Vec::new();
    for piece in [above, below] {
        let piece: Vec<cadrum::Solid> = piece.into_iter().filter(|s| s.volume() > 0.0).collect();
        if !piece.is_empty() {
            out.push(Body::from_solids(piece)?);
        }
    }
    if out.is_empty() {
        return Err(KernelError::Failed("the split left nothing".into()));
    }
    Ok(out)
}

/// The part of the body on `positive`'s side of `plane` (`None` when nothing is), and the body
/// itself: as the truck backend returns it, for callers that keep the whole.
pub fn half_space(body: &Body, plane: &Plane, positive: bool) -> Result<(Option<Body>, Body)> {
    body.require_brep("half space")?;
    let n = plane.normal() * if positive { 1.0 } else { -1.0 };
    let inner = body.inner()?;
    let kept = inner.with("half space", |solids| {
        let mut kept = Vec::new();
        for s in solids {
            let half = cadrum::Solid::half_space(dv(plane.origin), dv(n));
            kept.extend((s * &half).build_vec().map_err(|e| failed("half space", e))?);
        }
        Ok(kept)
    })?;
    let kept: Vec<cadrum::Solid> = kept.into_iter().filter(|s| s.volume() > 0.0).collect();
    Ok((if kept.is_empty() { None } else { Some(Body::from_solids(kept)?) }, body.clone()))
}

pub fn faceted_boolean(_a: &Body, _b: &solidcraft_geom::Mesh, _op: BoolOp, _tol: f64) -> Result<Option<Body>> {
    Err(unsupported("a boolean with a mesh tool"))
}

pub fn planar_boolean(a: &Body, b: &Body, op: BoolOp) -> Result<Option<Body>> {
    boolean(a, b, op)
}

pub fn planar_regions_boolean(_a: &[Region2], _b: &[Region2], _op: BoolOp, _tolerance: f64) -> Result<Vec<Region2>> {
    Err(unsupported("a boolean of sketch regions"))
}

/// One step of a decomposed transform, applied in order.
#[derive(Clone, Copy, Debug)]
enum Step {
    Mirror(Vec3),
    Scale(f64),
    Rotate(Vec3, f64),
    Translate(Vec3),
}

/// `m` as mirror, uniform scale, rotation and translation, or an error for a shear or an
/// anisotropic scale.
fn decompose(m: &[[f64; 4]; 4]) -> Result<Vec<Step>> {
    let cols = [Vec3::new(m[0][0], m[0][1], m[0][2]), Vec3::new(m[1][0], m[1][1], m[1][2]), Vec3::new(m[2][0], m[2][1], m[2][2])];
    let lens = cols.map(|c| c.len());
    let scale = lens[0];
    if !(scale.is_finite() && scale > 1e-12 && scale < 1e12) {
        return Err(KernelError::Invalid("degenerate transform".into()));
    }
    let tol = 1e-9 * scale.max(1.0);
    if (lens[1] - scale).abs() > tol * 1e3 || (lens[2] - scale).abs() > tol * 1e3 {
        return Err(unsupported("a non-uniform scale"));
    }
    let u = cols.map(|c| c * (1.0 / scale));
    if u[0].dot(u[1]).abs() > 1e-6 || u[1].dot(u[2]).abs() > 1e-6 || u[0].dot(u[2]).abs() > 1e-6 {
        return Err(unsupported("a shear"));
    }
    let det = u[0].dot(u[1].cross(u[2]));
    let mut steps = Vec::new();
    // A reflection first: mirror through the XY plane, then a proper rotation makes up the rest.
    let mut r = u;
    if det < 0.0 {
        steps.push(Step::Mirror(Vec3::Z));
        // u = R · H with H = diag(1, 1, -1): R's third column is -u's third column... as columns
        // of a column-major matrix, R = u · H flips the third column.
        r = [u[0], u[1], u[2] * -1.0];
    }
    if (scale - 1.0).abs() > 1e-12 {
        steps.push(Step::Scale(scale));
    }
    if let Some((axis, angle)) = axis_angle(r) {
        steps.push(Step::Rotate(axis, angle));
    }
    let t = Vec3::new(m[3][0], m[3][1], m[3][2]);
    if t != Vec3::ZERO {
        steps.push(Step::Translate(t));
    }
    Ok(steps)
}

/// The axis and angle of the proper rotation with columns `c`, or `None` for the identity.
fn axis_angle(c: [Vec3; 3]) -> Option<(Vec3, f64)> {
    // Row-major entries r[i][j] = component i of column j.
    let r = |i: usize, j: usize| match (i, j) {
        (0, _) => c[j].x,
        (1, _) => c[j].y,
        _ => c[j].z,
    };
    let trace = r(0, 0) + r(1, 1) + r(2, 2);
    let cos = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = cos.acos();
    if angle < 1e-12 {
        return None;
    }
    if angle < std::f64::consts::PI - 1e-6 {
        let axis = Vec3::new(r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1)).normalized()?;
        return Some((axis, angle));
    }
    // Half a turn: the axis is the eigenvector for +1, from the largest diagonal entry.
    let d = [r(0, 0), r(1, 1), r(2, 2)];
    let k = if d[0] >= d[1] && d[0] >= d[2] {
        Vec3::new(r(0, 0) + 1.0, r(1, 0), r(2, 0))
    } else if d[1] >= d[2] {
        Vec3::new(r(0, 1), r(1, 1) + 1.0, r(2, 1))
    } else {
        Vec3::new(r(0, 2), r(1, 2), r(2, 2) + 1.0)
    };
    Some((k.normalized()?, std::f64::consts::PI))
}

fn point(m: &[[f64; 4]; 4], p: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * p.x + m[1][0] * p.y + m[2][0] * p.z + m[3][0],
        m[0][1] * p.x + m[1][1] * p.y + m[2][1] * p.z + m[3][1],
        m[0][2] * p.x + m[1][2] * p.y + m[2][2] * p.z + m[3][2],
    )
}

fn is_identity(m: &[[f64; 4]; 4]) -> bool {
    (0..4).all(|i| (0..4).all(|j| (m[i][j] - if i == j { 1.0 } else { 0.0 }).abs() < 1e-12))
}

/// Is `m` a rotation and translation only?
pub(crate) fn rigid_placement(m: &[[f64; 4]; 4]) -> bool {
    decompose(m).map(|steps| steps.iter().all(|s| matches!(s, Step::Rotate(..) | Step::Translate(_)))).unwrap_or(false)
}

/// `a · b` for column-major matrices (apply `b`, then `a`).
fn multiply(a: &[[f64; 4]; 4], b: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut out = [[0.0; 4]; 4];
    for (j, col) in out.iter_mut().enumerate() {
        for (i, slot) in col.iter_mut().enumerate() {
            *slot = (0..4).map(|k| a[k][i] * b[j][k]).sum();
        }
    }
    out
}

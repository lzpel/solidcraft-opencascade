//! Booleans and transforms over cadrum solids.

use std::sync::Arc;

use solidcraft_geom::{Plane, Region2, Vec3};
use solidcraft_kernel_core::history::{Change, EdgeOrigin, EdgeProv, FaceOrigin, FaceProv, History};

use crate::body::{Body, Inner, cadrum_handle, dv};
use crate::types::{BoolOp, FaceSurface, HalfSpace, SplitTool};
use crate::{KernelError, Result, failed, unsupported};

/// A boolean of two bodies: `None` where nothing is left. The result carries its history over
/// `[a, b]` and the face sources that follow from it. A union's faces on one surface are fused
/// (OpenCASCADE's UnifySameDomain), as the truck backend joins coplanar faces.
pub fn boolean(a: &Body, b: &Body, op: BoolOp) -> Result<Option<Body>> {
    combine(&[a, b], op)
}

/// The boolean `op` of `operands` (the first is the target; a union takes them all).
fn combine(operands: &[&Body], op: BoolOp) -> Result<Option<Body>> {
    for b in operands {
        b.require_brep("boolean")?;
    }
    let inners: Vec<&Arc<Inner>> = operands.iter().map(|b| b.inner()).collect::<Result<_>>()?;
    let what = match op {
        BoolOp::Union => "union",
        BoolOp::Cut => "cut",
        BoolOp::Intersect => "intersection",
    };
    let size = operands.iter().map(|b| b.size()).fold(0.0, f64::max);
    let (results, read, rec) = Inner::with_many(&inners, what, |solids| {
        let mut out: Vec<cadrum::Solid> = Vec::new();
        let mut pairs: Vec<[u64; 2]> = Vec::new();
        let mut collect = |made: Vec<cadrum::Solid>| {
            for s in made {
                pairs.extend(s.iter_history());
                out.push(s);
            }
        };
        let first = solids.first().and_then(|s| s.first()).ok_or_else(|| KernelError::Failed("empty operand".into()))?;
        match op {
            BoolOp::Union => {
                let mut expr = cadrum::Boolean::from(first);
                for s in solids.iter().flat_map(|s| s.iter()).skip(1) {
                    expr = expr + s;
                }
                collect(expr.build_vec().map_err(|e| failed(what, e))?);
            }
            BoolOp::Cut => {
                for s in solids.first().map(|s| s.iter()).into_iter().flatten() {
                    let mut expr = cadrum::Boolean::from(s);
                    for t in solids.iter().skip(1).flat_map(|t| t.iter()) {
                        expr = expr - t;
                    }
                    collect(expr.build_vec().map_err(|e| failed(what, e))?);
                }
            }
            BoolOp::Intersect => {
                for s in solids.first().map(|s| s.iter()).into_iter().flatten() {
                    for t in solids.iter().skip(1).flat_map(|t| t.iter()) {
                        collect((s * t).build_vec().map_err(|e| failed(what, e))?);
                    }
                }
            }
        }
        let mut cleaned: Vec<[u64; 2]> = Vec::new();
        if op == BoolOp::Union {
            out = out
                .into_iter()
                .map(|s| match s.clean() {
                    Ok(c) => {
                        cleaned.extend(c.iter_history());
                        c
                    }
                    Err(e) => {
                        log::debug!("union not cleaned: {e}");
                        s
                    }
                })
                .collect();
        }
        let read: Vec<crate::record::Operand> = inners.iter().zip(solids).map(|(inner, s)| crate::record::Operand::from_solids(inner, s)).collect();
        Ok((out, read, crate::record::Recorded { pairs, cleaned }))
    })?;
    let results: Vec<cadrum::Solid> = results.into_iter().filter(|s| s.volume() > 0.0).collect();
    if results.is_empty() {
        return Ok(None);
    }
    // The largest lump first (it keeps the body's name when a cut leaves several), equal
    // volumes by their lowest corner: OpenCASCADE's order changes between runs, names must not.
    let keyed: Vec<(f64, [f64; 3], cadrum::Solid)> = results
        .into_iter()
        .map(|s| {
            let [lo, _] = s.bounding_box();
            (s.volume(), [lo.x, lo.y, lo.z], s)
        })
        .collect();
    let mut keyed = keyed;
    keyed.sort_by(|a, b| {
        let scale = a.0.abs().max(b.0.abs()).max(1e-9);
        if (a.0 - b.0).abs() > 1e-9 * scale {
            return b.0.total_cmp(&a.0);
        }
        a.1.iter().zip(&b.1).map(|(x, y)| x.total_cmp(y)).find(|o| o.is_ne()).unwrap_or(std::cmp::Ordering::Equal)
    });
    let results: Vec<cadrum::Solid> = keyed.into_iter().map(|(_, _, s)| s).collect();
    let inner = Inner::snapshot(results)?;
    let history = crate::record::history(&inner, &read, &rec, size);
    let body = Body::from_inner(inner);
    let body = match &history {
        Ok(h) => {
            let sources = h.face_sources(operands);
            body.with_face_sources(sources.iter().any(Option::is_some).then(|| Arc::new(sources)))
        }
        Err(_) => body,
    };
    Ok(Some(crate::record::with_history_or_log(body, history, operands)))
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

/// The union of bodies that do not touch, as one body of several lumps: every face and edge
/// kept from its operand, in order.
pub fn union_apart(bodies: &[Body]) -> Result<Option<Body>> {
    let mut lumps = Vec::new();
    let mut faces = Vec::new();
    let mut edge_origins = Vec::new();
    for (k, b) in bodies.iter().enumerate() {
        b.require_brep("union")?;
        let inner = b.inner()?;
        lumps.extend(inner.with("union apart", |solids| Ok(solids.iter().map(cadrum_handle).collect::<Vec<_>>()))?);
        faces.extend((0..inner.faces.len()).map(|face| FaceProv { origins: vec![FaceOrigin::Face { operand: k, face }], change: Change::Kept }));
        edge_origins.extend((0..inner.edges.len()).map(|edge| EdgeOrigin::Edge { operand: k, edge }));
    }
    if lumps.is_empty() {
        return Ok(None);
    }
    let inner = Inner::snapshot(lumps)?;
    let edges = inner.edge_faces.iter().zip(edge_origins).map(|(between, o)| EdgeProv { faces: between.clone(), origins: vec![o] }).collect();
    let history = History { operands: bodies.len(), faces, edges };
    let body = Body::from_inner(inner);
    let operands: Vec<&Body> = bodies.iter().collect();
    let sources = history.face_sources(&operands);
    let body = body.with_face_sources(sources.iter().any(Option::is_some).then(|| Arc::new(sources)));
    Ok(Some(crate::record::with_history_or_log(body, Ok(history), &operands)))
}

/// The union of bodies that may touch, with its history over all of them.
pub fn join_apart(bodies: &[Body]) -> Result<Option<Body>> {
    let operands: Vec<&Body> = bodies.iter().collect();
    match operands.len() {
        0 => Ok(None),
        1 => Ok(operands.first().map(|b| (*b).clone())),
        _ => combine(&operands, BoolOp::Union),
    }
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
    let mut out = Vec::new();
    for positive in [true, false] {
        out.extend(half_space(body, plane, positive)?.0);
    }
    if out.is_empty() {
        return Err(KernelError::Failed("the split left nothing".into()));
    }
    Ok(out)
}

/// The part of the body on `positive`'s side of `plane` (`None` when nothing is there), and the
/// tool it was cut with: a half space, whose one face the part's cut face comes from (operand 1
/// of the part's history).
pub fn half_space(body: &Body, plane: &Plane, positive: bool) -> Result<(Option<Body>, Body)> {
    body.require_brep("half space")?;
    let n = plane.normal() * if positive { 1.0 } else { -1.0 };
    if !(plane.origin.is_finite() && n.is_finite() && n.len() > 0.5) {
        return Err(KernelError::Invalid("split plane must be finite".into()));
    }
    let tool = Body::from_solids(vec![crate::guard("half space", || Ok(cadrum::Solid::half_space(dv(plane.origin), dv(n))))?])?;
    let part = combine(&[body, &tool], BoolOp::Intersect)?;
    Ok((part, tool))
}

/// The plane a split tool cuts along: the plane itself, or another body's planar face (its
/// plane extended). A curved face or sketch curves are not on this backend yet.
fn tool_plane(tool: &SplitTool) -> Result<Plane> {
    match tool {
        SplitTool::Plane(p) => Ok(*p),
        SplitTool::Face { body, face } => match crate::query::face_surface(body, *face) {
            Some(FaceSurface::Plane { normal, point }) => {
                let n = normal.vec();
                let x = crate::body::any_perpendicular(n).ok_or_else(|| KernelError::Invalid("split face normal".into()))?;
                Plane::new(point, x, n.cross(x)).ok_or_else(|| KernelError::Invalid("split face plane".into()))
            }
            Some(_) => Err(unsupported("splitting by a curved face")),
            None => Err(KernelError::Invalid("no such face to split by".into())),
        },
        SplitTool::Curves { .. } => Err(unsupported("splitting by sketch curves")),
    }
}

/// Split a body into the parts on either side of a tool: a plane, or another body's planar
/// face (its plane extended). Parts come with those on the tool's back first; a body the tool
/// misses comes back whole. Each part's history is over `[body]` (and the tool).
pub fn split_body(body: &Body, tool: &SplitTool) -> Result<Vec<Body>> {
    body.require_brep("split body")?;
    let plane = tool_plane(tool)?;
    let mut out = Vec::new();
    for positive in [false, true] {
        out.extend(half_space(body, &plane, positive)?.0);
    }
    if out.is_empty() {
        return Err(KernelError::Failed("the split left nothing".into()));
    }
    Ok(out)
}

/// The convex polyhedron bounded by the half-spaces (`n · x <= d`, `n` outward): a box
/// around them all, cut by each in turn. The half-spaces must bound a solid.
pub fn convex_polyhedron(hs: &[HalfSpace]) -> Result<Body> {
    if hs.len() < 4 || hs.len() > 1000 {
        return Err(KernelError::Invalid("a polyhedron needs 4…1000 half-spaces".into()));
    }
    if hs.iter().any(|h| !(h.n.is_finite() && h.d.is_finite() && h.d.abs() < 1e9) || h.n.normalized().is_none()) {
        return Err(KernelError::Invalid("half-spaces must be finite with a direction".into()));
    }
    let reach = hs.iter().map(|h| h.d.abs()).fold(1.0, f64::max) * 4.0;
    let mut body = crate::build::box_solid(Vec3::new(-reach, -reach, -reach), Vec3::new(reach, reach, reach))?;
    for h in hs {
        let n = h.n.normalized().ok_or_else(|| KernelError::Invalid("half-space normal".into()))?;
        let x = crate::body::any_perpendicular(n).ok_or_else(|| KernelError::Invalid("half-space normal".into()))?;
        let plane = Plane::new(n * h.d, x, n.cross(x)).ok_or_else(|| KernelError::Invalid("half-space plane".into()))?;
        body = half_space(&body, &plane, false)?.0.ok_or_else(|| KernelError::Invalid("the half-spaces bound no solid".into()))?;
    }
    let bbox = body.inner()?.bbox;
    if bbox.min.x <= -reach + 1e-6
        || bbox.min.y <= -reach + 1e-6
        || bbox.min.z <= -reach + 1e-6
        || bbox.max.x >= reach - 1e-6
        || bbox.max.y >= reach - 1e-6
        || bbox.max.z >= reach - 1e-6
    {
        return Err(KernelError::Invalid("the half-spaces do not bound a solid".into()));
    }
    Ok(body)
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

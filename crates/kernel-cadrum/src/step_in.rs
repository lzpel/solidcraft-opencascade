//! STEP import (ISO 10303-21 files, AP203/AP214/AP242 geometry).
//!
//! The Part 21 parser (`solidcraft_kernel_core::p21`) reads the exchange structure and this
//! module walks the product structure (products, shape representations, assemblies with their
//! placements, units, colours), as the truck backend's reader does. Each solid item's geometry
//! is read by OpenCASCADE: the item's entity closure and its representation context are written
//! out as a one-part file of their own (OpenCASCADE transfers only solids under a product
//! definition) and handed to cadrum's `read_step`. STEP files are untrusted: every count and
//! nesting depth is capped, every OpenCASCADE call runs under [`guard`], and what cannot be read
//! becomes a warning rather than a failure wherever the rest of the file is still usable.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::fmt::Write as _;
use std::sync::Arc;

use solidcraft_geom::Vec3;
use solidcraft_kernel_core::p21::{self, Entity, Exchange, Param};
use solidcraft_kernel_core::watch;

use crate::body::{Body, DisplayShare};
use crate::step::{ImportedBody, StepImport};
use crate::types::StepNode;
use crate::{FacePaint, FaceSource, KernelError, Paint, Result, guard};

/// Deepest assembly nesting followed.
const MAX_ASSEMBLY_DEPTH: usize = 32;
/// Most body instances in one import.
const MAX_BODIES: usize = 10_000;
/// Most product occurrences and mapped representations visited.
const MAX_VISITS: usize = 100_000;
/// Most planned solids converted in one step.
const MAX_WINDOW: usize = 64;
/// Most entities one solid's closure may hold.
const MAX_CLOSURE: usize = 5_000_000;
/// Coarsest vertex-on-curve tolerance taken from a file (mm).
const MAX_TOL: f64 = 1.0;
const KEEP_SMALL_BYTES: usize = 1 << 20;

type Mat = [[f64; 4]; 4];
type R<T> = std::result::Result<T, String>;

fn identity() -> Mat {
    [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
}

fn is_identity(m: &Mat) -> bool {
    let i = identity();
    m.iter().flatten().zip(i.iter().flatten()).all(|(a, b)| (a - b).abs() < 1e-12)
}

/// Column-major product `a * b` (apply `b`, then `a`).
fn mat_mul(a: &Mat, b: &Mat) -> Mat {
    let mut out = [[0.0; 4]; 4];
    for (c, col) in out.iter_mut().enumerate() {
        for (r, slot) in col.iter_mut().enumerate() {
            *slot = (0..4).map(|k| a[k][r] * b[c][k]).sum();
        }
    }
    out
}

/// The inverse of an affine matrix (identity when it has none).
fn inverse(m: &Mat) -> Mat {
    let a = [[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]];
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if !det.is_finite() || det.abs() < 1e-18 {
        return identity();
    }
    let inv = [
        [(a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det, (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det, (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det],
        [(a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det, (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det, (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det],
        [(a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det, (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det, (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det],
    ];
    let t = [m[3][0], m[3][1], m[3][2]];
    let mut out = identity();
    for c in 0..3 {
        for r in 0..3 {
            out[c][r] = inv[r][c];
        }
    }
    for r in 0..3 {
        out[3][r] = -(inv[r][0] * t[0] + inv[r][1] * t[1] + inv[r][2] * t[2]);
    }
    out
}

fn frame_matrix(o: Vec3, x: Vec3, y: Vec3, z: Vec3) -> Mat {
    [[x.x, x.y, x.z, 0.0], [y.x, y.y, y.z, 0.0], [z.x, z.y, z.z, 0.0], [o.x, o.y, o.z, 1.0]]
}

#[derive(Clone, Copy)]
struct Units {
    /// Millimetres per file length unit.
    len: f64,
    /// Radians per file angle unit.
    ang: f64,
    /// Vertex-on-curve tolerance (mm).
    tol: f64,
}

const DEFAULT_UNITS: Units = Units { len: 1.0, ang: 1.0, tol: 0.01 };

fn si_prefix(p: Option<&Param>) -> f64 {
    match p.and_then(Param::as_enum) {
        Some("EXA") => 1e18,
        Some("PETA") => 1e15,
        Some("TERA") => 1e12,
        Some("GIGA") => 1e9,
        Some("MEGA") => 1e6,
        Some("KILO") => 1e3,
        Some("HECTO") => 1e2,
        Some("DECA") => 1e1,
        Some("DECI") => 1e-1,
        Some("CENTI") => 1e-2,
        Some("MILLI") => 1e-3,
        Some("MICRO") => 1e-6,
        Some("NANO") => 1e-9,
        Some("PICO") => 1e-12,
        Some("FEMTO") => 1e-15,
        Some("ATTO") => 1e-18,
        _ => 1.0,
    }
}

/// A unit entity's kind and its size in millimetres or radians.
fn unit_factor(ex: &Exchange, id: u64, depth: usize) -> Option<(&'static str, f64)> {
    if depth > 4 {
        return None;
    }
    let e = ex.get(id)?;
    let kind = if e.has("LENGTH_UNIT") {
        "length"
    } else if e.has("PLANE_ANGLE_UNIT") {
        "angle"
    } else {
        "other"
    };
    if let Some(si) = e.record("SI_UNIT") {
        let base = match si.get(1).and_then(Param::as_enum) {
            Some("METRE") => 1000.0,
            Some("RADIAN") => 1.0,
            _ => return Some((kind, 1.0)),
        };
        return Some((kind, si_prefix(si.first()) * base));
    }
    if let Some(cb) = e.record("CONVERSION_BASED_UNIT") {
        let m = ex.get(cb.get(1)?.as_ref_id()?)?;
        let mp = m.params();
        let value = match mp.first()? {
            Param::Typed(_, v) => v.first().and_then(Param::as_f64)?,
            p => p.as_f64()?,
        };
        let (_, f) = unit_factor(ex, mp.get(1)?.as_ref_id()?, depth + 1)?;
        return Some((kind, value * f));
    }
    Some((kind, 1.0))
}

fn context_units(ex: &Exchange, ctx: Option<u64>, cache: &mut HashMap<u64, Units>) -> Units {
    let Some(id) = ctx else { return DEFAULT_UNITS };
    if let Some(u) = cache.get(&id) {
        return *u;
    }
    let mut u = DEFAULT_UNITS;
    let mut tol_file = None;
    if let Some(e) = ex.get(id) {
        if let Some(units) = e.record("GLOBAL_UNIT_ASSIGNED_CONTEXT").and_then(|r| r.first()).and_then(Param::as_list) {
            for uid in units.iter().filter_map(Param::as_ref_id) {
                match unit_factor(ex, uid, 0) {
                    Some(("length", f)) if f.is_finite() && f > 0.0 => u.len = f,
                    Some(("angle", f)) if f.is_finite() && f > 0.0 => u.ang = f,
                    _ => {}
                }
            }
        }
        if let Some(unc) = e.record("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT").and_then(|r| r.first()).and_then(Param::as_list) {
            for uid in unc.iter().filter_map(Param::as_ref_id) {
                if let Some(m) = ex.get(uid)
                    && let Some(v) = m.params().first().and_then(|p| match p {
                        Param::Typed(_, v) => v.first().and_then(Param::as_f64),
                        p => p.as_f64(),
                    })
                {
                    let f = m.params().get(1).and_then(Param::as_ref_id).and_then(|x| unit_factor(ex, x, 0)).map(|x| x.1);
                    tol_file = Some(v * f.unwrap_or(u.len));
                }
            }
        }
    }
    u.tol = tol_file.filter(|t| t.is_finite() && *t > 0.0).map(|t| (t * 10.0).max(1e-3)).unwrap_or(u.len * 1e-2).clamp(1e-4, MAX_TOL);
    cache.insert(id, u);
    u
}

/// A representation's items and its context.
fn representation(e: &Entity) -> Option<(Vec<u64>, Option<u64>)> {
    let p = if e.is_complex() {
        e.record("REPRESENTATION")
            .or_else(|| e.records.iter().find(|r| r.name.ends_with("REPRESENTATION") && r.params.len() >= 3).map(|r| r.params.as_slice()))?
    } else if e.name().ends_with("REPRESENTATION") {
        e.params()
    } else {
        return None;
    };
    let items = p.get(1)?.as_list()?.iter().filter_map(Param::as_ref_id).collect();
    Some((items, p.get(2).and_then(Param::as_ref_id)))
}

fn is_solid_item(e: &Entity) -> bool {
    matches!(e.name(), "MANIFOLD_SOLID_BREP" | "BREP_WITH_VOIDS" | "FACETED_BREP" | "SHELL_BASED_SURFACE_MODEL")
}

fn entity(ex: &Exchange, id: u64) -> R<&Entity> {
    ex.get(id).ok_or_else(|| format!("missing entity #{id}"))
}

fn coords(e: &Entity, what: &str) -> R<Vec3> {
    let c = e.params().get(1).and_then(Param::as_list).ok_or_else(|| format!("{what} without coordinates"))?;
    if c.is_empty() || c.len() > 3 {
        return Err(format!("{what} with {} coordinates", c.len()));
    }
    let g = |i: usize| -> R<f64> {
        match c.get(i) {
            None => Ok(0.0),
            Some(p) => p.as_f64().filter(|x| x.is_finite() && x.abs() < 1e12).ok_or_else(|| format!("{what}: coordinate out of range")),
        }
    };
    Ok(Vec3::new(g(0)?, g(1)?, g(2)?))
}

fn point(ex: &Exchange, id: u64, len: f64) -> R<Vec3> {
    let e = entity(ex, id)?;
    if e.name() != "CARTESIAN_POINT" {
        return Err(format!("#{id}: expected CARTESIAN_POINT, found {}", e.name()));
    }
    Ok(coords(e, "point")? * len)
}

fn direction(ex: &Exchange, id: u64) -> R<Vec3> {
    let e = entity(ex, id)?;
    if e.name() != "DIRECTION" {
        return Err(format!("#{id}: expected DIRECTION, found {}", e.name()));
    }
    coords(e, "direction")?.normalized().ok_or_else(|| format!("#{id}: zero direction"))
}

fn any_perpendicular(z: Vec3) -> Vec3 {
    let a = if z.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    (a - z * a.dot(z)).normalized().unwrap_or(Vec3::X)
}

/// AXIS2_PLACEMENT_3D (or 2D, as a frame in the XY plane) as a matrix.
fn frame(ex: &Exchange, id: u64, len: f64) -> R<Mat> {
    let e = entity(ex, id)?;
    let p = e.params();
    let o = point(ex, p.get(1).and_then(Param::as_ref_id).ok_or("placement without location")?, len)?;
    let (z, rf) = match e.name() {
        "AXIS2_PLACEMENT_3D" => {
            let z = match p.get(2).and_then(Param::as_ref_id) {
                Some(d) => direction(ex, d)?,
                None => Vec3::Z,
            };
            (z, p.get(3).and_then(Param::as_ref_id))
        }
        "AXIS2_PLACEMENT_2D" => (Vec3::Z, p.get(2).and_then(Param::as_ref_id)),
        n => return Err(format!("#{id}: expected AXIS2_PLACEMENT_3D, found {n}")),
    };
    let x = match rf {
        Some(d) => {
            let r = direction(ex, d)?;
            (r - z * r.dot(z)).normalized().unwrap_or_else(|| any_perpendicular(z))
        }
        None => any_perpendicular(z),
    };
    Ok(frame_matrix(o, x, z.cross(x), z))
}

/// The ids an entity refers to.
fn refs(e: &Entity, out: &mut Vec<u64>) {
    fn walk(p: &Param, out: &mut Vec<u64>) {
        match p {
            Param::Ref(r) => out.push(*r),
            Param::List(v) | Param::Typed(_, v) => v.iter().for_each(|x| walk(x, out)),
            _ => {}
        }
    }
    e.records.iter().flat_map(|r| &r.params).for_each(|p| walk(p, out));
}

/// A parameter written back as Part 21 text with its original ids.
fn write_param(p: &Param, o: &mut String) {
    match p {
        Param::Int(i) => {
            let _ = write!(o, "{i}");
        }
        Param::Real(x) => {
            let s = format!("{x:?}");
            let s = match s.split_once('e') {
                Some((m, e)) => format!("{}E{e}", if m.contains('.') { m.to_string() } else { format!("{m}.") }),
                None if s.contains('.') => s,
                None => format!("{s}."),
            };
            o.push_str(if x.is_finite() { &s } else { "0." });
        }
        Param::Str(s) => {
            o.push('\'');
            for c in s.chars() {
                match c {
                    '\'' => o.push_str("''"),
                    '\\' => o.push_str("\\\\"),
                    c if c.is_ascii() && !c.is_ascii_control() => o.push(c),
                    c if c.is_ascii_control() => {}
                    c => {
                        o.push_str("\\X2\\");
                        let mut buf = [0u16; 2];
                        for u in c.encode_utf16(&mut buf) {
                            let _ = write!(o, "{u:04X}");
                        }
                        o.push_str("\\X0\\");
                    }
                }
            }
            o.push('\'');
        }
        Param::Enum(e) => {
            let _ = write!(o, ".{e}.");
        }
        Param::Ref(r) => {
            let _ = write!(o, "#{r}");
        }
        Param::List(v) => {
            o.push('(');
            write_params(v, o);
            o.push(')');
        }
        Param::Typed(name, v) => {
            o.push_str(name);
            o.push('(');
            write_params(v, o);
            o.push(')');
        }
        Param::Binary(b) => {
            let _ = write!(o, "\"{b}\"");
        }
        Param::Null => o.push('$'),
        Param::Derived => o.push('*'),
    }
}

fn write_params(v: &[Param], o: &mut String) {
    for (i, p) in v.iter().enumerate() {
        if i > 0 {
            o.push(',');
        }
        write_param(p, o);
    }
}

fn write_entity(id: u64, e: &Entity, o: &mut String) {
    let _ = write!(o, "#{id}=");
    if e.is_complex() {
        o.push('(');
        for r in &e.records {
            o.push_str(&r.name);
            o.push('(');
            write_params(&r.params, o);
            o.push(')');
        }
        o.push(')');
    } else {
        o.push_str(e.name());
        o.push('(');
        write_params(e.params(), o);
        o.push(')');
    }
    o.push_str(";\n");
}

/// A solid item read where the file defines it.
struct Part {
    body: Body,
    /// The shells are closed.
    closed: bool,
    /// Faces the file had.
    file_faces: usize,
    /// The file face of each kernel face (empty when they could not be matched).
    face_origin: Vec<u64>,
}

/// The file's faces of `item`, in the order its shells list them.
fn item_faces(ex: &Exchange, item: u64) -> Vec<u64> {
    let mut shells = Vec::new();
    if let Some(e) = ex.get(item) {
        let p = e.params();
        match e.name() {
            "MANIFOLD_SOLID_BREP" | "FACETED_BREP" => shells.extend(p.get(1).and_then(Param::as_ref_id)),
            "BREP_WITH_VOIDS" => {
                shells.extend(p.get(1).and_then(Param::as_ref_id));
                shells.extend(p.get(2).and_then(Param::as_list).into_iter().flatten().filter_map(Param::as_ref_id));
            }
            "SHELL_BASED_SURFACE_MODEL" => shells.extend(p.get(1).and_then(Param::as_list).into_iter().flatten().filter_map(Param::as_ref_id)),
            _ => {}
        }
    }
    let mut faces = Vec::new();
    for s in shells {
        let Some(e) = ex.get(s) else { continue };
        let e =
            if e.name() == "ORIENTED_CLOSED_SHELL" { e.params().get(3).and_then(Param::as_ref_id).and_then(|i| ex.get(i)).unwrap_or(e) } else { e };
        faces.extend(e.params().get(1).and_then(Param::as_list).into_iter().flatten().filter_map(Param::as_ref_id));
    }
    faces
}

/// Read `item` through OpenCASCADE: the item's closure and its context as a one-part file.
fn read_item(ex: &Exchange, item: u64, ctx: Option<u64>, units: Units) -> R<Part> {
    let e = entity(ex, item)?;
    if e.name() == "SHELL_BASED_SURFACE_MODEL" {
        return Err("not supported yet: surface models (open shells) on the OpenCASCADE backend".into());
    }
    if !is_solid_item(e) {
        return Err(format!("#{item} is not a solid"));
    }
    let mut keep: HashSet<u64> = HashSet::new();
    let mut stack: Vec<u64> = vec![item];
    stack.extend(ctx);
    let mut out = Vec::new();
    while let Some(id) = stack.pop() {
        if !keep.insert(id) {
            continue;
        }
        if keep.len() > MAX_CLOSURE {
            return Err("solid too large".into());
        }
        if let Some(e) = ex.get(id) {
            out.clear();
            refs(e, &mut out);
            stack.extend(out.iter().copied().filter(|r| !keep.contains(r)));
        }
    }
    let mut ids: Vec<u64> = keep.iter().copied().collect();
    ids.sort_unstable();
    let next = ids.last().copied().unwrap_or(0) + 1;
    let mut text = String::with_capacity(keep.len() * 64);
    text.push_str("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('SolidCraft import'),'2;1');\nFILE_NAME('','',(''),(''),'','','');\n");
    let schema = ex.schemas.first().map(String::as_str).unwrap_or("AUTOMOTIVE_DESIGN");
    let _ = writeln!(text, "FILE_SCHEMA(('{}'));", schema.replace('\'', "''"));
    text.push_str("ENDSEC;\nDATA;\n");
    for id in &ids {
        if let Some(e) = ex.get(*id) {
            write_entity(*id, e, &mut text);
        }
    }
    let ctx_id = match ctx {
        Some(c) => c,
        None => {
            let _ = writeln!(text, "#{}=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));", next + 9);
            let _ = writeln!(
                text,
                "#{}=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#{}))REPRESENTATION_CONTEXT('',''));",
                next + 10,
                next + 9
            );
            next + 10
        }
    };
    let _ = writeln!(text, "#{next}=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{item}),#{ctx_id});");
    let _ = writeln!(text, "#{}=PRODUCT('P','P','',(#{}));", next + 1, next + 2);
    let _ = writeln!(text, "#{}=PRODUCT_CONTEXT('',#{},'mechanical');", next + 2, next + 3);
    let _ = writeln!(text, "#{}=APPLICATION_CONTEXT('automotive design');", next + 3);
    let _ = writeln!(text, "#{}=PRODUCT_DEFINITION_FORMATION('','',#{});", next + 4, next + 1);
    let _ = writeln!(text, "#{}=PRODUCT_DEFINITION('design','',#{},#{});", next + 5, next + 4, next + 6);
    let _ = writeln!(text, "#{}=PRODUCT_DEFINITION_CONTEXT('part definition',#{},'design');", next + 6, next + 3);
    let _ = writeln!(text, "#{}=PRODUCT_DEFINITION_SHAPE('','',#{});", next + 7, next + 5);
    let _ = writeln!(text, "#{}=SHAPE_DEFINITION_REPRESENTATION(#{},#{});", next + 8, next + 7, next);
    text.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    let solids = guard("STEP read", || cadrum::Solid::read_step(&mut text.as_bytes()).map_err(|e| KernelError::Failed(e.to_string())))
        .map_err(|e| e.to_string().trim_start_matches("the operation failed: ").to_string())?;
    if solids.is_empty() {
        return Err("no solid could be read from it".into());
    }
    let mut body = Body::from_solids(solids).map_err(|e| e.to_string())?;
    // Without a context OpenCASCADE read the numbers as millimetres: scale them.
    if ctx.is_none() && (units.len - 1.0).abs() > 1e-12 {
        let s = units.len;
        body = crate::transform_matrix(&body, [[s, 0.0, 0.0, 0.0], [0.0, s, 0.0, 0.0], [0.0, 0.0, s, 0.0], [0.0, 0.0, 0.0, 1.0]])
            .map_err(|e| e.to_string())?;
    }
    body.boundary_tol = Some(units.tol);
    let faces = item_faces(ex, item);
    let file_faces = faces.len();
    let face_origin = if faces.len() == body.face_count() { faces } else { Vec::new() };
    Ok(Part { body, closed: true, file_faces, face_origin })
}

/// A solid the walk found, converted later.
struct Planned {
    item: u64,
    units: Units,
    ctx: Option<u64>,
    batch: usize,
}

/// Where the solids of a batch go: their placement and names.
struct Batch {
    m: Mat,
    path: Vec<String>,
    product: String,
    instance: Option<String>,
}

struct Reader {
    ex: Arc<Exchange>,
    units: HashMap<u64, Units>,
    warnings: BTreeMap<String, usize>,
    reps: HashMap<u64, (Vec<u64>, Option<u64>)>,
    rep_links: HashMap<u64, Vec<u64>>,
    pd_reps: HashMap<u64, Vec<u64>>,
    children: HashMap<u64, Vec<u64>>,
    placements: HashMap<u64, u64>,
    colors: HashMap<u64, [f32; 3]>,
    opacity: HashMap<u64, f32>,
    unreadable: HashMap<u64, String>,
    uses: HashMap<u64, usize>,
    oriented: HashMap<u64, Arc<Part>>,
    shown: HashMap<u64, (DisplayShare, Mat)>,
    used_items: HashSet<u64>,
    node_items: HashSet<u64>,
    visits: usize,
    bodies: Vec<ImportedBody>,
    plan: Vec<Planned>,
    batches: Vec<Batch>,
    converted: usize,
    made: Vec<Option<usize>>,
    watch: Option<Arc<dyn watch::Watch>>,
    solid_items: usize,
    cancelled: bool,
}

impl Reader {
    fn new(ex: Arc<Exchange>) -> Self {
        Self {
            ex,
            units: HashMap::new(),
            warnings: BTreeMap::new(),
            reps: HashMap::new(),
            rep_links: HashMap::new(),
            pd_reps: HashMap::new(),
            children: HashMap::new(),
            placements: HashMap::new(),
            colors: HashMap::new(),
            opacity: HashMap::new(),
            unreadable: HashMap::new(),
            uses: HashMap::new(),
            oriented: HashMap::new(),
            shown: HashMap::new(),
            used_items: HashSet::new(),
            node_items: HashSet::new(),
            visits: 0,
            bodies: Vec::new(),
            plan: Vec::new(),
            batches: Vec::new(),
            converted: 0,
            made: Vec::new(),
            watch: None,
            solid_items: 0,
            cancelled: false,
        }
    }

    fn stop(&mut self) -> bool {
        if self.cancelled {
            return true;
        }
        if let Some(w) = &self.watch {
            if w.cancelled() {
                self.cancelled = true;
                return true;
            }
            w.progress("converting", self.converted.min(self.solid_items), self.solid_items);
        }
        false
    }

    fn warn(&mut self, msg: impl Into<String>) {
        *self.warnings.entry(msg.into()).or_default() += 1;
    }

    fn index(&mut self) {
        let ex = self.ex.clone();
        let ex = &*ex;
        let mut pds: HashMap<u64, u64> = HashMap::new();
        let mut sdrs: Vec<(u64, u64)> = Vec::new();
        let mut cdsrs: Vec<(u64, u64)> = Vec::new();
        let mut nauos: Vec<(u64, u64, u64)> = Vec::new();
        let mut ids: Vec<&u64> = ex.entities.keys().collect();
        ids.sort();
        for &id in ids {
            let Some(e) = ex.get(id) else { continue };
            if let Some(r) = representation(e) {
                self.reps.insert(id, r);
            }
            let p = e.params();
            let rf = |i: usize| p.get(i).and_then(Param::as_ref_id);
            match e.name() {
                "PRODUCT_DEFINITION_SHAPE" | "PROPERTY_DEFINITION" => {
                    if let Some(d) = rf(2) {
                        pds.insert(id, d);
                    }
                }
                "SHAPE_DEFINITION_REPRESENTATION" => {
                    if let (Some(d), Some(r)) = (rf(0), rf(1)) {
                        sdrs.push((d, r));
                    }
                }
                "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION" => {
                    if let (Some(r), Some(d)) = (rf(0), rf(1)) {
                        cdsrs.push((r, d));
                    }
                }
                "NEXT_ASSEMBLY_USAGE_OCCURRENCE" | "ASSEMBLY_COMPONENT_USAGE" | "SPECIFIED_HIGHER_USAGE_OCCURRENCE" => {
                    if let (Some(a), Some(b)) = (rf(3), rf(4)) {
                        nauos.push((id, a, b));
                    }
                }
                "SHAPE_REPRESENTATION_RELATIONSHIP" | "REPRESENTATION_RELATIONSHIP" if !e.is_complex() => {
                    if let (Some(a), Some(b)) = (rf(2), rf(3)) {
                        self.rep_links.entry(a).or_default().push(b);
                        self.rep_links.entry(b).or_default().push(a);
                    }
                }
                "STYLED_ITEM" | "OVER_RIDING_STYLED_ITEM" => {
                    if let Some(item) = rf(2)
                        && let Some(c) = self.find_colour(p.get(1))
                    {
                        self.colors.entry(item).or_insert(c);
                        if let Some(t) = self.find_transparency(p.get(1)) {
                            self.opacity.entry(item).or_insert((1.0 - t).clamp(0.0, 1.0));
                        }
                    }
                }
                _ => {}
            }
        }
        for (d, r) in sdrs {
            if let Some(&pd) = pds.get(&d) {
                self.pd_reps.entry(pd).or_default().push(r);
            }
        }
        for (r, d) in cdsrs {
            if let Some(&nauo) = pds.get(&d) {
                self.placements.insert(nauo, r);
            }
        }
        for (id, parent, child) in nauos {
            if parent != child {
                self.children.entry(parent).or_default().push(id);
            }
        }
    }

    fn style_walk(&self, styles: Option<&Param>, mut hit: impl FnMut(&Entity) -> Option<bool>) -> bool {
        let Some(list) = styles.and_then(Param::as_list) else { return false };
        let mut queue: Vec<u64> = list.iter().filter_map(Param::as_ref_id).collect();
        let mut seen = HashSet::new();
        while let Some(id) = queue.pop() {
            if seen.len() > 64 || !seen.insert(id) {
                continue;
            }
            let Some(e) = self.ex.get(id) else { continue };
            match hit(e) {
                Some(true) => return true,
                Some(false) => continue,
                None => {}
            }
            let mut out = Vec::new();
            refs(e, &mut out);
            queue.extend(out);
        }
        false
    }

    fn find_transparency(&self, styles: Option<&Param>) -> Option<f32> {
        let mut found = None;
        self.style_walk(styles, |e| {
            if e.name() == "SURFACE_STYLE_TRANSPARENT" {
                found = e.params().first().and_then(Param::as_f64).filter(|x| x.is_finite()).map(|x| x.clamp(0.0, 1.0) as f32);
                return Some(found.is_some());
            }
            if e.name() == "COLOUR_RGB" {
                return Some(false);
            }
            None
        });
        found
    }

    fn find_colour(&self, styles: Option<&Param>) -> Option<[f32; 3]> {
        let mut found = None;
        self.style_walk(styles, |e| {
            if e.name() == "COLOUR_RGB" {
                let p = e.params();
                let c = |i: usize| p.get(i).and_then(Param::as_f64).map(|x| x.clamp(0.0, 1.0) as f32);
                found = Some([c(1)?, c(2)?, c(3)?]);
                return Some(true);
            }
            if e.name() == "DRAUGHTING_PRE_DEFINED_COLOUR" {
                found = Some(match e.params().first().and_then(Param::as_str).unwrap_or("") {
                    "red" => [1.0, 0.0, 0.0],
                    "green" => [0.0, 1.0, 0.0],
                    "blue" => [0.0, 0.0, 1.0],
                    "yellow" => [1.0, 1.0, 0.0],
                    "magenta" => [1.0, 0.0, 1.0],
                    "cyan" => [0.0, 1.0, 1.0],
                    "black" => [0.0, 0.0, 0.0],
                    _ => [1.0, 1.0, 1.0],
                });
                return Some(true);
            }
            None
        });
        found
    }

    fn units_of(&mut self, rep: u64) -> Units {
        let ctx = self.reps.get(&rep).and_then(|r| r.1);
        context_units(&self.ex, ctx, &mut self.units)
    }

    fn placement_matrix(&self, item: u64, u: Units) -> R<Mat> {
        let ex = &*self.ex;
        let e = entity(ex, item)?;
        if matches!(e.name(), "CARTESIAN_TRANSFORMATION_OPERATOR_3D" | "CARTESIAN_TRANSFORMATION_OPERATOR_3D_NON_UNIFORM") {
            let p = e.params();
            let o = point(ex, p.get(4).and_then(Param::as_ref_id).ok_or("transformation without origin")?, u.len)?;
            let dir = |i: usize, fallback: Vec3| -> R<Vec3> {
                match p.get(i) {
                    None | Some(Param::Null) => Ok(fallback),
                    Some(Param::Ref(id)) => direction(ex, *id),
                    _ => Err("invalid transformation direction".into()),
                }
            };
            let scale = |i: usize, fallback: f64| -> R<f64> {
                match p.get(i) {
                    None | Some(Param::Null) => Ok(fallback),
                    Some(v) => v.as_f64().filter(|x| x.is_finite() && *x != 0.0).ok_or_else(|| "invalid transformation scale".into()),
                }
            };
            let sx = scale(5, 1.0)?;
            let (sy, sz) = (scale(7, sx)?, scale(8, sx)?);
            let m = frame_matrix(o, dir(2, Vec3::X)? * sx, dir(3, Vec3::Y)? * sy, dir(6, Vec3::Z)? * sz);
            if !m.iter().flatten().all(|x| x.is_finite()) {
                return Err("invalid transformation scale".into());
            }
            return Ok(m);
        }
        frame(ex, item, u.len)
    }

    fn rep_closure(&self, roots: &[u64]) -> Vec<u64> {
        let mut seen: Vec<u64> = Vec::new();
        let mut stack: Vec<u64> = roots.to_vec();
        while let Some(r) = stack.pop() {
            if seen.contains(&r) || seen.len() > 1024 {
                continue;
            }
            seen.push(r);
            if let Some(l) = self.rep_links.get(&r) {
                stack.extend(l.iter().copied());
            }
        }
        seen
    }

    fn label(&self, item: u64) -> String {
        let name = self.ex.get(item).and_then(|e| e.params().first()).and_then(Param::as_str).unwrap_or("").trim().to_string();
        if name.is_empty() { format!("solid #{item}") } else { format!("solid '{name}'") }
    }

    fn bodies_so_far(&self) -> usize {
        self.bodies.len() + self.plan.len() - self.converted
    }

    #[allow(clippy::too_many_arguments)]
    fn rep_bodies(&mut self, rep: u64, m: &Mat, path: &[String], product: &str, instance: Option<&str>, out: &mut Vec<usize>, depth: usize) {
        if depth > MAX_ASSEMBLY_DEPTH {
            self.warn("mapped items nested too deeply");
            return;
        }
        self.visits += 1;
        if self.visits > MAX_VISITS {
            self.warn(format!("more than {MAX_VISITS} component instances: the rest are left out"));
            return;
        }
        let u = self.units_of(rep);
        let ctx = self.reps.get(&rep).and_then(|r| r.1);
        let items: Vec<u64> = self.reps.get(&rep).map(|r| r.0.clone()).unwrap_or_default();
        let mut batch_id: Option<usize> = None;
        for item in items {
            if self.stop() {
                return;
            }
            let Some(e) = self.ex.get(item) else { continue };
            if is_solid_item(e) {
                let id = match batch_id {
                    Some(id) => id,
                    None => {
                        self.batches.push(Batch { m: *m, path: path.to_vec(), product: product.to_string(), instance: instance.map(str::to_string) });
                        let id = self.batches.len() - 1;
                        batch_id = Some(id);
                        id
                    }
                };
                self.used_items.insert(item);
                if !self.node_items.insert(item) {
                    continue;
                }
                if self.bodies_so_far() >= MAX_BODIES {
                    self.warn(format!("more than {MAX_BODIES} bodies: the rest are left out"));
                    return;
                }
                out.push(self.plan.len());
                self.plan.push(Planned { item, units: u, ctx, batch: id });
                *self.uses.entry(item).or_default() += 1;
            } else if e.name() == "MAPPED_ITEM" {
                let p = e.params();
                let (Some(map), Some(target)) = (p.get(1).and_then(Param::as_ref_id), p.get(2).and_then(Param::as_ref_id)) else { continue };
                let Some(me) = self.ex.get(map) else { continue };
                let (Some(origin), Some(mapped)) = (me.params().first().and_then(Param::as_ref_id), me.params().get(1).and_then(Param::as_ref_id))
                else {
                    continue;
                };
                let mu = self.units_of(mapped);
                match (self.placement_matrix(target, u), self.placement_matrix(origin, mu)) {
                    (Ok(t), Ok(o)) => {
                        let mm = mat_mul(m, &mat_mul(&t, &inverse(&o)));
                        for r in self.rep_closure(&[mapped]) {
                            self.rep_bodies(r, &mm, path, product, instance, out, depth + 1);
                        }
                    }
                    (Err(e), _) | (_, Err(e)) => self.warn(format!("mapped item #{item}: {e}")),
                }
            }
        }
    }

    /// Convert the next planned solid (or up to a window of them); false once all are.
    fn convert_next(&mut self, single: bool) -> bool {
        if self.converted >= self.plan.len() {
            return false;
        }
        if self.stop() {
            return false;
        }
        let first = self.converted;
        let last = if single { first + 1 } else { (first + MAX_WINDOW).min(self.plan.len()) };
        self.converted = last;
        let ex = self.ex.clone();
        for i in first..last {
            self.made.push(None);
            let Some(p) = self.plan.get(i) else { continue };
            let (item, units, ctx, batch) = (p.item, p.units, p.ctx, p.batch);
            let Some(e) = ex.get(item) else { continue };
            if let Some(msg) = self.unreadable.get(&item).cloned() {
                let label = self.label(item);
                self.warn(format!("{label}: not imported: {msg}"));
                continue;
            }
            let part = match self.oriented.get(&item) {
                Some(part) => part.clone(),
                None => match read_item(&ex, item, ctx, units) {
                    Ok(part) => {
                        let part = Arc::new(part);
                        if self.uses.get(&item).is_some_and(|n| *n > 1) {
                            self.oriented.insert(item, part.clone());
                        }
                        part
                    }
                    Err(msg) => {
                        let label = self.label(item);
                        self.warn(format!("{label}: not imported: {msg}"));
                        self.unreadable.insert(item, msg);
                        continue;
                    }
                },
            };
            let Some(batch) = self.batches.get(batch) else { continue };
            let (m, path, product, instance) = (batch.m, batch.path.clone(), batch.product.clone(), batch.instance.clone());
            let placed = if is_identity(&m) { Ok(part.body.clone()) } else { crate::transform_matrix(&part.body, m) };
            let body = match placed {
                Ok(b) => b,
                Err(err) => {
                    let label = self.label(item);
                    self.warn(format!("{label}: not placed: {err}"));
                    continue;
                }
            };
            let own = e.params().first().and_then(Param::as_str).unwrap_or("").trim().to_string();
            let name = if own.is_empty() { product.clone() } else { own };
            if let Some(slot) = self.made.get_mut(i) {
                *slot = Some(self.bodies.len());
            }
            let color = self.colors.get(&item).copied();
            let faces = part
                .face_origin
                .iter()
                .enumerate()
                .filter_map(|(i, f)| {
                    let c = self.colors.get(f)?;
                    Some(FacePaint { face: i, color: *c, opacity: self.opacity.get(f).copied().unwrap_or(1.0) })
                })
                .collect();
            let paint = Paint { opacity: self.opacity.get(&item).copied().unwrap_or(1.0), faces };
            let body = body.with_color(color).with_paint(Some(paint));
            let identities: Vec<Option<FaceSource>> =
                part.face_origin.iter().map(|face| Some(FaceSource::Imported { item, face: *face, payload_sha256: None, producer: None })).collect();
            let body = if identities.is_empty() { body } else { body.with_face_sources(Some(Arc::new(identities))) };
            let body = match self.shown.get(&item) {
                Some((shared, at)) => body.with_display_share(shared, mat_mul(&m, &inverse(at))),
                None if self.uses.get(&item).is_some_and(|n| *n > 1) => {
                    let shared = DisplayShare::new(body.clone());
                    self.shown.insert(item, (shared.clone(), m));
                    body.with_display_share(&shared, identity())
                }
                None => body,
            };
            self.bodies.push(ImportedBody {
                name,
                body,
                color,
                path,
                instance,
                closed: part.closed,
                file_faces: part.file_faces,
                source_item: item,
                face_origin: part.face_origin.clone(),
            });
        }
        true
    }

    fn remap_node(&self, n: &mut StepNode) {
        n.bodies = n.bodies.iter().filter_map(|i| self.made.get(*i).copied().flatten()).collect();
        for c in &mut n.children {
            self.remap_node(c);
        }
    }

    fn occurrence_name(&self, nauo: u64) -> Option<String> {
        let p = self.ex.get(nauo)?.params();
        let name = p.get(1).and_then(Param::as_str).filter(|s| !s.trim().is_empty()).or_else(|| p.first().and_then(Param::as_str))?;
        Some(name.trim().to_string()).filter(|n| !n.is_empty())
    }

    fn product_name(&self, pd: u64) -> String {
        let name = (|| {
            let pdf = self.ex.get(self.ex.get(pd)?.params().get(2)?.as_ref_id()?)?;
            let prod = self.ex.get(pdf.params().get(2)?.as_ref_id()?)?;
            let p = prod.params();
            let n = p.get(1).and_then(Param::as_str).filter(|s| !s.trim().is_empty()).or_else(|| p.first().and_then(Param::as_str))?;
            Some(n.trim().to_string())
        })();
        name.filter(|n| !n.is_empty()).unwrap_or_else(|| "Import".into())
    }

    fn occurrence_matrix(&mut self, nauo: u64, child_reps: &[u64]) -> Mat {
        let Some(&rel) = self.placements.get(&nauo) else { return identity() };
        let ex = self.ex.clone();
        let r = (|| -> R<Mat> {
            let e = ex.get(rel).ok_or("missing placement")?;
            let rr = e.record("REPRESENTATION_RELATIONSHIP").ok_or("placement without representations")?;
            let (r1, r2) =
                (rr.get(2).and_then(Param::as_ref_id).ok_or("bad placement")?, rr.get(3).and_then(Param::as_ref_id).ok_or("bad placement")?);
            let op = e
                .record("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
                .and_then(|x| x.first())
                .and_then(Param::as_ref_id)
                .ok_or("placement without transformation")?;
            let ope = ex.get(op).ok_or("missing transformation")?;
            let (u1, u2) = (self.units_of(r1), self.units_of(r2));
            if ope.name() == "ITEM_DEFINED_TRANSFORMATION" {
                let p = ope.params();
                let (i1, i2) = (
                    p.get(2).and_then(Param::as_ref_id).ok_or("bad transformation")?,
                    p.get(3).and_then(Param::as_ref_id).ok_or("bad transformation")?,
                );
                let m1 = self.placement_matrix(i1, u1)?;
                let m2 = self.placement_matrix(i2, u2)?;
                let child_is_1 = child_reps.contains(&r1) || !child_reps.contains(&r2);
                Ok(if child_is_1 { mat_mul(&m2, &inverse(&m1)) } else { mat_mul(&m1, &inverse(&m2)) })
            } else {
                self.placement_matrix(op, u2)
            }
        })();
        match r {
            Ok(m) => m,
            Err(e) => {
                self.warn(format!("component placement #{rel}: {e} (placed at the origin)"));
                identity()
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn node(&mut self, pd: u64, local: Mat, world: Mat, path: &[String], instance: Option<&str>, depth: usize, stack: &mut Vec<u64>) -> StepNode {
        let name = self.product_name(pd);
        let mut node = StepNode { name: name.clone(), transform: local, bodies: Vec::new(), children: Vec::new() };
        if self.stop() {
            return node;
        }
        if depth > MAX_ASSEMBLY_DEPTH || stack.contains(&pd) {
            self.warn("assembly nested too deeply or recursive: some components were left out");
            return node;
        }
        self.visits += 1;
        if self.visits > MAX_VISITS {
            self.warn(format!("more than {MAX_VISITS} component instances: the rest are left out"));
            return node;
        }
        stack.push(pd);
        let mut path = path.to_vec();
        path.push(name.clone());
        let own = self.pd_reps.get(&pd).cloned().unwrap_or_default();
        let reps = self.rep_closure(&own);
        let mut bodies = Vec::new();
        self.node_items.clear();
        for r in &reps {
            self.rep_bodies(*r, &world, &path, &name, instance, &mut bodies, 0);
        }
        node.bodies = bodies;
        for nauo in self.children.get(&pd).cloned().unwrap_or_default() {
            let Some(child) = self.ex.get(nauo).and_then(|e| e.params().get(4)).and_then(Param::as_ref_id) else { continue };
            let child_own = self.pd_reps.get(&child).cloned().unwrap_or_default();
            let child_reps = self.rep_closure(&child_own);
            let m = self.occurrence_matrix(nauo, &child_reps);
            let w = mat_mul(&world, &m);
            let instance = self.occurrence_name(nauo);
            let c = self.node(child, m, w, &path, instance.as_deref(), depth + 1, stack);
            node.children.push(c);
        }
        stack.pop();
        node
    }
}

struct Recent {
    key: (u64, usize),
    import: std::sync::Weak<StepImport>,
    _small: Option<Arc<StepImport>>,
}

static RECENT: std::sync::Mutex<Vec<Recent>> = std::sync::Mutex::new(Vec::new());

/// The cache key of a STEP text: its hash and length.
fn text_key(text: &str) -> (u64, usize) {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    text.hash(&mut h);
    (h.finish(), text.len())
}

fn remember(key: (u64, usize), imp: &Arc<StepImport>, small: bool) {
    if let Ok(mut cache) = RECENT.lock() {
        cache.retain(|r| r.key != key && r.import.strong_count() > 0);
        cache.push(Recent { key, import: Arc::downgrade(imp), _small: small.then(|| imp.clone()) });
        while cache.len() > 4 {
            cache.remove(0);
        }
    }
}

/// [`step_import`] through a small cache of recent results (keyed by the text).
pub fn step_import_shared(text: &str) -> Result<Arc<StepImport>> {
    let key = text_key(text);
    if let Ok(cache) = RECENT.lock()
        && let Some(imp) = cache.iter().filter(|r| r.key == key).find_map(|r| r.import.upgrade())
    {
        return Ok(imp);
    }
    let imp = Arc::new(step_import(text)?);
    remember(key, &imp, text.len() <= KEEP_SMALL_BYTES);
    Ok(imp)
}

/// Imports the cache still holds alive, and how many STEP bytes they came from.
pub fn step_cache_alive() -> (usize, usize) {
    RECENT.lock().map(|c| c.iter().filter(|r| r.import.strong_count() > 0).fold((0, 0), |(n, b), r| (n + 1, b + r.key.1))).unwrap_or((0, 0))
}

/// Read a STEP file: its solids (as bodies, placed by the assembly structure) and the tree.
pub fn step_import(text: &str) -> Result<StepImport> {
    let mut reading = StepReading::new(text)?;
    while reading.step() {}
    reading.finish()
}

/// A STEP file being read a piece at a time: [`StepReading::new`] parses the file and walks
/// its structure, each [`StepReading::step`] converts the next solids into bodies, and
/// [`StepReading::finish_shared`] gives the import, cached as [`step_import_shared`] would.
pub struct StepReading {
    rd: Reader,
    tree: Vec<StepNode>,
    key: (u64, usize),
    small: bool,
}

impl StepReading {
    /// Parse `text` and walk it; the solids are converted by [`Self::step`].
    pub fn new(text: &str) -> Result<StepReading> {
        let key = text_key(text);
        let watch = watch::current();
        let cancelled = || watch.as_ref().is_some_and(|w| w.cancelled());
        let ex = match p21::parse(text) {
            Ok(ex) => Arc::new(ex),
            Err(_) if cancelled() => return Err(watch::cancelled_error("STEP import")),
            Err(e) => return Err(KernelError::Invalid(e)),
        };
        let mut rd = Reader::new(ex.clone());
        rd.solid_items = ex.entities.values().filter(|e| is_solid_item(e)).count();
        rd.watch = watch.clone();
        rd.index();
        // Roots: product definitions that are nobody's component.
        let child_pds: HashSet<u64> =
            rd.children.values().flatten().filter_map(|n| ex.get(*n).and_then(|e| e.params().get(4)).and_then(Param::as_ref_id)).collect();
        let mut roots: Vec<u64> =
            ex.entities.iter().filter(|(id, e)| e.name() == "PRODUCT_DEFINITION" && !child_pds.contains(id)).map(|(id, _)| *id).collect();
        roots.sort();
        let mut tree = Vec::new();
        for pd in roots {
            tree.push(rd.node(pd, identity(), identity(), &[], None, 0, &mut Vec::new()));
        }
        // Solids outside any product structure (bare geometry files).
        let mut loose: Vec<u64> = rd
            .reps
            .iter()
            .flat_map(|(r, (items, _))| items.iter().map(move |i| (*r, *i)))
            .filter(|(_, i)| !rd.used_items.contains(i))
            .filter(|(_, i)| ex.get(*i).is_some_and(is_solid_item))
            .map(|(r, _)| r)
            .collect();
        loose.sort();
        loose.dedup();
        if !loose.is_empty() {
            let mut bodies = Vec::new();
            rd.node_items = rd.used_items.clone();
            rd.visits = 0;
            for r in loose {
                rd.rep_bodies(r, &identity(), &[], "Import", None, &mut bodies, 0);
            }
            tree.push(StepNode { name: "Import".into(), transform: identity(), bodies, children: Vec::new() });
        }
        Ok(StepReading { rd, tree, key, small: text.len() <= KEEP_SMALL_BYTES })
    }

    /// Convert the next solids; false once all are converted (or the watch cancelled).
    pub fn step(&mut self) -> bool {
        self.rd.convert_next(false)
    }

    /// Solids converted, of the solids to convert.
    pub fn progress(&self) -> (usize, usize) {
        (self.rd.converted, self.rd.plan.len())
    }

    /// The import, once [`Self::step`] has converted every solid.
    pub fn finish(mut self) -> Result<StepImport> {
        while self.step() {}
        let rd = &mut self.rd;
        if rd.cancelled || rd.watch.as_ref().is_some_and(|w| w.cancelled()) {
            return Err(watch::cancelled_error("STEP import"));
        }
        let mut tree = std::mem::take(&mut self.tree);
        for n in &mut tree {
            rd.remap_node(n);
        }
        tree.retain(|n| !n.bodies.is_empty() || !n.children.is_empty());
        let ex = rd.ex.clone();
        if let Some(first) = ex.duplicates.first() {
            rd.warn(format!("{} entity ids are defined twice (first #{first}); the first definitions were used", ex.duplicates.len()));
        }
        let warnings: Vec<String> = rd.warnings.iter().map(|(m, n)| if *n > 1 { format!("{m} (×{n})") } else { m.clone() }).collect();
        if rd.bodies.is_empty() {
            let why = warnings.first().cloned().unwrap_or_else(|| "the file has no solids".into());
            return Err(KernelError::Failed(format!("STEP import: no bodies ({why})")));
        }
        let bodies = std::mem::take(&mut rd.bodies);
        Ok(StepImport { bodies, tree, warnings, schema: ex.schemas.join(", "), system: ex.originating_system.clone() })
    }

    /// [`Self::finish`], and the import is cached for the next read of the same text.
    pub fn finish_shared(self) -> Result<Arc<StepImport>> {
        let (key, small) = (self.key, self.small);
        let imp = Arc::new(self.finish()?);
        remember(key, &imp, small);
        Ok(imp)
    }
}

/// Faces of a STEP file's solids that run the wrong way round: OpenCASCADE orients what it
/// reads, so the check is whether every solid read closed with a positive volume.
pub fn step_orientation_errors(text: &str) -> std::result::Result<Vec<String>, String> {
    let imp = step_import(text).map_err(|e| e.to_string())?;
    let mut out = Vec::new();
    for b in &imp.bodies {
        if !b.closed {
            out.push(format!("{}: the shell is not closed", b.name));
        }
    }
    Ok(out)
}

/// Check a STEP file's structure: every reference defined, every entity reachable, ids dense.
pub fn step_validate(text: &str) -> std::result::Result<usize, String> {
    let ex = p21::parse(text)?;
    if let Some(d) = ex.duplicates.first() {
        return Err(format!("{} duplicate entity ids (first #{d})", ex.duplicates.len()));
    }
    let mut ids: Vec<&u64> = ex.entities.keys().collect();
    ids.sort();
    let mut used = HashSet::new();
    for id in &ids {
        let mut out = Vec::new();
        if let Some(e) = ex.get(**id) {
            refs(e, &mut out);
        }
        if let Some(r) = out.iter().find(|r| ex.get(**r).is_none()) {
            return Err(format!("#{id} refers to undefined #{r}"));
        }
        used.extend(out);
    }
    const ROOTS: &[&str] = &[
        "SHAPE_DEFINITION_REPRESENTATION",
        "APPLICATION_PROTOCOL_DEFINITION",
        "PRODUCT_RELATED_PRODUCT_CATEGORY",
        "PRODUCT_CATEGORY_RELATIONSHIP",
        "SHAPE_REPRESENTATION_RELATIONSHIP",
        "REPRESENTATION_RELATIONSHIP",
        "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION",
        "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION",
        "DRAUGHTING_MODEL",
        "PROPERTY_DEFINITION_REPRESENTATION",
        "PRESENTATION_LAYER_ASSIGNMENT",
    ];
    for id in &ids {
        if used.contains(*id) {
            continue;
        }
        let Some(e) = ex.get(**id) else { continue };
        if !e.records.iter().any(|r| ROOTS.contains(&r.name.as_str()) || r.name.starts_with("APPLIED_")) {
            return Err(format!("#{id} ({}) is not referenced by anything", e.name()));
        }
    }
    let n = ex.entities.len();
    if ids.last().is_some_and(|m| **m != n as u64) || ids.first().is_some_and(|m| **m != 1) {
        return Err(format!("entity ids are not dense: {n} entities, #{}…#{}", ids.first().map_or(0, |x| **x), ids.last().map_or(0, |x| **x)));
    }
    Ok(n)
}

/// Read an IGES file: translated to STEP by the shared reader, then imported.
pub fn iges_import(text: &str) -> Result<StepImport> {
    let (step, notes) = solidcraft_kernel_core::iges_in::iges_to_step(text).map_err(KernelError::Invalid)?;
    let mut imp = step_import(&step)?;
    imp.warnings.splice(0..0, notes);
    Ok(imp)
}

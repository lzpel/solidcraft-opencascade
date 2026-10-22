//! STEP (ISO 10303-21) export: AP242 product structure, assemblies, names and colours.
//!
//! Each body's B-rep is written by OpenCASCADE's STEP writer (geometry and topology only),
//! read back with the Part 21 parser and copied into a file assembled here: one header and
//! context, a product per part with its shape representation, named solids, surface colours,
//! and assemblies as next-assembly-usage occurrences placed by item-defined transformations
//! (the structure the CAx-IF recommends and the reader follows). Identical geometric entities
//! (points, directions, placements, curves, surfaces) are written once and referenced; ids are
//! dense from `#1`. OpenCASCADE writes analytic surfaces as such, so nothing is rewritten.

use std::collections::HashMap;
use std::fmt::Write as _;

use solidcraft_kernel_core::p21::{self, Param};

use crate::body::Body;
use crate::step::{ExportBody, ExportProduct};
use crate::types::StepHeader;
use crate::{KernelError, Result, guard};

/// Entities whose identity is topological: never merged even when their text is equal.
const TOPOLOGY: &[&str] = &[
    "VERTEX_POINT",
    "EDGE_CURVE",
    "ORIENTED_EDGE",
    "EDGE_LOOP",
    "FACE_BOUND",
    "FACE_OUTER_BOUND",
    "FACE_SURFACE",
    "ADVANCED_FACE",
    "ORIENTED_FACE",
    "CLOSED_SHELL",
    "OPEN_SHELL",
    "ORIENTED_CLOSED_SHELL",
    "MANIFOLD_SOLID_BREP",
    "BREP_WITH_VOIDS",
];

/// Most entities copied for one solid.
const MAX_COPY: usize = 50_000_000;

fn real(x: f64) -> String {
    if !x.is_finite() {
        return "0.".into();
    }
    let s = format!("{x:?}");
    match s.split_once('e') {
        Some((m, e)) => {
            let m = if m.contains('.') { m.to_string() } else { format!("{m}.") };
            format!("{m}E{e}")
        }
        None if s.contains('.') => s,
        None => format!("{s}."),
    }
}

/// A STEP string literal (quotes doubled, non-ASCII as `\X2\` UTF-16 runs).
fn string(s: &str) -> String {
    let mut o = String::from("'");
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
    o
}

fn param(p: &Param, map: &HashMap<u64, u64>, o: &mut String) -> std::result::Result<(), String> {
    match p {
        Param::Int(i) => {
            let _ = write!(o, "{i}");
        }
        Param::Real(x) => o.push_str(&real(*x)),
        Param::Str(s) => o.push_str(&string(s)),
        Param::Enum(e) => {
            let _ = write!(o, ".{e}.");
        }
        Param::Ref(r) => {
            let n = map.get(r).ok_or_else(|| format!("dangling reference #{r}"))?;
            let _ = write!(o, "#{n}");
        }
        Param::List(v) => {
            o.push('(');
            params(v, map, o)?;
            o.push(')');
        }
        Param::Typed(name, v) => {
            o.push_str(name);
            o.push('(');
            params(v, map, o)?;
            o.push(')');
        }
        Param::Binary(b) => {
            let _ = write!(o, "\"{b}\"");
        }
        Param::Null => o.push('$'),
        Param::Derived => o.push('*'),
    }
    Ok(())
}

fn params(v: &[Param], map: &HashMap<u64, u64>, o: &mut String) -> std::result::Result<(), String> {
    for (i, p) in v.iter().enumerate() {
        if i > 0 {
            o.push(',');
        }
        param(p, map, o)?;
    }
    Ok(())
}

/// The DATA section under construction.
#[derive(Default)]
struct Out {
    lines: Vec<String>,
    /// Geometry text → id (shared entities).
    shared: HashMap<String, u64>,
    /// Faces of the solid being copied: exchange id → written id.
    faces_out: HashMap<u64, u64>,
}

impl Out {
    fn add(&mut self, text: impl Into<String>) -> u64 {
        self.lines.push(text.into());
        self.lines.len() as u64
    }
    /// Add, or reuse an identical earlier entity (geometry only).
    fn add_shared(&mut self, text: String) -> u64 {
        if let Some(id) = self.shared.get(&text) {
            return *id;
        }
        let id = self.add(text.clone());
        self.shared.insert(text, id);
        id
    }
    fn point(&mut self, p: [f64; 3]) -> u64 {
        self.add_shared(format!("CARTESIAN_POINT('',({},{},{}))", real(p[0]), real(p[1]), real(p[2])))
    }
    fn direction(&mut self, d: [f64; 3]) -> u64 {
        self.add_shared(format!("DIRECTION('',({},{},{}))", real(d[0]), real(d[1]), real(d[2])))
    }
    fn placement(&mut self, o: [f64; 3], z: [f64; 3], x: [f64; 3]) -> u64 {
        let (po, dz, dx) = (self.point(o), self.direction(z), self.direction(x));
        self.add_shared(format!("AXIS2_PLACEMENT_3D('',#{po},#{dz},#{dx})"))
    }

    /// Copy the solid `root` and everything it refers to from `ex`, named `name`.
    fn copy_solid(&mut self, ex: &p21::Exchange, root: u64, name: &str) -> std::result::Result<u64, String> {
        let mut map: HashMap<u64, u64> = HashMap::new();
        let mut stack: Vec<(u64, bool)> = vec![(root, false)];
        let mut budget = 0usize;
        while let Some((id, done)) = stack.pop() {
            budget += 1;
            if budget > MAX_COPY {
                return Err("solid too large".into());
            }
            if map.contains_key(&id) {
                continue;
            }
            let e = ex.get(id).ok_or_else(|| format!("missing #{id}"))?;
            if !done {
                stack.push((id, true));
                let mut refs = Vec::new();
                fn collect(p: &Param, out: &mut Vec<u64>) {
                    match p {
                        Param::Ref(r) => out.push(*r),
                        Param::List(v) | Param::Typed(_, v) => v.iter().for_each(|x| collect(x, out)),
                        _ => {}
                    }
                }
                e.records.iter().flat_map(|r| &r.params).for_each(|p| collect(p, &mut refs));
                stack.extend(refs.into_iter().filter(|r| !map.contains_key(r)).map(|r| (r, false)));
                continue;
            }
            let mut text = String::new();
            if e.is_complex() {
                text.push('(');
                for r in &e.records {
                    text.push_str(&r.name);
                    text.push('(');
                    params(&r.params, &map, &mut text)?;
                    text.push(')');
                }
                text.push(')');
            } else {
                let ename = if e.name() == "FACE_SURFACE" { "ADVANCED_FACE" } else { e.name() };
                text.push_str(ename);
                text.push('(');
                let mut ps = e.params().to_vec();
                if id == root
                    && let Some(first) = ps.first_mut()
                {
                    *first = Param::Str(name.to_string());
                }
                params(&ps, &map, &mut text)?;
                text.push(')');
            }
            let new = if TOPOLOGY.contains(&e.name()) { self.add(text) } else { self.add_shared(text) };
            map.insert(id, new);
            if matches!(e.name(), "FACE_SURFACE" | "ADVANCED_FACE") {
                self.faces_out.insert(id, new);
            }
        }
        map.get(&root).copied().ok_or_else(|| "solid not copied".into())
    }
}

type FaceLook = ([f32; 3], f32);

/// OpenCASCADE's STEP text for lump `lump` of `b`, parsed, with the file's face id of each
/// painted face.
fn lump_exchange(b: &Body, lump: usize) -> Result<(p21::Exchange, HashMap<u64, FaceLook>)> {
    let inner = b.inner()?;
    let text = inner.with("STEP export", |solids| {
        let solid = solids.get(lump).ok_or_else(|| KernelError::Failed("STEP export: lump".into()))?;
        guard("STEP export", || {
            let mut out = Vec::new();
            cadrum::Solid::write_step(std::iter::once(solid), &mut out).map_err(|e| KernelError::Failed(format!("STEP export: {e}")))?;
            String::from_utf8(out).map_err(|e| KernelError::Failed(format!("STEP export: {e}")))
        })
    })?;
    let ex = p21::parse(&text).map_err(|e| KernelError::Failed(format!("STEP export: {e}")))?;
    let mut paint = HashMap::new();
    if let Some(p) = b.paint()
        && !p.faces.is_empty()
    {
        // The shells list the faces in the traversal order the body's faces are numbered in.
        let mut shells: Vec<u64> = ex.entities.iter().filter(|(_, e)| matches!(e.name(), "CLOSED_SHELL" | "OPEN_SHELL")).map(|(i, _)| *i).collect();
        shells.sort();
        let faces: Vec<u64> = shells.iter().filter_map(|s| ex.get(*s)?.params().get(1)?.as_list()).flatten().filter_map(Param::as_ref_id).collect();
        let mine: Vec<usize> = inner.faces.iter().enumerate().filter(|(_, (l, _))| *l == lump).map(|(i, _)| i).collect();
        if faces.len() == mine.len() {
            for f in &p.faces {
                if let Some(k) = mine.iter().position(|i| *i == f.face)
                    && let Some(id) = faces.get(k)
                {
                    paint.insert(*id, (f.color, f.opacity));
                }
            }
        }
    }
    Ok((ex, paint))
}

fn rigid_frame(m: &[[f64; 4]; 4]) -> Result<([f64; 3], [f64; 3], [f64; 3])> {
    let x = [m[0][0], m[0][1], m[0][2]];
    let y = [m[1][0], m[1][1], m[1][2]];
    let z = [m[2][0], m[2][1], m[2][2]];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let cross = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]];
    let ok = (dot(x, x) - 1.0).abs() < 1e-6 && (dot(y, y) - 1.0).abs() < 1e-6 && dot(x, y).abs() < 1e-6 && dot(cross, z) > 0.999_999;
    if !ok || m.iter().flatten().any(|v| !v.is_finite()) {
        return Err(KernelError::Invalid("component placements must be rigid (rotation and translation) to export as STEP".into()));
    }
    Ok(([m[3][0], m[3][1], m[3][2]], z, x))
}

/// STEP text of the product tree rooted at `root`: each product's bodies as named solids with
/// their colours, each child placed by its transform.
pub fn step_export_products(products: &[ExportProduct], root: usize, header: &StepHeader) -> Result<String> {
    if products.is_empty() || root >= products.len() {
        return Err(KernelError::Invalid("nothing to export".into()));
    }
    if products.iter().all(|p| p.bodies.is_empty()) {
        return Err(KernelError::Invalid("nothing to export".into()));
    }
    for p in products {
        for b in &p.bodies {
            b.body.require_brep("STEP export")?;
        }
        if p.children.iter().any(|(c, _, _)| *c >= products.len()) {
            return Err(KernelError::Invalid("bad component reference".into()));
        }
    }
    // No product may contain itself.
    fn cyclic(products: &[ExportProduct], p: usize, stack: &mut Vec<usize>) -> bool {
        if stack.contains(&p) || stack.len() > 64 {
            return true;
        }
        stack.push(p);
        let bad = products.get(p).is_some_and(|x| x.children.iter().any(|(c, _, _)| cyclic(products, *c, stack)));
        stack.pop();
        bad
    }
    if cyclic(products, root, &mut Vec::new()) {
        return Err(KernelError::Invalid("the component tree is recursive".into()));
    }

    let mut o = Out::default();
    let app = o.add("APPLICATION_CONTEXT('managed model based 3d engineering')");
    o.add(format!("APPLICATION_PROTOCOL_DEFINITION('international standard','ap242_managed_model_based_3d_engineering',2014,#{app})"));
    let pctx = o.add(format!("PRODUCT_CONTEXT('',#{app},'mechanical')"));
    let pdctx = o.add(format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')"));
    let mm = o.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
    let rad = o.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
    let sr = o.add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
    let unc = o.add(format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#{mm},'distance_accuracy_value','confusion accuracy')"));
    let ctx = o.add(format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc}))GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm},#{rad},#{sr}))REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY'))"
    ));
    let origin = o.placement([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);

    let mut reach = vec![false; products.len()];
    let mut todo = vec![root];
    while let Some(p) = todo.pop() {
        if let Some(r) = reach.get_mut(p)
            && !*r
        {
            *r = true;
            if let Some(x) = products.get(p) {
                todo.extend(x.children.iter().map(|(c, _, _)| *c));
            }
        }
    }
    let mut pd = vec![0u64; products.len()];
    let mut shape_rep = vec![0u64; products.len()];
    let mut product_ids = Vec::new();
    let mut styled = Vec::new();
    let mut child_items: Vec<Vec<u64>> = vec![Vec::new(); products.len()];
    let mut placements: Vec<Vec<u64>> = vec![Vec::new(); products.len()];
    for (pi, p) in products.iter().enumerate() {
        if !reach.get(pi).copied().unwrap_or(false) {
            continue;
        }
        for (_, m, _) in &p.children {
            let (t, z, x) = rigid_frame(m)?;
            let (pt, dz, dx) = (o.point(t), o.direction(z), o.direction(x));
            let a = o.add(format!("AXIS2_PLACEMENT_3D('',#{pt},#{dz},#{dx})"));
            if let Some(v) = placements.get_mut(pi) {
                v.push(a);
            }
            if let Some(v) = child_items.get_mut(pi) {
                v.push(a);
            }
        }
    }
    for (pi, p) in products.iter().enumerate() {
        if !reach.get(pi).copied().unwrap_or(false) {
            continue;
        }
        let name = if p.name.trim().is_empty() { "Part".to_string() } else { p.name.clone() };
        let prod = o.add(format!("PRODUCT({},{},'',(#{pctx}))", string(&name), string(&name)));
        product_ids.push(prod);
        let pdf = o.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{prod})"));
        let d = o.add(format!("PRODUCT_DEFINITION('design','',#{pdf},#{pdctx})"));
        let pds = o.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{d})"));
        let mut items = vec![origin];
        items.extend(child_items.get(pi).cloned().unwrap_or_default());
        let item_list = items.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(",");
        let srep = o.add(format!("SHAPE_REPRESENTATION({},({item_list}),#{ctx})", string(&name)));
        o.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{srep})"));
        if let Some(slot) = pd.get_mut(pi) {
            *slot = d;
        }
        if let Some(slot) = shape_rep.get_mut(pi) {
            *slot = srep;
        }
        if !p.bodies.is_empty() {
            let mut solids = Vec::new();
            for b in &p.bodies {
                let lumps = b.body.inner()?.lumps.max(1);
                for lump in 0..lumps {
                    let (ex, face_paint) = lump_exchange(b.body, lump)?;
                    o.faces_out.clear();
                    let mut roots: Vec<u64> =
                        ex.entities.iter().filter(|(_, e)| matches!(e.name(), "MANIFOLD_SOLID_BREP" | "BREP_WITH_VOIDS")).map(|(i, _)| *i).collect();
                    roots.sort();
                    for r in roots {
                        let id = o.copy_solid(&ex, r, &b.name).map_err(|e| KernelError::Failed(format!("STEP export: {e}")))?;
                        solids.push(id);
                        let opacity = b.body.paint().map(|p| p.opacity).unwrap_or(1.0);
                        if let Some(c) = b.color {
                            styled.push(style(&mut o, id, c, opacity));
                        }
                    }
                    let mut painted: Vec<(u64, FaceLook)> =
                        face_paint.into_iter().filter_map(|(f, look)| Some((*o.faces_out.get(&f)?, look))).collect();
                    painted.sort_by_key(|(f, _)| *f);
                    for (f, (c, op)) in painted {
                        styled.push(style(&mut o, f, c, op));
                    }
                }
            }
            let list = solids.iter().chain(std::iter::once(&origin)).map(|i| format!("#{i}")).collect::<Vec<_>>().join(",");
            let absr = o.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION({},({list}),#{ctx})", string(&name)));
            o.add(format!("SHAPE_REPRESENTATION_RELATIONSHIP('','',#{srep},#{absr})"));
        }
    }
    if !product_ids.is_empty() {
        let list = product_ids.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(",");
        o.add(format!("PRODUCT_RELATED_PRODUCT_CATEGORY('part','',({list}))"));
    }
    for (pi, p) in products.iter().enumerate() {
        if !reach.get(pi).copied().unwrap_or(false) {
            continue;
        }
        for (k, (ci, _, oname)) in p.children.iter().enumerate() {
            let (Some(&parent_pd), Some(&child_pd), Some(&parent_sr), Some(&child_sr), Some(&target)) =
                (pd.get(pi), pd.get(*ci), shape_rep.get(pi), shape_rep.get(*ci), placements.get(pi).and_then(|v| v.get(k)))
            else {
                continue;
            };
            let nm = if oname.trim().is_empty() {
                format!("{}:{}", products.get(*ci).map(|c| c.name.as_str()).unwrap_or(""), k + 1)
            } else {
                oname.clone()
            };
            let nauo = o.add(format!("NEXT_ASSEMBLY_USAGE_OCCURRENCE({},{},'',#{parent_pd},#{child_pd},$)", string(&nm), string(&nm)));
            let pds = o.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{nauo})"));
            let idt = o.add(format!("ITEM_DEFINED_TRANSFORMATION('','',#{origin},#{target})"));
            let rr = o.add(format!(
                "(REPRESENTATION_RELATIONSHIP('','',#{child_sr},#{parent_sr})REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{idt})SHAPE_REPRESENTATION_RELATIONSHIP())"
            ));
            o.add(format!("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{rr},#{pds})"));
        }
    }
    if !styled.is_empty() {
        let list = styled.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(",");
        o.add(format!("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',({list}),#{ctx})"));
    }

    let mut s = String::new();
    s.push_str("ISO-10303-21;\nHEADER;\n");
    let description = if header.description.is_empty() { "SolidCraft model" } else { header.description.as_str() };
    let _ = writeln!(s, "FILE_DESCRIPTION(({}),'2;1');", string(description));
    let _ = writeln!(
        s,
        "FILE_NAME({},{},({}),({}),'SolidCraft','SolidCraft','');",
        string(&header.file_name),
        string(&header.time_stamp),
        string(&header.author),
        string(&header.organization)
    );
    s.push_str("FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }'));\nENDSEC;\nDATA;\n");
    for (i, l) in o.lines.iter().enumerate() {
        let _ = writeln!(s, "#{}={l};", i + 1);
    }
    s.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    Ok(s)
}

fn style(o: &mut Out, item: u64, c: [f32; 3], opacity: f32) -> u64 {
    let rgb = o.add_shared(format!("COLOUR_RGB('',{},{},{})", real(c[0] as f64), real(c[1] as f64), real(c[2] as f64)));
    let fasc = o.add_shared(format!("FILL_AREA_STYLE_COLOUR('',#{rgb})"));
    let fas = o.add_shared(format!("FILL_AREA_STYLE('',(#{fasc}))"));
    let ssfa = o.add_shared(format!("SURFACE_STYLE_FILL_AREA(#{fas})"));
    let sides = if opacity < 1.0 {
        let tr = o.add_shared(format!("SURFACE_STYLE_TRANSPARENT({})", real((1.0 - opacity as f64).clamp(0.0, 1.0))));
        format!("#{ssfa},#{tr}")
    } else {
        format!("#{ssfa}")
    };
    let sss = o.add_shared(format!("SURFACE_SIDE_STYLE('',({sides}))"));
    let ssu = o.add_shared(format!("SURFACE_STYLE_USAGE(.BOTH.,#{sss})"));
    let psa = o.add_shared(format!("PRESENTATION_STYLE_ASSIGNMENT((#{ssu}))"));
    o.add(format!("STYLED_ITEM('color',(#{psa}),#{item})"))
}

/// STEP text of the bodies as one part named after the file.
pub fn step_export_bodies(bodies: &[ExportBody], header: &StepHeader) -> Result<String> {
    let name = if header.file_name.is_empty() {
        "Part".to_string()
    } else {
        header.file_name.trim_end_matches(".step").trim_end_matches(".stp").to_string()
    };
    let p = ExportProduct {
        name,
        bodies: bodies.iter().map(|b| ExportBody { name: b.name.clone(), body: b.body, color: b.color }).collect(),
        children: Vec::new(),
    };
    step_export_products(&[p], 0, header)
}

/// STEP (ISO 10303-21, AP242) text for the given bodies as one part (`Body1`…, with the
/// bodies' colours).
pub fn step_export(bodies: &[&Body], system: &str) -> Result<String> {
    if bodies.is_empty() {
        return Err(KernelError::Invalid("nothing to export".into()));
    }
    let list: Vec<ExportBody> =
        bodies.iter().enumerate().map(|(i, b)| ExportBody { name: format!("Body{}", i + 1), body: b, color: b.color() }).collect();
    let header = StepHeader { organization: system.to_string(), ..Default::default() };
    step_export_bodies(&list, &header)
}

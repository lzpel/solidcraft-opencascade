//! Recording the exact history of a boolean's result over its operands. OpenCASCADE reports,
//! for each result face, the operand faces it was modified or generated from (`iter_history`
//! pairs of shape ids); a result face that still is an operand's face keeps that face's id. Ids
//! repeat for moved copies (lzpel/cadrum#317), so every id lookup is settled by geometry: the
//! candidate face whose surface is nearest the result face's centre. Edges are an operand's
//! when id and ends agree, else the meeting of their two faces.

use std::collections::HashMap;

use solidcraft_geom::Vec3;
use solidcraft_kernel_core::history::{Change, EdgeOrigin, EdgeProv, FaceOrigin, FaceProv, History, mark_shared_split};

use crate::Result;
use crate::body::{Body, Inner, v3};
use crate::build::meeting;

/// One operand's faces and edges by id, with the geometry that tells copies apart.
pub(crate) struct Operand {
    /// Operand face id → (face index, centre, area).
    faces: HashMap<u64, Vec<(usize, Vec3, f64)>>,
    /// Operand edge id → (edge index, start, end, the point halfway along).
    edges: HashMap<u64, Vec<(usize, Vec3, Vec3, Vec3)>>,
    pub(crate) counts: (usize, usize),
}

impl Operand {
    /// Read `inner`'s faces and edges from its locked `solids`.
    pub(crate) fn from_solids(inner: &Inner, solids: &[cadrum::Solid]) -> Operand {
        let mut faces: HashMap<u64, Vec<(usize, Vec3, f64)>> = HashMap::new();
        for i in 0..inner.faces.len() {
            if let Some(f) = inner.face(solids, i) {
                faces.entry(f.id()).or_default().push((i, v3(f.center()), f.area()));
            }
        }
        let mut edges: HashMap<u64, Vec<(usize, Vec3, Vec3, Vec3)>> = HashMap::new();
        let size = if inner.bbox.is_empty() { 1.0 } else { inner.bbox.diagonal().max(1e-9) };
        for e in 0..inner.edges.len() {
            if let Some(edge) = inner.edge(solids, e) {
                edges.entry(edge.id()).or_default().push((e, v3(edge.start_point()), v3(edge.end_point()), midpoint(edge, size)));
            }
        }
        Operand { faces, edges, counts: (inner.faces.len(), inner.edges.len()) }
    }

    /// The operand face with `id` nearest `at`, with how far its centre is from it.
    fn face(&self, id: u64, at: Vec3) -> Option<(usize, f64)> {
        self.faces.get(&id)?.iter().map(|(i, c, _)| (*i, (*c - at).len())).min_by(|a, b| a.1.total_cmp(&b.1))
    }

    /// The operand edge with these ends (either way round) and midpoint, within `tol`: the
    /// cells builder makes new edge shapes, so ids do not tell.
    fn edge(&self, a: Vec3, b: Vec3, mid: Vec3, tol: f64) -> Option<usize> {
        self.edges
            .values()
            .flatten()
            .find(|(_, s, e, m)| {
                (((*s - a).len() < tol && (*e - b).len() < tol) || ((*s - b).len() < tol && (*e - a).len() < tol)) && (*m - mid).len() < tol
            })
            .map(|(i, _, _, _)| *i)
    }
}

/// What the boolean said about its result: `[post, src]` pairs from the boolean itself, and
/// `[post, pre]` pairs from a clean-up that fused faces afterwards (`pre` in the boolean's post
/// ids), with the result's own snapshot.
pub(crate) struct Recorded {
    pub(crate) pairs: Vec<[u64; 2]>,
    pub(crate) cleaned: Vec<[u64; 2]>,
}

/// The history of the result `inner` over `operands` (in order), from what OpenCASCADE recorded.
pub(crate) fn history(inner: &Inner, operands: &[Operand], rec: &Recorded, size: f64) -> Result<History> {
    let tol = (size * 1e-6).max(1e-9);
    // src id → origins among the operands, settled by geometry at `at`.
    let origins_of = |src: u64, at: Vec3| -> Vec<FaceOrigin> {
        let mut best: Option<(usize, usize, f64)> = None;
        for (k, op) in operands.iter().enumerate() {
            if let Some((face, d)) = op.face(src, at)
                && best.is_none_or(|b| d < b.2)
            {
                best = Some((k, face, d));
            }
        }
        best.map(|(operand, face, _)| vec![FaceOrigin::Face { operand, face }]).unwrap_or_default()
    };
    let mut by_post: HashMap<u64, Vec<u64>> = HashMap::new();
    for [post, src] in &rec.pairs {
        by_post.entry(*post).or_default().push(*src);
    }
    let mut by_clean: HashMap<u64, Vec<u64>> = HashMap::new();
    for [post, pre] in &rec.cleaned {
        by_clean.entry(*post).or_default().push(*pre);
    }
    let (faces, edges) = inner.with("boolean history", |solids| {
        let mut faces = Vec::with_capacity(inner.faces.len());
        for i in 0..inner.faces.len() {
            let Some(face) = inner.face(solids, i) else {
                faces.push(FaceProv { origins: Vec::new(), change: Change::Generated });
                continue;
            };
            let (id, at, area) = (face.id(), v3(face.center()), face.area());
            // The boolean's post ids this face stands for (itself, unless a clean fused several).
            let pre: Vec<u64> = by_clean.get(&id).cloned().unwrap_or_else(|| vec![id]);
            let mut origins: Vec<FaceOrigin> = Vec::new();
            for p in &pre {
                // What the boolean made it from; a face the boolean left alone keeps its id.
                match by_post.get(p) {
                    Some(srcs) => {
                        for s in srcs {
                            origins.extend(origins_of(*s, at));
                        }
                    }
                    None => origins.extend(origins_of(*p, at)),
                }
            }
            origins.sort_unstable();
            origins.dedup();
            // The cells builder rebuilds every face, so "kept" is read off the geometry: one
            // origin at the same place with the same area.
            let same_region = |o: &FaceOrigin| match *o {
                FaceOrigin::Face { operand, face } => operands
                    .get(operand)
                    .map(|op| {
                        op.faces
                            .values()
                            .flatten()
                            .any(|(i, centre, a)| *i == face && (*centre - at).len() < tol && (a - area).abs() <= 1e-7 * a.abs().max(1.0))
                    })
                    .unwrap_or(false),
                _ => false,
            };
            let change = match origins.as_slice() {
                [] => Change::Generated,
                [one] if same_region(one) => Change::Kept,
                [_] => Change::Modified,
                _ => Change::Merged,
            };
            faces.push(FaceProv { origins, change });
        }
        mark_shared_split(&mut faces);
        let mut edges = Vec::with_capacity(inner.edges.len());
        for e in 0..inner.edges.len() {
            let between = inner.edge_faces.get(e).cloned().unwrap_or_default();
            let own = inner.edge(solids, e).and_then(|edge| {
                let (a, b, mid) = (v3(edge.start_point()), v3(edge.end_point()), midpoint(edge, size.max(1e-9)));
                operands.iter().enumerate().find_map(|(k, op)| op.edge(a, b, mid, tol).map(|i| EdgeOrigin::Edge { operand: k, edge: i }))
            });
            let origins = match own {
                Some(o) => vec![o],
                None => meeting(&between, &faces),
            };
            edges.push(EdgeProv { faces: between, origins });
        }
        Ok((faces, edges))
    })?;
    let h = History { operands: operands.len(), faces, edges };
    h.check_counts((inner.faces.len(), inner.edges.len()), &operands.iter().map(|o| o.counts).collect::<Vec<_>>())?;
    Ok(h)
}

/// The history of each lump of `whole` (one solid each) over `[whole]`: its faces and edges are
/// the whole's own, in the same order within the lump.
pub(crate) fn lump_histories(whole: &Inner) -> Vec<History> {
    let mut out = Vec::with_capacity(whole.lumps);
    for lump in 0..whole.lumps {
        let faces: Vec<usize> = whole.faces.iter().enumerate().filter(|(_, (l, _))| *l == lump).map(|(i, _)| i).collect();
        let local: HashMap<usize, usize> = faces.iter().enumerate().map(|(local, whole_i)| (*whole_i, local)).collect();
        let face_provs = faces.iter().map(|&i| FaceProv { origins: vec![FaceOrigin::Face { operand: 0, face: i }], change: Change::Kept }).collect();
        let edges = whole
            .edges
            .iter()
            .enumerate()
            .filter(|(_, (l, _, _))| *l == lump)
            .map(|(e, _)| EdgeProv {
                faces: whole.edge_faces.get(e).map(|fs| fs.iter().filter_map(|f| local.get(f).copied()).collect()).unwrap_or_default(),
                origins: vec![EdgeOrigin::Edge { operand: 0, edge: e }],
            })
            .collect();
        out.push(History { operands: 1, faces: face_provs, edges });
    }
    out
}

/// `body` as the result of its operands with `history`, or without one when it does not fit.
pub(crate) fn with_history_or_log(body: Body, history: Result<History>, operands: &[&Body]) -> Body {
    match history {
        Ok(h) => match body.clone().with_history(h, operands) {
            Ok(b) => b,
            Err(e) => {
                log::debug!("no history: {e}");
                body
            }
        },
        Err(e) => {
            log::debug!("no history: {e}");
            body
        }
    }
}

/// The point halfway along an edge's polyline.
fn midpoint(edge: &cadrum::Edge, size: f64) -> Vec3 {
    let tess = cadrum::Tessellation { deflection_linear: size * 1e-4, deflection_angular: 0.2, relative_linear: false };
    let pts: Vec<Vec3> = edge.approximation_segments(tess).into_iter().map(v3).collect();
    crate::tess::polyline_mid(&pts).map(|(m, _)| m).unwrap_or_else(|| (v3(edge.start_point()) + v3(edge.end_point())) * 0.5)
}

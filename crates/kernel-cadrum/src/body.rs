//! A body over cadrum solids: one or more lumps behind a mutex (cadrum's types are `Send` but
//! not `Sync`, and OpenCASCADE writes a shape's triangulation into the shape), with the face and
//! edge order fixed when the body is made. Faces and edges are addressed by their position in
//! that order everywhere above the kernel (`FaceInfo::index`, histories, face sources); the
//! OpenCASCADE shape ids behind them never leave this crate.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock, PoisonError};

use solidcraft_geom::{Aabb3, Mesh, Plane, Vec3};
use solidcraft_kernel_core::history::History;
use solidcraft_kernel_core::meshbody::TriMesh;

use crate::{CapSignature, FaceSource, KernelError, Paint, Result, guard};

pub(crate) fn dv(v: Vec3) -> cadrum::DVec3 {
    cadrum::DVec3::new(v.x, v.y, v.z)
}

pub(crate) fn v3(d: cadrum::DVec3) -> Vec3 {
    Vec3::new(d.x, d.y, d.z)
}

/// Most faces a body may have (a snapshot walks every face and edge once).
const MAX_FACES: usize = 1_000_000;

/// The solids of a body and what was read of them when the body was made.
///
/// Faces and edges are addressed by position (lump, place in the lump's traversal), not by
/// cadrum's id: OpenCASCADE makes a prism's top face by *moving* its bottom face, so both carry
/// one shape id (lzpel/cadrum#317), and the same goes for the moved edges.
pub(crate) struct Inner {
    solids: Mutex<Vec<cadrum::Solid>>,
    /// Each face in face order: its lump and its place in that lump's face traversal.
    pub(crate) faces: Vec<(usize, usize)>,
    /// Each edge in edge order: the lump, the face (in the lump's traversal) and the place in
    /// that face's edge traversal where it was first met.
    pub(crate) edges: Vec<(usize, usize, usize)>,
    /// The faces each edge bounds, in edge order.
    pub(crate) edge_faces: Vec<Vec<usize>>,
    /// The faces (in face order) each cadrum face id stands for.
    pub(crate) face_index: HashMap<u64, Vec<usize>>,
    pub(crate) bbox: Aabb3,
    pub(crate) lumps: usize,
    pub(crate) vertices: usize,
    pub(crate) volume: OnceLock<f64>,
    /// Meshes made of these solids, by chord tolerance and angle, newest last.
    pub(crate) meshes: Mutex<Vec<((u64, u64), Arc<Mesh>)>>,
}

/// Meshes an [`Inner`] keeps at most (the oldest goes first).
const MEMO_MESHES: usize = 4;

impl std::fmt::Debug for Inner {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Inner(lumps={}, faces={}, edges={})", self.lumps, self.faces.len(), self.edges.len())
    }
}

impl Inner {
    /// Read the solids' faces and edges once, in a fixed order.
    pub(crate) fn snapshot(solids: Vec<cadrum::Solid>) -> Result<Inner> {
        guard("read topology", || {
            let lumps = solids.len();
            let mut faces = Vec::new();
            let mut edges: Vec<(usize, usize, usize)> = Vec::new();
            let mut edge_faces: Vec<Vec<usize>> = Vec::new();
            let mut face_index: HashMap<u64, Vec<usize>> = HashMap::new();
            // An edge by its id and its ends: a moved copy of an edge keeps the id but not the ends.
            let mut edge_index: HashMap<(u64, [i64; 3], [i64; 3]), usize> = HashMap::new();
            let mut bbox = Aabb3::EMPTY;
            for solid in solids.iter() {
                let [lo, hi] = solid.bounding_box();
                bbox.add(v3(lo));
                bbox.add(v3(hi));
            }
            let span = if bbox.is_empty() { 0.0 } else { bbox.diagonal() };
            let quantum = (span * 1e-7).max(1e-9);
            let key = |p: Vec3| [p.x, p.y, p.z].map(|c| if c.is_finite() { (c / quantum).round() as i64 } else { i64::MAX });
            let mut ends: Vec<Vec3> = Vec::new();
            for (lump, solid) in solids.iter().enumerate() {
                for (pos, face) in solid.iter_face().enumerate() {
                    if faces.len() >= MAX_FACES {
                        return Err(KernelError::TooLarge(format!("more than {MAX_FACES} faces")));
                    }
                    let fi = faces.len();
                    faces.push((lump, pos));
                    face_index.entry(face.id()).or_default().push(fi);
                    for (epos, edge) in face.iter_edge().enumerate() {
                        let (a, b) = (v3(edge.start_point()), v3(edge.end_point()));
                        let (ka, kb) = (key(a), key(b));
                        let ekey = if ka <= kb { (edge.id(), ka, kb) } else { (edge.id(), kb, ka) };
                        let ei = match edge_index.get(&ekey) {
                            Some(&ei) => ei,
                            None => {
                                edge_index.insert(ekey, edges.len());
                                edges.push((lump, pos, epos));
                                edge_faces.push(Vec::new());
                                ends.push(a);
                                ends.push(b);
                                edges.len() - 1
                            }
                        };
                        if let Some(fs) = edge_faces.get_mut(ei)
                            && !fs.contains(&fi)
                        {
                            fs.push(fi);
                        }
                    }
                }
            }
            if faces.is_empty() {
                return Err(KernelError::Failed("empty result".into()));
            }
            let vertices = distinct_points(&ends, quantum);
            Ok(Inner {
                solids: Mutex::new(solids),
                faces,
                edges,
                edge_faces,
                face_index,
                bbox,
                lumps,
                vertices,
                volume: OnceLock::new(),
                meshes: Mutex::new(Vec::new()),
            })
        })
    }

    pub(crate) fn lock(&self) -> std::sync::MutexGuard<'_, Vec<cadrum::Solid>> {
        self.solids.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Run `f` over the solids, under [`guard`] and the lock.
    pub(crate) fn with<R>(&self, what: &str, f: impl FnOnce(&[cadrum::Solid]) -> Result<R>) -> Result<R> {
        guard(what, || {
            let solids = self.lock();
            f(&solids)
        })
    }

    /// Run `f` over the solids of several bodies (one slice per entry of `inners`, in order),
    /// locking each distinct body once, in a fixed order.
    pub(crate) fn with_many<R>(inners: &[&Arc<Inner>], what: &str, f: impl FnOnce(&[&[cadrum::Solid]]) -> Result<R>) -> Result<R> {
        guard(what, || {
            let mut distinct: Vec<&Arc<Inner>> = Vec::new();
            for i in inners {
                if !distinct.iter().any(|d| Arc::ptr_eq(d, i)) {
                    distinct.push(i);
                }
            }
            distinct.sort_by_key(|i| Arc::as_ptr(i) as usize);
            let guards: Vec<std::sync::MutexGuard<'_, Vec<cadrum::Solid>>> = distinct.iter().map(|i| i.lock()).collect();
            let slices: Vec<&[cadrum::Solid]> = inners
                .iter()
                .map(|i| distinct.iter().position(|d| Arc::ptr_eq(d, i)).and_then(|k| guards.get(k)).map(|g| g.as_slice()).unwrap_or(&[]))
                .collect();
            f(&slices)
        })
    }

    /// Face `index` of the locked solids.
    pub(crate) fn face<'s>(&self, solids: &'s [cadrum::Solid], index: usize) -> Option<&'s cadrum::Face> {
        let (lump, pos) = *self.faces.get(index)?;
        solids.get(lump)?.iter_face().nth(pos)
    }

    /// Edge `index` of the locked solids.
    pub(crate) fn edge<'s>(&self, solids: &'s [cadrum::Solid], index: usize) -> Option<&'s cadrum::Edge> {
        let (lump, pos, epos) = *self.edges.get(index)?;
        solids.get(lump)?.iter_face().nth(pos)?.iter_edge().nth(epos)
    }

    /// The face (in face order) a mesh triangle with cadrum face id `id` and centroid `at`
    /// belongs to: the one of the id's faces nearest the point.
    pub(crate) fn face_of(&self, solids: &[cadrum::Solid], id: u64, at: Vec3) -> Option<usize> {
        let candidates = self.face_index.get(&id)?;
        match candidates.as_slice() {
            [one] => Some(*one),
            many => many
                .iter()
                .filter_map(|&fi| {
                    let face = self.face(solids, fi)?;
                    let (q, _) = face.project(dv(at));
                    let d = (v3(q) - at).len();
                    d.is_finite().then_some((fi, d))
                })
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(fi, _)| fi),
        }
    }

    /// A mesh made before at exactly this tolerance and angle.
    pub(crate) fn memo(&self, key: (u64, u64)) -> Option<Arc<Mesh>> {
        self.meshes.lock().unwrap_or_else(PoisonError::into_inner).iter().find(|(k, _)| *k == key).map(|(_, m)| m.clone())
    }

    pub(crate) fn remember(&self, key: (u64, u64), mesh: Arc<Mesh>) {
        let mut made = self.meshes.lock().unwrap_or_else(PoisonError::into_inner);
        made.retain(|(k, _)| *k != key);
        if made.len() >= MEMO_MESHES {
            made.remove(0);
        }
        made.push((key, mesh));
    }
}

/// How many distinct points there are among `points`, within `tol`.
fn distinct_points(points: &[Vec3], tol: f64) -> usize {
    let mut seen: HashMap<[i64; 3], ()> = HashMap::new();
    for p in points {
        if !p.is_finite() {
            continue;
        }
        let key = [p.x, p.y, p.z].map(|c| (c / tol).round() as i64);
        seen.insert(key, ());
    }
    seen.len()
}

/// What a body is made of.
#[derive(Clone, Debug)]
pub(crate) enum Shape {
    Solid(Arc<Inner>),
    Mesh(Arc<TriMesh>),
}

/// The display mesh of a body that several placed copies show (see `Body::display_of`).
#[derive(Debug)]
pub(crate) struct SharedDisplay {
    pub(crate) original: Body,
}

/// The display one body shares with its placed copies (see [`Body::display_share`]), so a
/// saved design can store the part once and give its copies the same display again.
#[derive(Clone, Debug)]
pub struct DisplayShare(Arc<SharedDisplay>);

impl DisplayShare {
    pub fn new(original: Body) -> Self {
        DisplayShare(Arc::new(SharedDisplay { original: Body { display_of: None, ..original } }))
    }
    /// The body whose display the copies show.
    pub fn original(&self) -> &Body {
        &self.0.original
    }
    /// Identity: copies that show the same display have the same id.
    pub fn id(&self) -> usize {
        Arc::as_ptr(&self.0) as usize
    }
}

/// A solid body (one or more closed shells), or a mesh body (triangles only, like an imported
/// 3MF or STL: it renders, measures and exports, but B-rep features cannot use it). Cheap to
/// clone; never mutated in place.
#[derive(Clone, Debug)]
pub struct Body {
    pub(crate) shape: Shape,
    /// Bounding-box diagonal; 0 means "not measured". Every kernel tolerance is a fraction of it.
    pub(crate) size: f64,
    /// The accuracy a body read from a file declares for its boundaries (mm).
    pub(crate) boundary_tol: Option<f64>,
    /// Display colour (linear RGB 0..1) from an imported file; operations drop it.
    pub(crate) color: Option<[f32; 3]>,
    pub(crate) paint: Option<Arc<Paint>>,
    pub(crate) splits: Option<Arc<Vec<Vec<Vec3>>>>,
    pub(crate) face_sources: Option<Arc<Vec<Option<FaceSource>>>>,
    pub(crate) regularization_protected: bool,
    pub(crate) history: Option<Arc<History>>,
    pub(crate) display_of: Option<Arc<(Arc<SharedDisplay>, [[f64; 4]; 4])>>,
}

type FaceSources = Arc<Vec<Option<FaceSource>>>;

impl Body {
    /// A body of these lumps, fresh: no colour, paint, sources or history.
    pub(crate) fn from_solids(solids: Vec<cadrum::Solid>) -> Result<Body> {
        let inner = Inner::snapshot(solids)?;
        let size = if inner.bbox.is_empty() { 0.0 } else { inner.bbox.diagonal() };
        Ok(Body {
            shape: Shape::Solid(Arc::new(inner)),
            size: if size.is_finite() && size > 0.0 { size } else { 0.0 },
            boundary_tol: None,
            color: None,
            paint: None,
            splits: None,
            face_sources: None,
            regularization_protected: false,
            history: None,
            display_of: None,
        })
    }

    /// A body of an already read snapshot.
    pub(crate) fn from_inner(inner: Inner) -> Body {
        let size = if inner.bbox.is_empty() { 0.0 } else { inner.bbox.diagonal() };
        Body {
            shape: Shape::Solid(Arc::new(inner)),
            size: if size.is_finite() && size > 0.0 { size } else { 0.0 },
            boundary_tol: None,
            color: None,
            paint: None,
            splits: None,
            face_sources: None,
            regularization_protected: false,
            history: None,
            display_of: None,
        }
    }

    pub(crate) fn from_mesh(mesh: TriMesh) -> Body {
        let mut bbox = Aabb3::EMPTY;
        for p in &mesh.positions {
            bbox.add(*p);
        }
        let size = if bbox.is_empty() { 0.0 } else { bbox.diagonal() };
        Body {
            shape: Shape::Mesh(Arc::new(mesh)),
            size: if size.is_finite() { size } else { 0.0 },
            boundary_tol: None,
            color: None,
            paint: None,
            splits: None,
            face_sources: None,
            regularization_protected: false,
            history: None,
            display_of: None,
        }
    }

    /// The solids behind a B-rep body; an error for a mesh body.
    pub(crate) fn inner(&self) -> Result<&Arc<Inner>> {
        match &self.shape {
            Shape::Solid(inner) => Ok(inner),
            Shape::Mesh(_) => Err(KernelError::Invalid("this needs a solid body; mesh bodies can only be moved, measured and exported".into())),
        }
    }

    pub(crate) fn require_brep(&self, what: &str) -> Result<()> {
        if self.is_mesh() {
            Err(KernelError::Invalid(format!("{what} needs a solid body; mesh bodies can only be moved, measured and exported")))
        } else {
            Ok(())
        }
    }

    pub(crate) fn tri_mesh(&self) -> Option<&Arc<TriMesh>> {
        match &self.shape {
            Shape::Mesh(m) => Some(m),
            Shape::Solid(_) => None,
        }
    }

    /// Are these the same body (the same solids, not merely equal ones)?
    pub fn same(&self, other: &Body) -> bool {
        self.same_shape(other)
            && self.color == other.color
            && self.face_sources.as_ref().map(Arc::as_ptr) == other.face_sources.as_ref().map(Arc::as_ptr)
            && self.paint.as_ref().map(Arc::as_ptr) == other.paint.as_ref().map(Arc::as_ptr)
    }

    /// Do they share their solids (whatever their colour and sources)?
    pub fn same_shape(&self, other: &Body) -> bool {
        match (&self.shape, &other.shape) {
            (Shape::Solid(a), Shape::Solid(b)) => Arc::ptr_eq(a, b),
            (Shape::Mesh(a), Shape::Mesh(b)) => Arc::ptr_eq(a, b),
            _ => false,
        }
    }

    pub fn history(&self) -> Option<&History> {
        self.history.as_deref()
    }

    pub(crate) fn with_history(mut self, history: History, operands: &[&Body]) -> Result<Self> {
        history.check(&self, operands)?;
        self.history = Some(Arc::new(history));
        Ok(self)
    }

    pub fn face_sources(&self) -> Option<&[Option<FaceSource>]> {
        self.face_sources.as_deref().map(Vec::as_slice)
    }

    pub(crate) fn with_face_sources(mut self, sources: Option<FaceSources>) -> Self {
        self.face_sources = sources;
        self
    }

    pub fn face_was_coalesced(&self, index: usize) -> bool {
        matches!(self.face_sources().and_then(|s| s.get(index)), Some(Some(FaceSource::ExtrudeSide { coalesced: true, .. })))
    }

    /// The plane of face `index` as its source records it, else as the geometry has it.
    pub fn face_plane(&self, index: usize) -> Option<Plane> {
        if index >= 50_000 {
            return None;
        }
        match self.face_sources().and_then(|s| s.get(index)).and_then(Option::as_ref) {
            Some(FaceSource::ExtrudeSide { plane, .. } | FaceSource::CertifiedExtrudeSide { plane, .. }) => *plane,
            Some(
                FaceSource::ExtrudeCap { plane, .. }
                | FaceSource::CertifiedExtrudeCap { plane, .. }
                | FaceSource::Box { plane, .. }
                | FaceSource::CylinderCap { plane, .. }
                | FaceSource::ShellPlanar { plane, .. },
            ) => Some(*plane),
            Some(FaceSource::Imported { .. }) | None => self.planar_face(index),
        }
    }

    /// The exact plane of face `index`, if it is planar: through the face's centre, its outward
    /// normal, `x` along the surface's own x direction.
    pub fn planar_face(&self, index: usize) -> Option<Plane> {
        if index >= 50_000 {
            return None;
        }
        let inner = self.inner().ok()?;
        inner
            .with("exact planar face", |solids| {
                let Some(face) = inner.face(solids, index) else { return Ok(None) };
                let Some(surface) = face.surface() else { return Ok(None) };
                if surface.kind != cadrum::SurfaceKind::Plane {
                    return Ok(None);
                }
                let center = v3(face.center());
                let (_, normal) = face.project(dv(center));
                let Some(n) = v3(normal).normalized() else { return Ok(None) };
                let x = v3(surface.axis_x);
                let Some(x) = (x - n * x.dot(n)).normalized().or_else(|| any_perpendicular(n)) else { return Ok(None) };
                let y = n.cross(x);
                Ok(Plane::new(center, x, y))
            })
            .ok()
            .flatten()
    }

    pub fn with_cylinder_certificate(mut self, producer: u64) -> Result<Self> {
        if producer == 0 {
            return Err(KernelError::Invalid("invalid cylinder producer".into()));
        }
        let Some(sources) = &self.face_sources else {
            return Err(KernelError::Failed("cylinder construction source".into()));
        };
        let mut sources = sources.as_ref().clone();
        let mut ends = [false; 2];
        for source in sources.iter_mut() {
            if let Some(FaceSource::ExtrudeCap { high, plane, .. }) = source {
                let end = ends.get_mut(usize::from(*high)).ok_or_else(|| KernelError::Failed("cylinder cap role".into()))?;
                if std::mem::replace(end, true) {
                    return Err(KernelError::Failed("duplicate cylinder cap role".into()));
                }
                *source = Some(FaceSource::CylinderCap { producer, high: *high, plane: *plane });
            }
        }
        if ends != [true; 2] {
            return Err(KernelError::Failed("cylinder cap role count".into()));
        }
        self.face_sources = Some(Arc::new(sources));
        Ok(self)
    }

    pub fn with_box_certificate(mut self, producer: u64) -> Result<Self> {
        if producer == 0 {
            return Err(KernelError::Invalid("invalid box producer".into()));
        }
        if let Some(sources) = &self.face_sources {
            let mut sources = sources.as_ref().clone();
            for source in sources.iter_mut().flatten() {
                if let FaceSource::Box { producer: id, .. } = source {
                    *id = Some(producer);
                }
            }
            self.face_sources = Some(Arc::new(sources));
        }
        Ok(self)
    }

    pub fn with_extrude_cap_certificates(mut self, producer: u64, sketch: u64, signatures: &[Option<CapSignature>]) -> Result<Self> {
        if producer == 0 || sketch == 0 {
            return Err(KernelError::Invalid("invalid cap producer".into()));
        }
        if let Some(sources) = &self.face_sources {
            if sources.len() != signatures.len() {
                return Err(KernelError::Invalid("cap certificate count mismatch".into()));
            }
            let mut sources = sources.as_ref().clone();
            for (source, signature) in sources.iter_mut().zip(signatures) {
                if let Some(FaceSource::ExtrudeCap { high, plane, .. }) = source {
                    let Some((outer, holes)) = signature else {
                        *source = None;
                        continue;
                    };
                    if outer.len() > 4096 || holes.len() > 4096 || holes.iter().any(|hole| hole.len() > 4096) {
                        return Err(KernelError::Invalid("oversized cap curve signature".into()));
                    }
                    let count = outer.len() + holes.iter().map(Vec::len).sum::<usize>();
                    if outer.is_empty()
                        || count > 4096
                        || holes.len() > 4096
                        || outer.iter().chain(holes.iter().flatten()).map(String::len).sum::<usize>() > 65_536
                        || outer.iter().chain(holes.iter().flatten()).any(|id| id.is_empty() || id.len() > 128)
                    {
                        return Err(KernelError::Invalid("invalid cap curve signature".into()));
                    }
                    let mut outer = outer.clone();
                    outer.sort();
                    outer.dedup();
                    let mut holes = holes.clone();
                    for hole in &mut holes {
                        if hole.is_empty() {
                            return Err(KernelError::Invalid("empty cap hole signature".into()));
                        }
                        hole.sort();
                        hole.dedup();
                    }
                    holes.sort();
                    if holes.windows(2).any(|pair| pair[0] == pair[1]) || holes.iter().flatten().any(|id| outer.binary_search(id).is_ok()) {
                        return Err(KernelError::Invalid("ambiguous cap hole signature".into()));
                    }
                    *source = Some(FaceSource::CertifiedExtrudeCap { producer, sketch, high: *high, outer, holes, plane: *plane });
                }
            }
            self.face_sources = Some(Arc::new(sources));
        }
        Ok(self)
    }

    /// The face sources moved by `m` (column-major, translation in `m[3]`).
    pub(crate) fn transformed_sources(&self, m: [[f64; 4]; 4]) -> Result<Option<FaceSources>> {
        let Some(sources) = &self.face_sources else { return Ok(None) };
        let vector = |p: Vec3| {
            Vec3::new(
                m[0][0] * p.x + m[1][0] * p.y + m[2][0] * p.z,
                m[0][1] * p.x + m[1][1] * p.y + m[2][1] * p.z,
                m[0][2] * p.x + m[1][2] * p.y + m[2][2] * p.z,
            )
        };
        let translation = Vec3::new(m[3][0], m[3][1], m[3][2]);
        let reflected = vector(Vec3::X).dot(vector(Vec3::Y).cross(vector(Vec3::Z))) < 0.;
        let mut sources = sources.as_ref().clone();
        for source in sources.iter_mut().flatten() {
            if let Some(plane) = source.plane_mut() {
                *plane = Plane::new(vector(plane.origin) + translation, vector(plane.x), vector(plane.y) * if reflected { -1. } else { 1. })
                    .ok_or_else(|| KernelError::Invalid("degenerate transformed face plane".into()))?;
            }
        }
        Ok(Some(Arc::new(sources)))
    }

    pub fn with_extrude_certificates(mut self, producer: u64, sketch: u64, curves: &[Option<String>]) -> Result<Self> {
        if producer == 0 || sketch == 0 || curves.iter().flatten().any(|curve| curve.is_empty() || curve.len() > 128) {
            return Err(KernelError::Invalid("invalid constructed side certificate".into()));
        }
        if let Some(sources) = &self.face_sources {
            if sources.len() != curves.len() {
                return Err(KernelError::Invalid("side certificate count mismatch".into()));
            }
            let sources = sources
                .iter()
                .zip(curves)
                .map(|(source, curve)| match (source, curve) {
                    (Some(FaceSource::ExtrudeSide { plane, .. }), Some(curve)) => {
                        Some(FaceSource::CertifiedExtrudeSide { producer, sketch, curve: curve.clone(), plane: *plane })
                    }
                    (Some(FaceSource::ExtrudeSide { .. }), None) => None,
                    _ => source.clone(),
                })
                .collect();
            self.face_sources = Some(Arc::new(sources));
        }
        Ok(self)
    }

    pub fn with_import_certificate(mut self, digest: &str, producer: u64) -> Result<Self> {
        if producer == 0 || digest.len() != 64 || !digest.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)) {
            return Err(KernelError::Invalid("invalid immutable import certificate".into()));
        }
        if let Some(sources) = &self.face_sources {
            let mut sources = sources.as_ref().clone();
            for source in sources.iter_mut().flatten() {
                if let FaceSource::Imported { payload_sha256, producer: origin, .. } = source {
                    *payload_sha256 = Some(digest.into());
                    *origin = Some(producer);
                }
            }
            self.face_sources = Some(Arc::new(sources));
        }
        Ok(self)
    }

    /// A shell's planar faces keep the sources of the faces they came from: not on this backend
    /// yet (shell arrives in phase 5), so the body is returned as it is.
    pub fn with_preserved_planar_sources(self, _input: &Body) -> Result<Self> {
        Ok(self)
    }

    pub fn with_shell_certificate(self, producer: u64) -> Result<Self> {
        if producer == 0 {
            return Err(KernelError::Invalid("invalid shell producer".into()));
        }
        Ok(self)
    }

    pub fn is_mesh(&self) -> bool {
        matches!(self.shape, Shape::Mesh(_))
    }

    /// An open sheet body (surface workspace): never on this backend yet.
    pub fn is_surface(&self) -> bool {
        false
    }

    pub fn color(&self) -> Option<[f32; 3]> {
        self.color
    }

    pub(crate) fn with_boundary_tol(mut self, tol: Option<f64>) -> Body {
        self.boundary_tol = tol.filter(|t| t.is_finite() && *t > 0.0);
        self
    }

    pub fn with_color(mut self, c: Option<[f32; 3]>) -> Body {
        self.color = c.filter(|c| c.iter().all(|x| x.is_finite())).map(|c| c.map(|x| x.clamp(0.0, 1.0)));
        self
    }

    pub fn split_lines(&self) -> Option<&Arc<Vec<Vec<Vec3>>>> {
        self.splits.as_ref()
    }

    pub fn display_share(&self) -> Option<(DisplayShare, [[f64; 4]; 4])> {
        self.display_of.as_deref().map(|(shared, t)| (DisplayShare(shared.clone()), *t))
    }

    pub fn with_display_share(mut self, share: &DisplayShare, t: [[f64; 4]; 4]) -> Body {
        if self.splits.is_none() && !self.is_mesh() {
            self.display_of = Some(Arc::new((share.0.clone(), t)));
        }
        self
    }

    /// Identity of the solids (shared by clones).
    pub fn solid_id(&self) -> usize {
        match &self.shape {
            Shape::Solid(inner) => Arc::as_ptr(inner) as usize,
            Shape::Mesh(m) => Arc::as_ptr(m) as usize,
        }
    }

    pub fn with_split_lines(mut self, s: Option<Arc<Vec<Vec<Vec3>>>>) -> Body {
        self.splits = s.filter(|l| !l.is_empty());
        if self.splits.is_some() {
            self.display_of = None;
        }
        self.regularization_protected |= self.splits.is_some();
        self
    }

    pub fn paint(&self) -> Option<&Paint> {
        self.paint.as_deref()
    }

    pub fn with_paint(mut self, p: Option<Paint>) -> Body {
        let clean = |x: f32| if x.is_finite() { x.clamp(0.0, 1.0) } else { 1.0 };
        self.paint = p
            .map(|mut p| {
                p.opacity = clean(p.opacity);
                p.faces.retain(|f| f.color.iter().all(|x| x.is_finite()));
                for f in &mut p.faces {
                    f.color = f.color.map(|x| x.clamp(0.0, 1.0));
                    f.opacity = clean(f.opacity);
                }
                p
            })
            .filter(|p| p.opacity < 1.0 || !p.faces.is_empty())
            .map(Arc::new);
        self.regularization_protected |= self.paint().is_some_and(|paint| !paint.faces.is_empty());
        self
    }

    /// The body's bounding-box diagonal (0 when unknown).
    pub fn size(&self) -> f64 {
        self.size
    }

    /// The body's bounding box.
    pub fn outline_bounds(&self) -> Aabb3 {
        match &self.shape {
            Shape::Solid(inner) => inner.bbox,
            Shape::Mesh(m) => {
                let mut b = Aabb3::EMPTY;
                for p in &m.positions {
                    b.add(*p);
                }
                b
            }
        }
    }

    /// The volume, when already measured at `tol` or better (cadrum's volume is exact).
    pub fn volume_at(&self, _tol: f64) -> Option<f64> {
        match &self.shape {
            Shape::Solid(inner) => inner.volume.get().copied(),
            Shape::Mesh(m) => Some(m.volume6().abs() / 6.0),
        }
    }

    /// About how much memory the body keeps beyond its shape.
    pub fn retained_bytes(&self) -> usize {
        match &self.shape {
            Shape::Solid(inner) => {
                let meshes: usize = inner.meshes.lock().unwrap_or_else(PoisonError::into_inner).iter().map(|(_, m)| mesh_bytes(m)).sum();
                (inner.faces.len() + inner.edges.len()) * 16 + meshes
            }
            Shape::Mesh(m) => m.positions.len() * 24 + m.triangles.len() * 12,
        }
    }

    /// The body with its meshes kept for reuse (they are kept by tolerance on every body here).
    pub fn keeping_meshes(&self) -> Body {
        self.clone()
    }

    pub fn contains(&self, p: Vec3) -> bool {
        if !p.is_finite() {
            return false;
        }
        match &self.shape {
            Shape::Solid(inner) => inner.with("contains", |solids| Ok(solids.iter().any(|s| s.contains(dv(p))))).unwrap_or(false),
            Shape::Mesh(_) => false,
        }
    }

    pub fn face_count(&self) -> usize {
        match &self.shape {
            Shape::Solid(inner) => inner.faces.len(),
            Shape::Mesh(_) => 1,
        }
    }

    pub fn edge_count(&self) -> usize {
        match &self.shape {
            Shape::Solid(inner) => inner.edges.len(),
            Shape::Mesh(_) => 0,
        }
    }

    pub fn vertex_count(&self) -> usize {
        match &self.shape {
            Shape::Solid(inner) => inner.vertices,
            Shape::Mesh(_) => 0,
        }
    }

    /// What is wrong with the body (nothing, for a sound one).
    pub fn validity(&self) -> Vec<String> {
        if let Shape::Mesh(m) = &self.shape {
            return if solidcraft_kernel_core::meshbody::is_closed(m) { Vec::new() } else { vec!["the mesh is not closed".into()] };
        }
        let mut out = Vec::new();
        match crate::measure(self) {
            Ok(m) if m.volume.is_finite() && m.volume > 0.0 => {}
            Ok(m) => out.push(format!("volume {}", m.volume)),
            Err(e) => out.push(format!("no measure: {e}")),
        }
        out
    }

    pub fn shell_count(&self) -> usize {
        match &self.shape {
            Shape::Solid(inner) => inner.lumps.max(1),
            Shape::Mesh(_) => 1,
        }
    }

    /// The body's separate lumps, each as a body of its own.
    pub fn lumps(&self) -> Result<Vec<Body>> {
        let Shape::Solid(inner) = &self.shape else { return Ok(vec![self.clone()]) };
        if inner.lumps < 2 {
            return Ok(vec![self.clone()]);
        }
        let pieces: Vec<Vec<cadrum::Solid>> = inner.with("lumps", |solids| Ok(solids.iter().map(|s| vec![cadrum_handle(s)]).collect()))?;
        let histories = crate::record::lump_histories(inner);
        pieces
            .into_iter()
            .zip(histories)
            .map(|(solids, h)| {
                let piece = Body::from_solids(solids)?;
                let sources = self.face_sources.as_ref().map(|_| h.face_sources(&[self]));
                let piece = piece.with_face_sources(sources.filter(|s| s.iter().any(Option::is_some)).map(Arc::new));
                Ok(crate::record::with_history_or_log(piece, Ok(h), &[self]))
            })
            .collect()
    }

    pub fn meets(&self, other: &Body) -> Result<bool> {
        self.require_brep("contact test")?;
        other.require_brep("contact test")?;
        let tol = (self.size().max(other.size()) * 1e-4).max(1e-6);
        let (a, b) = (self.outline_bounds().inflated(tol), other.outline_bounds());
        if a.is_empty()
            || b.is_empty()
            || a.min.x > b.max.x
            || b.min.x > a.max.x
            || a.min.y > b.max.y
            || b.min.y > a.max.y
            || a.min.z > b.max.z
            || b.min.z > a.max.z
        {
            return Ok(false);
        }
        Ok(crate::ops::boolean(self, other, crate::BoolOp::Intersect)?.is_some())
    }

    pub fn meets_each(&self, others: &[Body]) -> Result<Vec<bool>> {
        if others.len() > 1024 {
            return Err(KernelError::Invalid("too many contact tools".into()));
        }
        others.iter().map(|o| self.meets(o)).collect()
    }
}

impl solidcraft_kernel_core::Topology for Body {
    fn face_count(&self) -> usize {
        Body::face_count(self)
    }
    fn edge_count(&self) -> usize {
        Body::edge_count(self)
    }
    fn face_sources(&self) -> Option<&[Option<FaceSource>]> {
        Body::face_sources(self)
    }
    fn edge_faces(&self) -> Vec<Vec<usize>> {
        match &self.shape {
            Shape::Solid(inner) => inner.edge_faces.clone(),
            Shape::Mesh(_) => Vec::new(),
        }
    }
}

/// An owned copy of a solid for an operation that consumes one (cadrum's transforms take the
/// solid by value). A deep copy: the shape ids change, the face and edge order does not, and a
/// body made of it reads its own ids afresh.
pub(crate) fn cadrum_handle(s: &cadrum::Solid) -> cadrum::Solid {
    s.clone()
}

pub(crate) fn any_perpendicular(n: Vec3) -> Option<Vec3> {
    let t = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    (t - n * t.dot(n)).normalized()
}

fn mesh_bytes(m: &Mesh) -> usize {
    m.positions.len() * 48 + m.triangles.len() * 16 + m.edges.iter().map(|e| e.len() * 24).sum::<usize>()
}

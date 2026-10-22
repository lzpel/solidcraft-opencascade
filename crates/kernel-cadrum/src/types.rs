//! The plain data types of the kernel API that carry no backend state: the same definitions
//! (fields, derives, serde names) as the truck backend's, so documents and commands read and
//! write them identically whichever backend is built.

use std::sync::Arc;

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Plane, Seg2, UnitVec3, Vec3};

use crate::{KernelError, Result};

/// A boolean operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoolOp {
    Union,
    Cut,
    Intersect,
}

/// Geometric continuity across a seam.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Continuity {
    G0,
    #[default]
    G1,
    G2,
    G3,
}

impl Continuity {
    /// How many derivative conditions the order imposes (position, tangent, curvature, …).
    pub fn conditions(self) -> usize {
        match self {
            Continuity::G0 => 1,
            Continuity::G1 => 2,
            Continuity::G2 => 3,
            Continuity::G3 => 4,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Continuity::G0 => "g0",
            Continuity::G1 => "g1",
            Continuity::G2 => "g2",
            Continuity::G3 => "g3",
        }
    }

    pub fn parse(s: &str) -> Option<Continuity> {
        match s.to_ascii_lowercase().as_str() {
            "g0" => Some(Continuity::G0),
            "g1" => Some(Continuity::G1),
            "g2" => Some(Continuity::G2),
            "g3" => Some(Continuity::G3),
            _ => None,
        }
    }
}

/// One station of a boundary: where a bridge starts, the direction it leaves in, and what the
/// support surface does there along that direction.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub p: Vec3,
    pub leave: Vec3,
    pub bend: Vec3,
    pub bend_change: Vec3,
}

/// A segment of a sweep path.
#[derive(Clone, Copy, Debug)]
pub enum PathSeg {
    Line { a: Vec3, b: Vec3 },
    Arc { a: Vec3, center: Vec3, axis: Vec3, angle: f64 },
}

/// Which side of a chamfer's first distance the second is measured as.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ChamferSide {
    Distance(f64),
    Angle(f64),
}

/// A radius law along an edge: a B-spline over the edge's parameter 0…1.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RadiusSpline {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub radii: Vec<f64>,
}

impl RadiusSpline {
    /// A law through `pts` (parameter, radius): straight between them, or a smooth cubic.
    pub fn through(pts: &[(f64, f64)], smooth: bool) -> RadiusSpline {
        if !smooth || pts.len() < 3 {
            let mut knots = vec![0.0];
            knots.extend(pts.iter().map(|p| p.0));
            knots.push(1.0);
            return RadiusSpline { degree: 1, knots, radii: pts.iter().map(|p| p.1).collect() };
        }
        let n = pts.len();
        let at = |i: usize| pts.get(i).copied().unwrap_or((0.0, 0.0));
        let slope = |i: usize| {
            let (a, b) = (at(i.saturating_sub(1)), at((i + 1).min(n - 1)));
            if b.0 > a.0 { (b.1 - a.1) / (b.0 - a.0) } else { 0.0 }
        };
        let mut knots = vec![0.0; 4];
        let mut radii = vec![at(0).1];
        for i in 0..n - 1 {
            let ((t0, r0), (t1, r1)) = (at(i), at(i + 1));
            let h = t1 - t0;
            radii.extend([r0 + slope(i) * h / 3.0, r1 - slope(i + 1) * h / 3.0, r1]);
            knots.extend(if i + 2 == n { vec![1.0; 4] } else { vec![t1; 3] });
        }
        RadiusSpline { degree: 3, knots, radii }
    }

    /// The Greville abscissa of each control radius.
    pub fn greville(&self) -> Vec<f64> {
        (0..self.radii.len()).map(|j| self.knots.iter().skip(j + 1).take(self.degree).sum::<f64>() / self.degree.max(1) as f64).collect()
    }

    /// The radius at parameter `t` (Cox–de Boor over the clamped knot vector).
    pub fn eval(&self, t: f64) -> f64 {
        let t = t.clamp(0.0, 1.0);
        let (p, kv) = (self.degree, &self.knots);
        let n = self.radii.len();
        if n == 0 || kv.len() != n + p + 1 {
            return self.radii.first().copied().unwrap_or(0.0);
        }
        // The span: the last knot interval `t` lies in (the end parameter takes the last span).
        let mut k = p;
        while k + 1 < n && kv.get(k + 1).is_some_and(|x| t >= *x) {
            k += 1;
        }
        let mut d: Vec<f64> = (0..=p).map(|j| self.radii.get(j + k - p).copied().unwrap_or(0.0)).collect();
        for r in 1..=p {
            for j in (r..=p).rev() {
                let (lo, hi) = (kv.get(j + k - p).copied().unwrap_or(0.0), kv.get(j + 1 + k - r).copied().unwrap_or(1.0));
                let a = if hi > lo { (t - lo) / (hi - lo) } else { 0.0 };
                let (dj, dj1) = (d.get(j).copied().unwrap_or(0.0), d.get(j - 1).copied().unwrap_or(0.0));
                if let Some(slot) = d.get_mut(j) {
                    *slot = (1.0 - a) * dj1 + a * dj;
                }
            }
        }
        d.get(p).copied().unwrap_or(0.0)
    }

    /// The law without control radius `j` (an inner one), its nearest inner knot dropped with it.
    pub fn remove(&self, j: usize) -> Option<RadiusSpline> {
        let (n, p) = (self.radii.len(), self.degree);
        if j == 0 || j + 1 >= n || n <= p + 1 {
            return None;
        }
        let at = *self.greville().get(j)?;
        let inner = p + 1..n;
        let drop = inner.min_by(|a, b| {
            let (ka, kb) = (self.knots.get(*a).copied().unwrap_or(0.0), self.knots.get(*b).copied().unwrap_or(0.0));
            (ka - at).abs().total_cmp(&(kb - at).abs())
        })?;
        let mut knots = self.knots.clone();
        if drop >= knots.len() {
            return None;
        }
        knots.remove(drop);
        let mut radii = self.radii.clone();
        radii.remove(j);
        Some(RadiusSpline { degree: self.degree, knots, radii })
    }

    pub fn reversed(&self) -> RadiusSpline {
        RadiusSpline {
            degree: self.degree,
            knots: self.knots.iter().rev().map(|k| 1.0 - k).collect(),
            radii: self.radii.iter().rev().copied().collect(),
        }
    }

    /// The same law with a knot inserted at `u` (Boehm), or `None` where it cannot be.
    pub fn insert(&self, u: f64) -> Option<RadiusSpline> {
        let (n, p) = (self.radii.len(), self.degree);
        if !(u > 1e-6 && u < 1.0 - 1e-6) || n >= 100 || p == 0 || self.knots.iter().any(|k| (k - u).abs() < 1e-6) {
            return None;
        }
        let k = (p..n).rev().find(|i| self.knots.get(*i).is_some_and(|x| *x <= u))?;
        let mut radii = Vec::with_capacity(n + 1);
        radii.extend_from_slice(self.radii.get(..=k - p)?);
        for i in k - p + 1..=k {
            let (lo, hi) = (*self.knots.get(i)?, *self.knots.get(i + p)?);
            let a = (u - lo) / (hi - lo);
            radii.push((1.0 - a) * self.radii.get(i - 1)? + a * self.radii.get(i)?);
        }
        radii.extend_from_slice(self.radii.get(k..)?);
        let mut knots = self.knots.clone();
        knots.insert(k + 1, u);
        Some(RadiusSpline { degree: self.degree, knots, radii })
    }
}

/// A fillet whose radius changes along the edge: where it starts and its law.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Taper {
    pub start: Vec3,
    pub spline: RadiusSpline,
}

/// What a swept region finds along the way.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Occupancy {
    Empty,
    Full,
    Mixed,
}

/// Where two coaxial cylinders overlap.
#[derive(Clone, Debug)]
pub struct CoaxialOverlap {
    pub volume: f64,
    pub area: f64,
    pub centroid: Vec3,
    pub bbox: solidcraft_geom::Aabb3,
    pub offset: f64,
    pub length: f64,
}

/// An edge named for a continuity check.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EdgePick {
    Point(Vec3),
    Index(usize),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContinuityTolerances {
    pub gap_mm: f64,
    pub angle_deg: f64,
    pub curvature_per_mm: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct SeamWorst {
    pub value: f64,
    pub at: Vec3,
    pub edge_fraction: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SeamVerdict {
    Smooth,
    CurvatureJump,
    Sharp,
    Gap,
}

#[derive(Clone, Debug, Serialize)]
pub struct SeamReport {
    pub edge: usize,
    pub length: f64,
    pub samples: usize,
    pub gap_mm: SeamWorst,
    pub normal_angle_deg: SeamWorst,
    pub curvature_jump_per_mm: SeamWorst,
    pub verdict: SeamVerdict,
    pub points: Vec<Vec3>,
}

#[derive(Clone, Debug, Serialize)]
pub struct ContinuityReport {
    pub seams: Vec<SeamReport>,
    pub skipped: usize,
    pub unmeasured: usize,
}

/// The shortest distance between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyDistance {
    pub distance: f64,
    pub from: Vec3,
    pub to: Vec3,
    pub exact: bool,
}

const MAX_FAIR_ITERATIONS: usize = 1000;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FairOptions {
    pub strength: f64,
    pub iterations: usize,
    pub keep_edges: bool,
    pub tolerance: f64,
}

impl FairOptions {
    pub fn new(strength: f64, iterations: usize, keep_edges: bool, tolerance: f64) -> Result<FairOptions> {
        FairOptions { strength, iterations, keep_edges, tolerance }.checked()
    }

    pub fn checked(self) -> Result<FairOptions> {
        if !(self.strength.is_finite() && self.strength > 0.0 && self.strength <= 1.0) {
            return Err(KernelError::Invalid(format!("the fairing strength {} must be greater than 0 and at most 1", self.strength)));
        }
        if self.iterations == 0 || self.iterations > MAX_FAIR_ITERATIONS {
            return Err(KernelError::Invalid(format!("{} fairing iterations (1 to {MAX_FAIR_ITERATIONS})", self.iterations)));
        }
        if !(self.tolerance.is_finite() && self.tolerance > 0.0) {
            return Err(KernelError::Invalid(format!("the fairing tolerance {} must be a positive number of mm", self.tolerance)));
        }
        Ok(self)
    }
}

impl Default for FairOptions {
    fn default() -> Self {
        FairOptions { strength: 1.0, iterations: 1, keep_edges: false, tolerance: 1.0 }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FairReport {
    pub energy_before: f64,
    pub energy_after: f64,
    pub steps: usize,
    pub movement: f64,
    pub energy_trace: Vec<f64>,
}

pub const PATCH_ANGLE_TOL_DEG: f64 = 0.05;

#[derive(Clone, Debug, PartialEq)]
pub struct PatchSpec {
    pub continuity: Continuity,
    pub edges: Vec<(Vec3, Continuity)>,
    pub points: Vec<Vec3>,
    pub crown: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PatchReport {
    pub general: bool,
    pub gap: f64,
    pub normal_deviation_deg: f64,
    pub normal_deviation_all_deg: f64,
    pub curvature_mismatch: f64,
    pub point_gap: f64,
}

/// A half-space `n · p <= d`.
#[derive(Clone, Copy, Debug)]
pub struct HalfSpace {
    pub n: Vec3,
    pub d: f64,
}

pub const PLANAR_FIT_TOL: f64 = 1e-3;

pub const MAX_EDGE_PICKS: usize = 10_000;
pub const MAX_CHAIN_CURVES: usize = 1000;

/// An edge of a face to sew.
#[derive(Clone, Debug, PartialEq)]
pub enum EdgeSpec {
    Line { a: Vec3, b: Vec3 },
    Arc { a: Vec3, b: Vec3, mid: Vec3 },
    Chain { degree: usize, ctrl: Arc<[Vec3]> },
}

impl EdgeSpec {
    pub fn reversed(&self) -> EdgeSpec {
        match self {
            EdgeSpec::Line { a, b } => EdgeSpec::Line { a: *b, b: *a },
            EdgeSpec::Arc { a, b, mid } => EdgeSpec::Arc { a: *b, b: *a, mid: *mid },
            EdgeSpec::Chain { degree, ctrl } => EdgeSpec::Chain { degree: *degree, ctrl: ctrl.iter().rev().copied().collect() },
        }
    }
}

/// The surface of a face to sew.
#[derive(Clone, Debug, PartialEq)]
pub enum SurfSpec {
    Plane,
    Revolved { a: Vec3, b: Vec3, axis_origin: Vec3, axis: Vec3, angle: f64 },
    Extruded { a: Vec3, b: Vec3, mid: Vec3, dir: Vec3 },
    Ruled { a: EdgeSpec, b: EdgeSpec },
}

/// A face to sew: its loops, surface and a probe (a point and the outward normal there).
#[derive(Clone, Debug, PartialEq)]
pub struct FaceSpec {
    pub loops: Vec<Vec<EdgeSpec>>,
    pub surface: SurfSpec,
    pub probe: Option<(Vec3, Vec3)>,
}

/// A node of a STEP product tree.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StepNode {
    pub name: String,
    /// Placement in the parent (column-major 4×4, millimetres).
    pub transform: [[f64; 4]; 4],
    /// Indices into the import's bodies of this product's own bodies.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bodies: Vec<usize>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub children: Vec<StepNode>,
}

/// The header of a written STEP file.
#[derive(Clone, Debug, Default)]
pub struct StepHeader {
    pub file_name: String,
    pub time_stamp: String,
    pub author: String,
    pub organization: String,
    pub description: String,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ExtendMode {
    #[default]
    Natural,
    Tangent,
    Curvature,
}

impl ExtendMode {
    pub fn is_natural(self) -> bool {
        self == ExtendMode::Natural
    }
}

/// The elementary surface a face lies on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FaceSurface {
    Plane { normal: UnitVec3, point: Vec3 },
    Cylinder { origin: Vec3, axis: UnitVec3, radius: f64 },
    Cone { apex: Vec3, axis: UnitVec3, half_angle: f64 },
    Sphere { center: Vec3, radius: f64 },
    Torus { center: Vec3, axis: UnitVec3, major: f64, minor: f64 },
}

impl FaceSurface {
    /// The point of the whole surface (not only of its face) nearest `p`. `None` where every
    /// point of a ring is as near (on a cylinder's or cone's axis, at a sphere's or torus's
    /// centre), and for a point that is not finite.
    pub fn nearest(&self, p: Vec3) -> Option<Vec3> {
        if !p.is_finite() {
            return None;
        }
        match *self {
            FaceSurface::Plane { normal, point } => Some(p - normal.vec() * (p - point).dot(normal)),
            FaceSurface::Cylinder { origin, axis, radius } => {
                let w = p - origin;
                let along = axis.vec() * w.dot(axis);
                Some(origin + along + (w - along).normalized()? * radius)
            }
            FaceSurface::Cone { apex, axis, half_angle } => {
                let w = p - apex;
                let out = (w - axis.vec() * w.dot(axis)).normalized()?;
                let line = axis.vec() * half_angle.cos() + out * half_angle.sin();
                Some(apex + line * w.dot(line).max(0.0))
            }
            FaceSurface::Sphere { center, radius } => Some(center + (p - center).normalized()? * radius),
            FaceSurface::Torus { center, axis, major, minor } => {
                let w = p - center;
                let tube = center + (w - axis.vec() * w.dot(axis)).normalized()? * major;
                Some(tube + (p - tube).normalized()? * minor)
            }
        }
    }
}

/// A circular edge.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EdgeCircle {
    pub center: Vec3,
    pub axis: UnitVec3,
    pub radius: f64,
    pub start: Vec3,
    pub end: Vec3,
    pub angle: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RuledMode {
    Normal,
    Tangent,
    Angle { angle: f64, reference: Option<Vec3> },
    Direction(Vec3),
}

impl RuledMode {
    pub fn is_normal(&self) -> bool {
        matches!(self, RuledMode::Normal)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RuledCorners {
    #[default]
    Mitre,
    Round,
}

impl RuledCorners {
    pub fn is_mitre(self) -> bool {
        self == RuledCorners::Mitre
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ruled {
    pub distance: f64,
    pub mode: RuledMode,
    pub flip: bool,
    pub corners: RuledCorners,
    pub face: Option<Vec3>,
}

/// Which part of a trimmed surface to keep.
#[derive(Clone, Copy, Debug)]
pub enum TrimKeep<'a> {
    Side(Vec3),
    Regions(&'a [Vec3]),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum UntrimHoles<'a> {
    All,
    At(&'a [Vec3]),
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum SweepOrientation {
    #[default]
    Perpendicular,
    Parallel,
    Direction(Vec3),
}

impl SweepOrientation {
    pub fn is_perpendicular(&self) -> bool {
        matches!(self, SweepOrientation::Perpendicular)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RailFit {
    #[default]
    Scale,
    Stretch,
}

impl RailFit {
    pub fn is_scale(self) -> bool {
        self == RailFit::Scale
    }
}

/// How a swept profile turns, twists, scales and follows guide rails along its path.
#[derive(Clone, Debug, PartialEq)]
pub struct SweepControl {
    pub orientation: SweepOrientation,
    pub twist: f64,
    pub scale: f64,
    pub rails: Vec<Vec<Vec3>>,
    pub rail_fit: RailFit,
}

impl SweepControl {
    pub const MAX_TWIST: f64 = 4.0 * std::f64::consts::TAU;
    pub const MIN_SCALE: f64 = 0.01;
    pub const MAX_SCALE: f64 = 100.0;

    pub fn is_default(&self) -> bool {
        *self == SweepControl::default()
    }

    pub fn check(&self) -> Result<()> {
        if !(self.twist.is_finite() && self.twist.abs() <= Self::MAX_TWIST) {
            return Err(KernelError::Invalid("the sweep twist must be within four turns (1440 deg)".into()));
        }
        if !(self.scale.is_finite() && (Self::MIN_SCALE..=Self::MAX_SCALE).contains(&self.scale)) {
            return Err(KernelError::Invalid("the sweep end scale must be between 0.01 and 100".into()));
        }
        if let SweepOrientation::Direction(d) = self.orientation
            && !(d.is_finite() && d.len() > 1e-9 && d.len() < 1e9)
        {
            return Err(KernelError::Invalid("the sweep orientation direction must be a finite, non-zero vector".into()));
        }
        if self.rails.len() > 2 {
            return Err(KernelError::Invalid("a sweep takes one or two guide rails".into()));
        }
        for rail in &self.rails {
            if rail.len() < 2 || rail.len() > 100_000 || rail.iter().any(|p| !p.is_finite()) {
                return Err(KernelError::Invalid("a guide rail needs 2…100000 finite points along its length".into()));
            }
        }
        if !self.rails.is_empty() && (self.orientation != SweepOrientation::Perpendicular || self.twist != 0.0 || self.scale != 1.0) {
            return Err(KernelError::Invalid(
                "guide rails set the profile's turn and size along the path: leave the orientation, twist and end scale at their defaults".into(),
            ));
        }
        Ok(())
    }
}

impl Default for SweepControl {
    fn default() -> Self {
        SweepControl { orientation: SweepOrientation::Perpendicular, twist: 0.0, scale: 1.0, rails: Vec::new(), rail_fit: RailFit::Scale }
    }
}

/// A cylindrical face: its axis, radius and extent along the axis.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct CylinderFace {
    pub axis_point: Vec3,
    pub axis: UnitVec3,
    pub radius: f64,
    pub start: f64,
    pub end: f64,
    pub internal: bool,
}

impl CylinderFace {
    /// How far `p` is off the face: off its radius, plus beyond its ends.
    pub fn off(&self, p: Vec3) -> f64 {
        let v = p - self.axis_point;
        let along = v.dot(self.axis);
        ((v - self.axis.vec() * along).len() - self.radius).abs() + (self.start - along).max(along - self.end).max(0.0)
    }
}

/// A planar face a sketch region matches whole.
#[derive(Clone, Copy, Debug)]
pub struct WholeFace {
    pub face: usize,
    pub normal: Vec3,
    pub height: f64,
}

/// A tool that splits faces or bodies.
#[derive(Clone, Debug)]
pub enum SplitTool<'a> {
    Plane(Plane),
    Face { body: &'a crate::Body, face: usize },
    Curves { plane: Plane, curves: Vec<Vec<Seg2>>, along: Option<Vec3> },
}

#[derive(Clone, Copy, Debug)]
pub enum ExtendLimit<'a> {
    Distance(f64),
    UpTo(&'a SplitTool<'a>),
    Bounds,
}

#[derive(Clone, Copy, Debug)]
pub struct ExtendOptions<'a> {
    pub mode: ExtendMode,
    pub limit: ExtendLimit<'a>,
    pub neighbours: bool,
}

/// Where a boundary curve comes from.
pub enum CurveSource<'a> {
    Sketch { plane: Plane, segs: Vec<Seg2> },
    Points(Vec<Vec3>),
    Edges { body: &'a crate::Body, at: Vec<Vec3> },
    Point(Vec3),
}

pub struct BoundaryCurve<'a> {
    pub label: String,
    pub source: CurveSource<'a>,
    pub continuity: Continuity,
}

/// One boundary of a patch: its stations and the continuity it asks for.
#[derive(Clone, Debug)]
pub struct Boundary {
    pub frames: Vec<Frame>,
    pub order: Continuity,
}

/// A body's triangles, boxed and indexed for distance queries. Distances are between the
/// meshes on this backend (within the chord tolerances), never refined on the exact surfaces.
pub struct DistanceShape<'a> {
    set: solidcraft_geom::mesh_distance::TriangleSet,
    chord: f64,
    _mesh: &'a solidcraft_geom::Mesh,
}

impl<'a> DistanceShape<'a> {
    /// `mesh` is the body's mesh, within `chord` (mm) of its faces.
    pub fn new(_body: &'a crate::Body, mesh: &'a solidcraft_geom::Mesh, chord: f64) -> DistanceShape<'a> {
        DistanceShape {
            set: solidcraft_geom::mesh_distance::TriangleSet::new(mesh),
            chord: if chord.is_finite() && chord > 0.0 { chord } else { 0.0 },
            _mesh: mesh,
        }
    }

    /// The box around the body: its mesh's, grown by the chord tolerance.
    pub fn bounds(&self) -> solidcraft_geom::Aabb3 {
        self.set.bounds().inflated(self.chord)
    }

    pub fn chord(&self) -> f64 {
        self.chord
    }

    /// The nearest points of the two meshes, if they come within `limit`.
    pub fn mesh_closest(&self, other: &DistanceShape, limit: f64) -> Option<solidcraft_geom::mesh_distance::MeshClosest> {
        self.set.closest(&other.set, limit)
    }
}

/// The shortest distance between two bodies, from their meshes (`exact: false`), if it is within
/// `limit`.
pub fn body_distance(a: &DistanceShape, b: &DistanceShape, limit: f64) -> Result<Option<BodyDistance>> {
    let slack = 2.0 * (a.chord + b.chord);
    let Some(m) = a.mesh_closest(b, if limit.is_finite() { limit + slack } else { f64::INFINITY }) else { return Ok(None) };
    Ok(Some(BodyDistance { distance: m.distance, from: m.a, to: m.b, exact: false }))
}

/// A body's faces as a swept-region occupancy test.
pub struct Clearance<'a> {
    pub(crate) body: &'a crate::Body,
}

impl<'a> Clearance<'a> {
    /// `None` where the body cannot be prepared (not yet on this backend).
    pub fn new(_body: &'a crate::Body, _chord: f64) -> Option<Clearance<'a>> {
        None
    }

    pub fn swept(&self, _faces: &[([Vec3; 3], f64)], _along: Vec3, _lo: f64, _hi: f64) -> Occupancy {
        let _ = self.body;
        Occupancy::Mixed
    }
}

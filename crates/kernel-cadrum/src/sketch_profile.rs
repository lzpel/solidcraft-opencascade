//! Sketch profiles as cadrum edges: each segment of a region's loops placed on its plane, outer
//! loop first then the holes, so `Solid::extrude` and `Solid::revolve` read them as closed loops.
//! Lines and arcs are exact; free-form segments are interpolated through sample points until
//! cadrum takes control points (lzpel/cadrum#317).

use solidcraft_geom::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};
use solidcraft_kernel_core::numbered::{self, NumberedRegion};

use crate::body::dv;
use crate::{KernelError, Result, failed};

/// Most segments of one region (the cap's source stays small enough to store).
pub(crate) const MAX_PROFILE_SEGMENTS: usize = 4096;
/// Points a free-form segment is interpolated through.
const FREEFORM_SAMPLES: usize = 24;

pub(crate) fn frame_ok(plane: &Plane) -> bool {
    plane.origin.is_finite()
        && plane.x.is_finite()
        && plane.y.is_finite()
        && (plane.x.len2() - 1.).abs() <= 1e-8
        && (plane.y.len2() - 1.).abs() <= 1e-8
        && plane.x.dot(plane.y).abs() <= 1e-8
}

pub(crate) fn finite(x: f64, what: &str) -> Result<f64> {
    if x.is_finite() && x.abs() < 1e9 { Ok(x) } else { Err(KernelError::Invalid(format!("{what} is not finite"))) }
}

fn seg_ok(s: &Seg2) -> bool {
    let fin = |p: Vec2| p.is_finite() && p.x.abs() < 1e9 && p.y.abs() < 1e9;
    match *s {
        Seg2::Line { a, b } => fin(a) && fin(b),
        Seg2::Arc { center, radius, start, sweep } => {
            fin(center) && radius.is_finite() && radius > 1e-9 && radius < 1e9 && start.is_finite() && sweep.is_finite()
        }
        Seg2::Cubic { p0, p1, p2, p3 } => fin(p0) && fin(p1) && fin(p2) && fin(p3),
        Seg2::Conic { a, apex, b, w } => fin(a) && fin(apex) && fin(b) && w.is_finite() && w > 1e-9,
        Seg2::Bezier { n, p } => (4..=solidcraft_geom::MAX_BEZIER_DEGREE as u8).contains(&n) && p.iter().all(|q| fin(*q)),
    }
}

pub(crate) fn region_ok(r: &Region2) -> Result<()> {
    if !r.area().is_finite() || r.area().abs() < 1e-9 {
        return Err(KernelError::Invalid("profile has no area".into()));
    }
    let count = r.outer.segs.len() + r.holes.iter().map(|h| h.segs.len()).sum::<usize>();
    if count == 0 || count > MAX_PROFILE_SEGMENTS {
        return Err(KernelError::Invalid(format!("profile with {count} segments (limit {MAX_PROFILE_SEGMENTS})")));
    }
    if !std::iter::once(&r.outer).chain(&r.holes).flat_map(|l| &l.segs).all(seg_ok) {
        return Err(KernelError::Invalid("non-finite profile".into()));
    }
    Ok(())
}

/// Two consecutive lines that continue one another (whole sides merge them).
pub(crate) fn continues_straight(s: &Seg2, t: &Seg2) -> bool {
    let (u, v) = (s.end() - s.start(), t.end() - t.start());
    let (lu, lv) = (u.len(), v.len());
    lu > 0.0 && lv > 0.0 && s.end().dist(t.start()) <= 1e-9 * (lu + lv) && u.dot(v) > 0.0 && u.cross(v).abs() <= 1e-9 * lu * lv
}

/// A profile edge as built: the cadrum edge, the sketch segment it runs along (the first of
/// several merged ones) and the profile segment numbers under it.
pub(crate) struct ProfileEdge {
    pub(crate) edge: cadrum::Edge,
    pub(crate) segment: Seg2,
    pub(crate) numbers: Vec<usize>,
}

/// `r` numbered in profile order and oriented as the builders take it (outer counter-clockwise,
/// holes clockwise); with `whole_sides`, runs of collinear lines merged into one segment. Arcs
/// are never merged: each sketch arc certifies the side face it sweeps, so a circle drawn as two
/// arcs sweeps two half walls, as on the truck backend (measures merge them back for counting).
pub(crate) fn numbered(r: &Region2, whole_sides: bool) -> Result<NumberedRegion> {
    let n = numbered::number_regions(std::slice::from_ref(r)).pop().ok_or_else(|| KernelError::Invalid("profile".into()))?.oriented();
    Ok(if whole_sides { n.map_loops(|l| l.merged_lines(continues_straight)) } else { n })
}

/// The edges of `r`'s loops on `plane`, loop by loop (outer first), each loop closed.
pub(crate) fn edges(plane: &Plane, r: &NumberedRegion) -> Result<Vec<Vec<ProfileEdge>>> {
    r.loops()
        .map(|lp| {
            let mut out = Vec::with_capacity(lp.lp.segs.len());
            for (i, seg) in lp.lp.segs.iter().enumerate() {
                let Some(edge) = edge(plane, seg)? else { continue };
                out.push(ProfileEdge { edge, segment: *seg, numbers: lp.numbers(i).to_vec() });
            }
            if out.is_empty() {
                return Err(KernelError::Invalid("a profile loop has no length".into()));
            }
            Ok(out)
        })
        .collect()
}

/// One sketch segment as a cadrum edge on `plane`; `None` for a segment of no length.
fn edge(plane: &Plane, seg: &Seg2) -> Result<Option<cadrum::Edge>> {
    let at = |p: Vec2| dv(plane.to_world(p));
    let made = match *seg {
        Seg2::Line { a, b } => {
            if a.dist(b) <= 1e-9 {
                return Ok(None);
            }
            cadrum::Edge::line(at(a), at(b))
        }
        Seg2::Arc { center, radius, start, sweep } => {
            if sweep.abs() <= 1e-9 || radius <= 1e-9 {
                return Ok(None);
            }
            if sweep.abs() >= std::f64::consts::TAU - 1e-9 {
                // A whole circle as one segment: three points of it are not an arc.
                let axis = dv(plane.normal() * sweep.signum());
                cadrum::Edge::circle(radius, axis).map(|e| e.translate(at(center)))
            } else {
                let p = |t: f64| center + Vec2::new((start + sweep * t).cos(), (start + sweep * t).sin()) * radius;
                cadrum::Edge::arc_3pts(at(p(0.0)), at(p(0.5)), at(p(1.0)))
            }
        }
        Seg2::Cubic { .. } | Seg2::Conic { .. } | Seg2::Bezier { .. } => {
            let points: Vec<cadrum::DVec3> = (0..=FREEFORM_SAMPLES).map(|k| at(seg.point_at(k as f64 / FREEFORM_SAMPLES as f64))).collect();
            let tangent = |d: Vec2| dv(plane.dir_to_world(d));
            let (t0, t1) = (seg.start_tangent(), seg.end_tangent());
            if seg.length() <= 1e-9 {
                return Ok(None);
            }
            cadrum::Edge::bspline(points.iter(), cadrum::BSplineEnd::Clamped { start: tangent(t0), end: tangent(t1) })
        }
    };
    made.map(Some).map_err(|e| failed("profile edge", e))
}

/// The point on the loop's segment nearest `p`, by segment index.
pub(crate) fn nearest_segment(segs: &[Seg2], pts: &[Vec2]) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (k, s) in segs.iter().enumerate() {
        let d: f64 = pts.iter().map(|p| s.point_at(s.closest_param(*p)).dist(*p)).fold(0.0, f64::max);
        if best.is_none_or(|(_, b)| d < b) {
            best = Some((k, d));
        }
    }
    best.map(|(k, _)| k)
}

/// The outer loop's centroid in space.
pub(crate) fn centroid(plane: &Plane, lp: &Loop2) -> Vec3 {
    plane.to_world(lp.centroid())
}

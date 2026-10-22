//! Tapered extrudes: the profile's loops offset in their plane by the taper at each level
//! (the shared 2-D offsetting of `solidcraft_kernel_core::offset2d`), the levels skinned with
//! ruled faces (a loft), holes lofted and cut. The faces are placed by the profile geometry
//! as a sweep's are.

use std::f64::consts::FRAC_PI_2;

use solidcraft_geom::{Loop2, Plane, Region2, Seg2};
use solidcraft_kernel_core::offset2d::{offset_crosses, offset_loop, offset_splits};

use crate::body::Body;
use crate::sketch_profile::{self as profile, ProfileEdge};
use crate::sweep::{End, placed, without};
use crate::{KernelError, Result, failed, guard};

/// Extrude one region by `length` along `dir_sign` × the plane normal with a taper angle
/// (radians; positive grows the profile).
pub fn extrude_tapered(plane: &Plane, region: &Region2, length: f64, dir_sign: f64, taper: f64) -> Result<Body> {
    extrude_tapered_between(plane, region, 0.0, length, 0.0, dir_sign, taper)
}

/// [`extrude_tapered`] from `from` to `to` along `dir_sign` × the plane normal, the region
/// being the section at 0: its sides run on through it, so `from` below 0 starts the solid
/// that far before the profile, at the profile shrunk or grown by the taper. With a `foot`
/// (a height, less than half of `to - from`), the sides stand straight for that height above
/// `from` and only then slope, up to the same top.
pub fn extrude_tapered_between(plane: &Plane, region: &Region2, from: f64, to: f64, foot: f64, dir_sign: f64, taper: f64) -> Result<Body> {
    if !profile::frame_ok(plane) {
        return Err(KernelError::Invalid("extrude frame is not finite and orthonormal".into()));
    }
    let (from, to, foot, taper) =
        (profile::finite(from, "start")?, profile::finite(to, "distance")?, profile::finite(foot, "foot")?, profile::finite(taper, "taper")?);
    if to - from <= 1e-6 {
        return Err(KernelError::Invalid("extrude distance is zero".into()));
    }
    if !(0.0..(to - from) * 0.5).contains(&foot) {
        return Err(KernelError::Invalid(format!("taper foot = {foot}")));
    }
    if taper.abs() >= FRAC_PI_2 - 1e-3 {
        return Err(KernelError::Invalid("taper must be between −90° and 90°".into()));
    }
    profile::region_ok(region)?;
    let (t, sign) = (taper.tan(), dir_sign.signum());
    let numbered = profile::numbered(region, true)?;
    let reach = from.abs().max(to.abs()) * t;
    // Each loop cut where its free curves need it for the offset, with the segment of the
    // profile each piece comes from.
    let split: Vec<(Loop2, Vec<usize>)> = numbered.loops().map(|l| offset_splits(&l.lp, reach)).collect();
    let ends = |h: f64| split.iter().map(|(lp, _)| offset_loop(lp, h * t)).collect::<Result<Vec<_>>>();
    let (bottoms, tops) = (ends(from)?, ends(to)?);
    let tol = (reach.abs() * 0.02).max(1e-6);
    if t != 0.0 {
        match (offset_crosses(&bottoms, tol), offset_crosses(&tops, tol)) {
            (Some(false), Some(false)) => {}
            (Some(true), _) | (_, Some(true)) => {
                return Err(KernelError::Invalid(
                    "the taper closes the profile: at this distance its sides would cross (a stroke, a curve or a gap narrower than the taper); try a smaller taper or distance".into(),
                ));
            }
            _ => {
                return Err(KernelError::TooLarge(
                    "the tapered profile has too many details to check that its sides stay apart; taper fewer profiles at a time".into(),
                ));
            }
        }
    }
    let closed = |moved: &[Option<Seg2>]| Loop2 { segs: moved.iter().flatten().copied().collect() };
    // The levels, bottom to top, each loop's section at that level.
    let mut levels: Vec<(Plane, Vec<Loop2>)> = vec![(plane.offset(from * sign), bottoms.iter().map(|b| closed(b)).collect())];
    if foot > 0.0 {
        levels.push((plane.offset((from + foot) * sign), bottoms.iter().map(|b| closed(b)).collect()));
    }
    levels.push((plane.offset(to * sign), tops.iter().map(|b| closed(b)).collect()));
    let loops_count = split.len();
    // One ruled skin per loop through its sections (the outer loop's solid, the holes' tools).
    let skin = |k: usize| -> Result<cadrum::Solid> {
        let mut wires: Vec<Vec<cadrum::Edge>> = Vec::with_capacity(levels.len());
        for (lp_plane, loops) in &levels {
            let lp = loops.get(k).ok_or_else(|| KernelError::Failed("taper: loop".into()))?;
            if lp.segs.is_empty() {
                return Err(KernelError::Invalid("the taper swallows a loop of the profile".into()));
            }
            let region = Region2 { outer: lp.clone(), holes: Vec::new() };
            let n = profile::numbered(&region, false)?;
            let edges = profile::edges(lp_plane, &n)?;
            wires.push(edges.into_iter().flatten().map(|e| e.edge).collect());
        }
        guard("tapered extrude", || cadrum::Solid::loft(wires.iter().map(|w| w.iter()), true).map_err(|e| failed("tapered extrude", e)))
    };
    let solid = skin(0)?;
    let tools: Vec<cadrum::Solid> = (1..loops_count).map(skin).collect::<Result<_>>()?;
    let solid = without("tapered extrude", solid, &tools)?;
    // Placed by the profile at the level the section is exact (height 0): its own loops.
    let base = *plane;
    let loops: Vec<Vec<ProfileEdge>> = profile::edges(&base, &numbered)?;
    let end_plane = plane.offset(to * sign);
    let start_plane = plane.offset(from * sign);
    let end = End { point: end_plane.origin, normal: end_plane.normal() };
    placed_between("tapered extrude", solid, &base, &start_plane, region, &loops, Some(end))
}

/// [`placed`] for a solid whose profile plane (where its sides are placed from) is not its
/// start cap's plane: the start cap lies on `start`.
fn placed_between(
    what: &str,
    solid: cadrum::Solid,
    plane: &Plane,
    start: &Plane,
    region: &Region2,
    loops: &[Vec<ProfileEdge>],
    end: Option<End>,
) -> Result<Body> {
    if (start.origin - plane.origin).len() < 1e-9 {
        return placed(what, solid, plane, region, loops, end);
    }
    // The sides are matched through the profile plane; the caps found on the start and end
    // planes by `placed`, which takes the start cap on the profile plane, so the solid is
    // placed as if its profile sat at the start: the sides' points carry back the same way.
    placed(what, solid, start, region, loops, end)
}

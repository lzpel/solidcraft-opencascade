//! Offsetting the loops of a sketch region in its plane (the sides of a tapered extrude or a
//! draft): each segment moved to its right by a distance, arcs and free curves kept as such
//! where the offset stays close, corners mitred or rounded, and a check that the moved loops do
//! not cross. Pure 2-D geometry, shared by the backends.

use solidcraft_geom::{Loop2, Seg2, Vec2};

use crate::{KernelError, Result};

/// Offset every segment of a loop to its right by `d` (outward for a region's outer loop and
/// holes alike), each segment as one of the same kind (`None`: a line the offset swallows, whose
/// neighbours now meet at one point). Loops of arcs on one circle, and loops of lines, arcs and
/// free curves (spline and conic pieces, such as the outlines of text) where the arcs meet their
/// neighbours tangentially are supported.
pub fn offset_loop(lp: &Loop2, d: f64) -> Result<Vec<Option<Seg2>>> {
    if d == 0.0 {
        return Ok(lp.segs.iter().copied().map(Some).collect());
    }
    let all_arcs = lp.segs.iter().all(|s| matches!(s, Seg2::Arc { .. }));
    if all_arcs {
        let moved = lp
            .segs
            .iter()
            .map(|s| match *s {
                Seg2::Arc { center, radius, start, sweep } => {
                    let r = radius + sweep.signum() * d;
                    if r > 1e-6 {
                        Ok(Seg2::Arc { center, radius: r, start, sweep })
                    } else {
                        Err(KernelError::Invalid("the taper closes the profile".into()))
                    }
                }
                _ => Err(KernelError::Failed("taper: profile loop".into())),
            })
            .collect::<Result<Vec<_>>>()?;
        return arc_corners(lp, moved).map(|arcs| arcs.into_iter().map(Some).collect());
    }
    offset_segments(lp, d)
}

/// The moved arcs of an all-arc loop made to meet again: where two arcs of different circles meet
/// at a corner (a circle and the lobes on its sides), their moved circles no longer cross at the
/// arcs' old end angles, so the corner goes to where the moved circles cross, nearest the old
/// corner, and both arcs are cut there. Arcs meeting tangentially (or on one circle) keep their
/// angles: moved along their shared normal, they still meet.
fn arc_corners(lp: &Loop2, moved: Vec<Seg2>) -> Result<Vec<Seg2>> {
    let bad = || KernelError::Failed("taper: profile loop".into());
    let closes = || KernelError::Invalid("the taper closes the profile".into());
    let n = moved.len();
    // Each arc's start and end angle once moved.
    let mut ang: Vec<(f64, f64)> = moved
        .iter()
        .map(|s| match *s {
            Seg2::Arc { start, sweep, .. } => Ok((start, start + sweep)),
            _ => Err(bad()),
        })
        .collect::<Result<_>>()?;
    for i in 0..n {
        let k = (i + n - 1) % n;
        let (Some(prev), Some(this), Some(Seg2::Arc { center: c1, radius: r1, .. }), Some(Seg2::Arc { center: c2, radius: r2, .. })) =
            (lp.segs.get(k), lp.segs.get(i), moved.get(k).copied(), moved.get(i).copied())
        else {
            return Err(bad());
        };
        let (tin, tout) = (prev.end_tangent(), this.start_tangent());
        if tin.cross(tout).abs() < 1e-6 && tin.dot(tout) > 0.0 {
            continue;
        }
        let x = circles_cross(c1, r1, c2, r2, this.start()).ok_or_else(|| {
            KernelError::Failed("not supported yet: a taper this steep for arcs meeting at a corner (their moved circles part)".into())
        })?;
        if let Some(a) = ang.get_mut(k) {
            a.1 = (x - c1).angle();
        }
        if let Some(a) = ang.get_mut(i) {
            a.0 = (x - c2).angle();
        }
    }
    moved
        .iter()
        .zip(&ang)
        .map(|(s, &(a0, a1))| match *s {
            Seg2::Arc { center, radius, sweep, .. } => {
                // The same turning sense, the whole-turn equivalent nearest the old sweep.
                let tau = std::f64::consts::TAU;
                let mut sw = a1 - a0;
                sw += ((sweep - sw) / tau).round() * tau;
                if sw * sweep <= 0.0 || sw.abs() < 1e-9 {
                    return Err(closes());
                }
                Ok(Seg2::Arc { center, radius, start: a0, sweep: sw })
            }
            _ => Err(bad()),
        })
        .collect()
}

/// Where the circles (`c1`, `r1`) and (`c2`, `r2`) cross, the crossing nearest `near`; `None`
/// when they don't (apart, one inside the other, or concentric).
fn circles_cross(c1: Vec2, r1: f64, c2: Vec2, r2: f64, near: Vec2) -> Option<Vec2> {
    let d = c1.dist(c2);
    if d < 1e-12 || d > r1 + r2 + 1e-9 || d < (r1 - r2).abs() - 1e-9 {
        return None;
    }
    let u = (c2 - c1) * (1.0 / d);
    let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h = (r1 * r1 - a * a).max(0.0).sqrt();
    let base = c1 + u * a;
    let (p, q) = (base + u.perp() * h, base - u.perp() * h);
    let x = if p.dist(near) <= q.dist(near) { p } else { q };
    x.is_finite().then_some(x)
}

/// Where the line through `p` along `tin` and the one through `q` along `tout` meet once each
/// has moved `d` to its right (`q` moved, where they run on together).
fn mitre(p: Vec2, tin: Vec2, q: Vec2, tout: Vec2, d: f64) -> Vec2 {
    let right = |t: Vec2| Vec2::new(t.y, -t.x);
    let (p0, p1) = (p + right(tin) * d, q + right(tout) * d);
    let den = tin.cross(tout);
    if den.abs() < 1e-12 { p1 } else { p0 + tin * ((p1 - p0).cross(tout) / den) }
}

/// A free curve's control polygon moved `d` to its right, from `a` to `b` (where its moved
/// neighbours meet it): each leg moves `d` to its right and each inner control point goes where
/// its two moved legs meet. For a curve that turns little against `d` this stays close to the
/// true offset, and its ends run along the curve's end tangents, so its side meets its
/// neighbours' along straight rulings.
fn offset_polygon(ctrl: &[Vec2], a: Vec2, b: Vec2, d: f64) -> Option<Vec<Vec2>> {
    let legs: Vec<Option<Vec2>> = ctrl.iter().zip(ctrl.iter().skip(1)).map(|(p, q)| (*q - *p).normalized()).collect();
    // A leg of no length takes the direction of the nearest leg after it, else before it.
    let dirs: Vec<Vec2> = (0..legs.len())
        .map(|i| legs.iter().skip(i).flatten().next().or_else(|| legs.iter().take(i).rev().flatten().next()).copied())
        .collect::<Option<_>>()?;
    let m = ctrl.len();
    let mut out = Vec::with_capacity(m);
    for (i, p) in ctrl.iter().enumerate() {
        out.push(if i == 0 {
            a
        } else if i + 1 == m {
            b
        } else {
            mitre(*p, *dirs.get(i - 1)?, *p, *dirs.get(i)?, d)
        });
    }
    out.iter().all(|p| p.is_finite()).then_some(out)
}

/// Samples a moved free curve is fitted to, and checked against.
const OFFSET_SAMPLES: usize = 16;

/// Where the point at `t` of the free curve `s` goes once moved `d` to its right (along its
/// normal there). `None` where it has no tangent.
fn true_offset(s: &Seg2, t: f64, d: f64) -> Option<Vec2> {
    let tangent = s.derivative(t).normalized()?;
    Some(s.point_at(t) + Vec2::new(tangent.y, -tangent.x) * d)
}

/// The free curve `s` moved `d` to its right, as one curve of its kind from `a` to `b` (where
/// its moved neighbours meet it; its own moved ends where it runs on into them). It is fitted to
/// the true offset between the curve's own moved ends: a cubic keeps its end tangents and takes
/// the handle lengths that fit best (least squares over [`OFFSET_SAMPLES`] points); a conic or a
/// higher Bézier moves its control polygon ([`offset_polygon`]), its middle then put on the true
/// offset. Its ends then go to `a` and `b`, along its end tangents (where moved neighbours meet
/// it), their handles kept pointing the same way. Close for a curve that turns little against
/// `d`; [`offset_splits`] cuts the others into such pieces first.
fn offset_curve(s: &Seg2, a: Vec2, b: Vec2, d: f64) -> Option<Seg2> {
    let (n0, n1) = (true_offset(s, 0.0, d)?, true_offset(s, 1.0, d)?);
    let (t0, t3) = (s.start_tangent(), s.end_tangent());
    let middle = |moved: &Seg2| Some(true_offset(s, 0.5, d)? - moved.point_at(0.5));
    // The control points fitted between the natural ends.
    let mut ctrl: Vec<Vec2> = match *s {
        Seg2::Cubic { p0, p1, p2, p3 } => {
            // The moved cubic n0·(B0+B1) + n1·(B2+B3) + α·t0·B1 − β·t3·B2: linear in α, β.
            let (mut m, mut r) = ([[0.0f64; 2]; 2], [0.0f64; 2]);
            for k in 1..OFFSET_SAMPLES {
                let t = k as f64 / OFFSET_SAMPLES as f64;
                let u = 1.0 - t;
                let (b0, b1, b2, b3) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
                let rest = true_offset(s, t, d)? - (n0 * (b0 + b1) + n1 * (b2 + b3));
                let (c1, c2) = (t0 * b1, t3 * -b2);
                m[0][0] += c1.dot(c1);
                m[0][1] += c1.dot(c2);
                m[1][1] += c2.dot(c2);
                r[0] += c1.dot(rest);
                r[1] += c2.dot(rest);
            }
            let det = m[0][0] * m[1][1] - m[0][1] * m[0][1];
            let (alpha, beta) = if det.abs() > 1e-12 * (m[0][0] * m[1][1]).max(1e-30) {
                ((r[0] * m[1][1] - r[1] * m[0][1]) / det, (m[0][0] * r[1] - m[0][1] * r[0]) / det)
            } else {
                (f64::NAN, f64::NAN)
            };
            if alpha.is_finite() && beta.is_finite() && alpha > 0.0 && beta > 0.0 {
                vec![n0, n0 + t0 * alpha, n1 - t3 * beta, n1]
            } else {
                // Handles that would turn back: the moved control polygon, its middle on the offset.
                let mut q = offset_polygon(&[p0, p1, p2, p3], n0, n1, d)?;
                let [q0, q1, q2, q3] = q[..] else { return None };
                let shift = middle(&Seg2::Cubic { p0: q0, p1: q1, p2: q2, p3: q3 })? * (8.0 / 6.0);
                for point in q.iter_mut().take(3).skip(1) {
                    *point += shift;
                }
                q
            }
        }
        Seg2::Conic { a: c0, apex, b: c1, w } => {
            let mut q = offset_polygon(&[c0, apex, c1], n0, n1, d)?;
            let [q0, qa, q1] = q[..] else { return None };
            // The middle is (a + 2w·apex + b) / (2 + 2w).
            let shift = middle(&Seg2::Conic { a: q0, apex: qa, b: q1, w })? * ((2.0 + 2.0 * w) / (2.0 * w));
            if let Some(apex) = q.get_mut(1) {
                *apex += shift;
            }
            q
        }
        Seg2::Bezier { n, p } => {
            let k = (n as usize).clamp(1, solidcraft_geom::MAX_BEZIER_DEGREE);
            let mut q = offset_polygon(p.get(..=k)?, n0, n1, d)?;
            let mut all = p;
            for (dst, src) in all.iter_mut().zip(&q) {
                *dst = *src;
            }
            // The middle weighs the inner points by binomials summing to 2^k − 2.
            let shift = middle(&Seg2::Bezier { n, p: all })? * (2f64.powi(k as i32) / (2f64.powi(k as i32) - 2.0));
            for point in q.iter_mut().take(k).skip(1) {
                *point += shift;
            }
            q
        }
        Seg2::Line { .. } | Seg2::Arc { .. } => return None,
    };
    // The ends to where the neighbours meet; a handle the move would turn round goes with it.
    let last = ctrl.len().checked_sub(1)?;
    if last < 2 {
        return None;
    }
    let (ea, eb) = (a - n0, b - n1);
    if let Some(h) = ctrl.get_mut(1)
        && (*h - a).dot(t0) <= 0.0
    {
        *h += ea;
    }
    if let Some(h) = ctrl.get_mut(last - 1)
        && (b - *h).dot(t3) <= 0.0
    {
        *h += eb;
    }
    if let Some(first) = ctrl.first_mut() {
        *first = a;
    }
    if let Some(end) = ctrl.last_mut() {
        *end = b;
    }
    Some(match (*s, &ctrl[..]) {
        (Seg2::Cubic { .. }, &[p0, p1, p2, p3]) => Seg2::Cubic { p0, p1, p2, p3 },
        (Seg2::Conic { w, .. }, &[a, apex, b]) => Seg2::Conic { a, apex, b, w },
        (Seg2::Bezier { n, p }, q) => {
            let mut all = p;
            for (dst, src) in all.iter_mut().zip(q) {
                *dst = *src;
            }
            Seg2::Bezier { n, p: all }
        }
        _ => return None,
    })
}

/// How far a moved free curve strays from the free curve `s`'s true offset by `d`: the largest
/// distance from its points to the true offset's (on polylines of [`OFFSET_SAMPLES`] spans).
pub fn offset_error(s: &Seg2, d: f64) -> Option<f64> {
    let (a, b) = (true_offset(s, 0.0, d)?, true_offset(s, 1.0, d)?);
    let moved = offset_curve(s, a, b, d)?;
    let n = OFFSET_SAMPLES * 2;
    let truth: Vec<Vec2> = (0..=n).map(|k| true_offset(s, k as f64 / n as f64, d)).collect::<Option<_>>()?;
    let near = |p: Vec2| {
        truth
            .iter()
            .zip(truth.iter().skip(1))
            .map(|(x, y)| Seg2::Line { a: *x, b: *y })
            .map(|l| l.point_at(l.closest_param(p)).dist(p))
            .fold(f64::INFINITY, f64::min)
    };
    Some((0..=n).map(|k| near(moved.point_at(k as f64 / n as f64))).fold(0.0, f64::max))
}

/// How close a moved free curve is kept to its true offset by `d`: 2 % of it, or 10 µm (a
/// slope's offset is a fraction of a millimetre on text: finer would cut letters into many
/// pieces, and every piece a face the booleans meet).
pub fn offset_tolerance(d: f64) -> f64 {
    (d.abs() * 2e-2).max(1e-2)
}

/// Deepest halving of a free curve in [`offset_splits`] (at most 2^6 pieces).
const MAX_OFFSET_DEPTH: u32 = 6;

/// Whether the free curve `s` bends anywhere tighter than `d` (its radius of curvature below
/// |d|, on [`OFFSET_SAMPLES`] points): moved that far towards its centre, its true offset folds
/// over itself there.
fn tighter_than(s: &Seg2, d: f64) -> bool {
    let h = 1e-4;
    (0..=OFFSET_SAMPLES).any(|k| {
        let t = (k as f64 / OFFSET_SAMPLES as f64).clamp(h, 1.0 - h);
        let (v, a) = (s.derivative(t), (s.derivative(t + h) - s.derivative(t - h)) * (0.5 / h));
        let speed = v.len();
        speed > 1e-12 && v.cross(a).abs() * d.abs() >= speed.powi(3)
    })
}

/// The loop `lp` with each free curve cut into pieces whose offset by up to `d` (either way)
/// [`offset_curve`] fits within a small part of `d`, and the segment of `lp` each piece comes
/// from. Lines and arcs stay whole, and so does a piece bending tighter than `d`
/// ([`tighter_than`]): its true offset folds there, and a piece following it would carry the
/// fold, where one moved piece smooths it over.
pub fn offset_splits(lp: &Loop2, d: f64) -> (Loop2, Vec<usize>) {
    let tol = offset_tolerance(d);
    let mut segs = Vec::with_capacity(lp.segs.len());
    let mut from = Vec::with_capacity(lp.segs.len());
    for (i, s) in lp.segs.iter().enumerate() {
        if s.is_line_or_arc() || d == 0.0 {
            segs.push(*s);
            from.push(i);
            continue;
        }
        // Spans still to check, the last pushed checked first: the right half goes in first.
        let mut todo = vec![(*s, 0u32)];
        while let Some((piece, depth)) = todo.pop() {
            let close = |d: f64| offset_error(&piece, d).is_some_and(|e| e <= tol);
            if depth < MAX_OFFSET_DEPTH && !tighter_than(&piece, d) && !(close(d) && close(-d)) {
                let (left, right) = piece.split_at(0.5);
                todo.push((right, depth + 1));
                todo.push((left, depth + 1));
                continue;
            }
            segs.push(piece);
            from.push(i);
        }
    }
    (Loop2 { segs }, from)
}

/// [`offset_loop`] for a loop of lines, arcs and free curves. Each arc keeps its centre and
/// angles and its radius moves by `d` (so its side stays an exact cone), each line moves `d` to
/// its right, and each free curve moves its control polygon ([`offset_polygon`]). Where two
/// moved segments part (a corner the offset goes round) they are carried on along their end
/// tangents to where they meet; where they run into each other (a corner the offset goes into)
/// both are cut where they cross. A line shorter than the offset can turn around between its
/// moved neighbours (the flat tip of a letter's stroke): it goes, and its neighbours meet. An arc
/// has to meet its neighbours tangentially: at a corner its side would meet the next one along a
/// conic, not a straight ruling.
fn offset_segments(lp: &Loop2, d: f64) -> Result<Vec<Option<Seg2>>> {
    let closes = || KernelError::Invalid("the taper closes the profile".into());
    let at_corner = || KernelError::Failed("not supported yet: tapered profiles where an arc meets another edge at a corner".into());
    let bad = || KernelError::Failed("taper: profile loop".into());
    let right = |t: Vec2| Vec2::new(t.y, -t.x);
    let n = lp.segs.len();
    let moved: Vec<Option<Seg2>> = lp
        .segs
        .iter()
        .map(|s| match *s {
            Seg2::Arc { center, radius, start, sweep } => {
                let r = radius + sweep.signum() * d;
                if r > 1e-6 { Ok(Some(Seg2::Arc { center, radius: r, start, sweep })) } else { Err(closes()) }
            }
            _ => Ok(None),
        })
        .collect::<Result<_>>()?;
    let mut gone = vec![false; n];
    // Each round takes away at least one line, or ends.
    let pieces = loop {
        let kept: Vec<usize> = (0..n).filter(|&i| !gone.get(i).copied().unwrap_or(true)).collect();
        if kept.len() < 2 {
            return Err(closes());
        }
        // Where each kept segment starts and ends once moved, and the corners where two moved
        // segments run into each other (the one before, the one after).
        let (mut starts, mut ends) = (vec![Vec2::ZERO; n], vec![Vec2::ZERO; n]);
        let mut into: Vec<(usize, usize)> = Vec::new();
        for (j, &i) in kept.iter().enumerate() {
            let Some(&k) = kept.get((j + kept.len() - 1) % kept.len()) else { return Err(bad()) };
            let (Some(prev), Some(s), Some(prev_moved), Some(this_moved)) = (lp.segs.get(k), lp.segs.get(i), moved.get(k), moved.get(i)) else {
                return Err(bad());
            };
            let (tin, tout) = (prev.end_tangent(), s.start_tangent());
            let adjacent = (k + 1) % n == i;
            let turn = tin.cross(tout);
            let tangent = adjacent && turn.abs() < 1e-6 && tin.dot(tout) > 0.0;
            let (end, start) = match (prev_moved, this_moved) {
                (Some(a), _) if tangent => (a.end(), a.end()),
                (None, Some(b)) if tangent => (b.start(), b.start()),
                // Two sides running back towards each other across a gone tip: the stroke is
                // thinner than the taper.
                (None, None) if !adjacent && turn.abs() < 1e-9 => return Err(closes()),
                // Running on smoothly (the pieces of one curve): both move to the same point.
                // (The tangents' crossing is ill-conditioned there.)
                (None, None) if tangent => {
                    let m = s.start() + right(tout) * d;
                    (m, m)
                }
                (None, None) if turn * d < 0.0 => {
                    into.push((k, i));
                    (prev.end() + right(tin) * d, s.start() + right(tout) * d)
                }
                (None, None) => {
                    let m = mitre(prev.end(), tin, s.start(), tout, d);
                    (m, m)
                }
                _ => return Err(at_corner()),
            };
            if let (Some(e), Some(st)) = (ends.get_mut(k), starts.get_mut(i)) {
                (*e, *st) = (end, start);
            }
        }
        let mut pieces: Vec<Option<Seg2>> = vec![None; n];
        for &i in &kept {
            let (Some(s), Some(m), Some(&a), Some(&b)) = (lp.segs.get(i), moved.get(i), starts.get(i), ends.get(i)) else { return Err(bad()) };
            let piece = match (m, *s) {
                (Some(arc), _) => {
                    if arc.start().dist(a) > 1e-6 || arc.end().dist(b) > 1e-6 {
                        return Err(at_corner());
                    }
                    *arc
                }
                (None, Seg2::Line { .. }) => Seg2::Line { a, b },
                (None, Seg2::Arc { .. }) => return Err(bad()),
                (None, curve) => offset_curve(&curve, a, b, d).ok_or_else(closes)?,
            };
            if let Some(slot) = pieces.get_mut(i) {
                *slot = Some(piece);
            }
        }
        // Cut the two pieces at each corner the offset goes into where they cross; pieces that
        // don't cross meet where their end tangents do.
        for &(k, i) in &into {
            let (Some(Some(p)), Some(Some(q))) = (pieces.get(k).copied(), pieces.get(i).copied()) else { return Err(bad()) };
            let (p, q) = match meet(&p, &q) {
                Some((tp, tq, x)) => (with_end(&p.split_at(tp).0, x), with_start(&q.split_at(tq).1, x)),
                None => {
                    let x = mitre(p.end(), p.end_tangent(), q.start(), q.start_tangent(), 0.0);
                    (with_end(&p, x), with_start(&q, x))
                }
            };
            if let Some(a) = pieces.get_mut(k) {
                *a = Some(p);
            }
            if let Some(b) = pieces.get_mut(i) {
                *b = Some(q);
            }
        }
        let mut more = false;
        for &i in &kept {
            // A line that turned around has been swallowed by its neighbours.
            if let (Some(Seg2::Line { a: s0, b: s1 }), Some(Some(Seg2::Line { a, b }))) = (lp.segs.get(i), pieces.get(i))
                && (*b - *a).dot(*s1 - *s0) <= 0.0
                && let Some(g) = gone.get_mut(i)
            {
                *g = true;
                more = true;
            }
        }
        if !more {
            break pieces;
        }
    };
    let out = Loop2 { segs: pieces.iter().flatten().copied().collect() };
    if out.signed_area().signum() != lp.signed_area().signum() || out.signed_area().abs() < 1e-9 {
        return Err(closes());
    }
    Ok(pieces)
}

/// Where the end of `p` crosses the start of `q`, nearest the corner between them: the
/// parameters on each and the point. On polylines of a few dozen points each, the point taken
/// on a line where one is, else half way between the two crossing pieces.
///
/// Its limit: curves are sampled at [`MEET_SAMPLES`] points, so two that cross twice within one
/// sample span (a tight curve against its neighbour) can be missed. The corner then gets the
/// mitre of the end tangents instead, and the loop that makes, if it crosses itself, is refused
/// by [`offset_crosses`] rather than built.
fn meet(p: &Seg2, q: &Seg2) -> Option<(f64, f64, Vec2)> {
    const SAMPLES: usize = MEET_SAMPLES;
    let samples = |s: &Seg2| -> Vec<(f64, Vec2)> {
        let n = if matches!(s, Seg2::Line { .. }) { 1 } else { SAMPLES };
        (0..=n).map(|k| k as f64 / n as f64).map(|t| (t, s.point_at(t))).collect()
    };
    let (sp, sq) = (samples(p), samples(q));
    let mut best: Option<(f64, f64)> = None;
    for (a, b) in sp.iter().zip(sp.iter().skip(1)) {
        for (c, e) in sq.iter().zip(sq.iter().skip(1)) {
            let (r, s) = (b.1 - a.1, e.1 - c.1);
            let den = r.cross(s);
            if den.abs() < 1e-15 {
                continue;
            }
            let (u, v) = ((c.1 - a.1).cross(s) / den, (c.1 - a.1).cross(r) / den);
            if !((0.0..=1.0).contains(&u) && (0.0..=1.0).contains(&v)) {
                continue;
            }
            let (tp, tq) = (a.0 + (b.0 - a.0) * u, c.0 + (e.0 - c.0) * v);
            // Nearest the corner: little of `p` left behind its crossing, little of `q` before.
            if best.is_none_or(|(bp, bq)| (1.0 - tp) + tq < (1.0 - bp) + bq) {
                best = Some((tp, tq));
            }
        }
    }
    let (tp, tq) = best?;
    // Not at an end: a piece cut to nothing has no length left to rule.
    if tp <= 1e-9 || tq >= 1.0 - 1e-9 {
        return None;
    }
    // On a line where one meets it, so a line stays on its line (its side stays flat).
    let x = match (p, q) {
        (Seg2::Line { .. }, _) => p.point_at(tp),
        (_, Seg2::Line { .. }) => q.point_at(tq),
        _ => (p.point_at(tp) + q.point_at(tq)) * 0.5,
    };
    Some((tp, tq, x))
}

/// `s` ending at `x` (its last control point moved there).
fn with_end(s: &Seg2, x: Vec2) -> Seg2 {
    match *s {
        Seg2::Line { a, .. } => Seg2::Line { a, b: x },
        Seg2::Cubic { p0, p1, p2, .. } => Seg2::Cubic { p0, p1, p2, p3: x },
        Seg2::Conic { a, apex, w, .. } => Seg2::Conic { a, apex, b: x, w },
        Seg2::Bezier { n, p } => {
            let mut q = p;
            if let Some(last) = q.get_mut((n as usize).clamp(1, solidcraft_geom::MAX_BEZIER_DEGREE)) {
                *last = x;
            }
            Seg2::Bezier { n, p: q }
        }
        Seg2::Arc { .. } => *s,
    }
}

/// `s` starting at `x` (its first control point moved there).
fn with_start(s: &Seg2, x: Vec2) -> Seg2 {
    match *s {
        Seg2::Line { b, .. } => Seg2::Line { a: x, b },
        Seg2::Cubic { p1, p2, p3, .. } => Seg2::Cubic { p0: x, p1, p2, p3 },
        Seg2::Conic { apex, b, w, .. } => Seg2::Conic { a: x, apex, b, w },
        Seg2::Bezier { n, p } => {
            let mut q = p;
            if let Some(first) = q.first_mut() {
                *first = x;
            }
            Seg2::Bezier { n, p: q }
        }
        Seg2::Arc { .. } => *s,
    }
}

/// Points a free curve is sampled at by [`meet`].
const MEET_SAMPLES: usize = 48;

/// Most pairs of polyline pieces [`offset_crosses`] compares before it gives up.
const CROSSING_BUDGET: usize = 20_000_000;

/// Whether the offset loops of a region cross themselves or each other, on polylines within
/// `tol` of them: the taper is too steep for the profile's details (a stroke thinner than the
/// taper, a curve tighter than it, two loops it brings together). `None` when there is too
/// much to compare within [`CROSSING_BUDGET`]: then the offset is not known to be sound.
pub fn offset_crosses(loops: &[Vec<Option<Seg2>>], tol: f64) -> Option<bool> {
    let lines: Vec<Vec<Vec2>> = loops
        .iter()
        .map(|l| {
            let mut pts: Vec<Vec2> = Vec::new();
            for s in l.iter().flatten() {
                let p = s.polyline(tol);
                pts.extend(p.iter().skip(if pts.is_empty() { 0 } else { 1 }));
            }
            pts
        })
        .collect();
    // Each piece: its loop, its place in the loop, its ends.
    let mut pieces: Vec<(usize, usize, usize, Vec2, Vec2)> = Vec::new();
    for (k, pts) in lines.iter().enumerate() {
        for (i, (a, b)) in pts.iter().zip(pts.iter().skip(1)).enumerate() {
            pieces.push((k, i, pts.len() - 1, *a, *b));
        }
    }
    pieces.sort_by(|p, q| p.3.x.min(p.4.x).total_cmp(&q.3.x.min(q.4.x)));
    let side = |p: Vec2, q: Vec2, r: Vec2| (q - p).cross(r - p);
    let mut work = 0usize;
    for (n, &(k, i, len, a, b)) in pieces.iter().enumerate() {
        let right = a.x.max(b.x);
        for &(k2, i2, _, c, d) in pieces.iter().skip(n + 1) {
            if c.x.min(d.x) > right {
                break;
            }
            work += 1;
            if work > CROSSING_BUDGET {
                return None;
            }
            // Neighbours in one loop share an end.
            if k == k2 && (i.abs_diff(i2) <= 1 || i.abs_diff(i2) == len.saturating_sub(1)) {
                continue;
            }
            if a.y.max(b.y) < c.y.min(d.y) || c.y.max(d.y) < a.y.min(b.y) {
                continue;
            }
            let (d1, d2, d3, d4) = (side(c, d, a), side(c, d, b), side(a, b, c), side(a, b, d));
            if d1 * d2 < 0.0 && d3 * d4 < 0.0 {
                return Some(true);
            }
        }
    }
    Some(false)
}

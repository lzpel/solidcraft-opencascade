//! Robustness checks on random inputs (`solidcraft-cli fuzz`, docs/robustness.md): the boolean
//! volume identity on random pairs of primitives placed on a coarse grid, so coincident faces
//! and tangencies come up often.

use serde::Serialize;
use solidcraft_geom::Vec3;

pub use solidcraft_kernel_core::fuzz::Rng;

use crate::{Body, BoolOp, boolean, box_solid, cylinder, measure, sphere, torus};

/// A primitive used by the boolean checks, as plain data (so a failure can be written down).
#[derive(Clone, Debug, Serialize)]
pub enum Shape {
    Box { a: [f64; 3], b: [f64; 3] },
    Cylinder { base: [f64; 3], axis: [f64; 3], radius: f64, height: f64 },
    Sphere { center: [f64; 3], radius: f64 },
    Torus { center: [f64; 3], major: f64, minor: f64 },
}

impl Shape {
    pub fn body(&self) -> crate::Result<Body> {
        let v = |p: &[f64; 3]| Vec3::new(p[0], p[1], p[2]);
        match self {
            Shape::Box { a, b } => box_solid(v(a), v(b)),
            Shape::Cylinder { base, axis, radius, height } => cylinder(v(base), v(axis), *radius, *height),
            Shape::Sphere { center, radius } => sphere(v(center), *radius),
            Shape::Torus { center, major, minor } => torus(v(center), *major, *minor),
        }
    }

    fn random(r: &mut Rng) -> Shape {
        let g = 2.5;
        let mut p = || [r.grid(0.0, 30.0, g), r.grid(0.0, 30.0, g), r.grid(0.0, 30.0, g)];
        let at = p();
        match r.below(10) {
            0..=4 => {
                let size = [r.grid(5.0, 30.0, g), r.grid(5.0, 30.0, g), r.grid(5.0, 30.0, g)];
                Shape::Box { a: at, b: [at[0] + size[0], at[1] + size[1], at[2] + size[2]] }
            }
            5..=7 => {
                let axis = r.pick(&[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]).unwrap_or([0.0, 0.0, 1.0]);
                Shape::Cylinder { base: at, axis, radius: r.grid(2.5, 10.0, g), height: r.grid(5.0, 30.0, g) }
            }
            8 => Shape::Sphere { center: at, radius: r.grid(2.5, 12.5, g) },
            _ => {
                let minor = r.grid(2.5, 5.0, g);
                Shape::Torus { center: at, major: minor + r.grid(2.5, 10.0, g), minor }
            }
        }
    }
}

/// One boolean check: the shapes, what happened, and the retries the booleans reported.
#[derive(Clone, Debug, Serialize)]
pub struct BoolCase {
    pub seed: u64,
    pub a: Shape,
    pub b: Shape,
    pub outcome: String,
    pub detail: String,
    pub boolean_fallbacks: std::collections::BTreeMap<crate::BooleanStrategy, usize>,
    pub boolean_reports_truncated: bool,
}

fn vol(b: &Option<Body>) -> crate::Result<f64> {
    match b {
        Some(b) => Ok(measure(b)?.volume),
        None => Ok(0.0),
    }
}

pub fn boolean_shapes(seed: u64) -> (Shape, Shape) {
    let mut r = Rng::new(seed);
    (Shape::random(&mut r), Shape::random(&mut r))
}

/// The volume identity `vol(a) + vol(b) = vol(a ∪ b) + vol(a ∩ b)` and `vol(a − b) = vol(a) −
/// vol(a ∩ b)` on the seed's pair, to a tenth of a percent of the larger volume.
pub fn boolean_case(seed: u64) -> BoolCase {
    let (a, b) = boolean_shapes(seed);
    let mut case = BoolCase {
        seed,
        a: a.clone(),
        b: b.clone(),
        outcome: "ok".into(),
        detail: String::new(),
        boolean_fallbacks: Default::default(),
        boolean_reports_truncated: false,
    };
    let fail = |c: &mut BoolCase, kind: &str, d: String| {
        c.outcome = kind.into();
        c.detail = d;
    };
    let (Ok(ba), Ok(bb)) = (a.body(), b.body()) else {
        fail(&mut case, "error", "a primitive could not be built".into());
        return case;
    };
    let (va, vb) = match (measure(&ba), measure(&bb)) {
        (Ok(ma), Ok(mb)) => (ma.volume, mb.volume),
        _ => {
            fail(&mut case, "error", "a primitive could not be measured".into());
            return case;
        }
    };
    let mut results = Vec::new();
    for op in [BoolOp::Union, BoolOp::Intersect, BoolOp::Cut] {
        let (result, diagnostics) = crate::capture_boolean_diagnostics(|| boolean(&ba, &bb, op));
        for event in &diagnostics.events {
            *case.boolean_fallbacks.entry(event.strategy).or_default() += 1;
        }
        case.boolean_reports_truncated |= diagnostics.truncated;
        match result {
            Ok(r) => results.push(r),
            Err(e) => {
                fail(&mut case, "error", format!("{op:?}: {e}"));
                return case;
            }
        }
    }
    let v = |b: &Option<Body>| vol(b).unwrap_or(f64::NAN);
    let (vu, vi, vc) = (v(results.first().unwrap_or(&None)), v(results.get(1).unwrap_or(&None)), v(results.get(2).unwrap_or(&None)));
    let tol = 1e-3 * va.max(vb).max(1e-9);
    if !(vu.is_finite() && vi.is_finite() && vc.is_finite()) {
        fail(&mut case, "error", "a result could not be measured".into());
    } else if (va + vb - vu - vi).abs() > tol || (vc - (va - vi)).abs() > tol {
        fail(&mut case, "mismatch", format!("a={va:.3} b={vb:.3} union={vu:.3} intersection={vi:.3} cut={vc:.3}"));
    }
    case
}

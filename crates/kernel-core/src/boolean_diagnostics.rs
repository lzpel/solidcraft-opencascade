//! Scoped reports for successful Boolean retries. Discarded attempts never enter
//! their caller's report, and independent evaluations cannot inherit old events.

use std::cell::RefCell;

use serde::{Deserialize, Serialize};

#[doc(hidden)]
pub const MAX_EVENTS: usize = 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BooleanStrategy {
    RefinedTolerance,
    ShiftedOperands,
    CoplanarSeparation,
    CommonBore,
    /// A boss in the other body's bore, grown into the material around it (nothing recut).
    GrownBoss,
    PlanarFallback,
    ToolNudge,
    /// A rotationally symmetric tool turned about its own axis; its volume is unchanged.
    ToolReseam,
    Piecewise,
    /// No retry gave a result, and the answer from sample points was kept although the bodies'
    /// facetings disagree with it.
    UnverifiedShortcut,
}

/// Numerical adjustments used by a successful retry, in millimetres/radians.
/// Operand shifts are reversed on the result; face shifts are temporary working
/// offsets. A tool nudge, in contrast, changes the relative operand placement.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BooleanFallback {
    pub strategy: BooleanStrategy,
    pub translation_mm: f64,
    pub rotation_rad: f64,
    pub face_shift_mm: f64,
    pub tolerance_mm: f64,
}

impl BooleanStrategy {
    /// Whether a result made this way can differ from the exact one: the tool was moved
    /// against the other body, or part of where they meet was ignored. The other retries work
    /// on temporary geometry or tolerances only, and their results are the exact ones.
    pub fn alters_result(self) -> bool {
        matches!(self, BooleanStrategy::ToolNudge | BooleanStrategy::UnverifiedShortcut)
    }
}

impl BooleanFallback {
    #[doc(hidden)]
    pub fn new(strategy: BooleanStrategy) -> Self {
        Self { strategy, translation_mm: 0., rotation_rad: 0., face_shift_mm: 0., tolerance_mm: 0. }
    }

    fn description(&self) -> String {
        match self.strategy {
            BooleanStrategy::RefinedTolerance => format!("refined tolerance {:.6e} mm", self.tolerance_mm),
            BooleanStrategy::ShiftedOperands => {
                format!("shifted operands {:.6e} mm (result shifted back), tolerance {:.6e} mm", self.translation_mm, self.tolerance_mm)
            }
            BooleanStrategy::CoplanarSeparation => format!("temporary coincident-face separation up to {:.6e} mm per face", self.face_shift_mm),
            BooleanStrategy::CommonBore => format!("temporary bore offset {:.6e} mm, followed by recutting the original bore", self.face_shift_mm),
            BooleanStrategy::GrownBoss => format!("boss grown {:.6e} mm into the material around the bore it fills", self.face_shift_mm),
            BooleanStrategy::PlanarFallback => "exact planar fallback (no displacement)".into(),
            BooleanStrategy::ToolNudge => format!("tool displacement {:.6e} mm, rotation {:.6e} rad", self.translation_mm, self.rotation_rad),
            BooleanStrategy::ToolReseam => format!("axisymmetric tool reseamed by {:.6e} rad (no displacement)", self.rotation_rad),
            BooleanStrategy::Piecewise => "piecewise retry (no displacement)".into(),
            BooleanStrategy::UnverifiedShortcut => format!(
                "not verified: the bodies' facets pass through each other by more than {:.6e} mm, but they could not be intersected there; that part is ignored",
                self.tolerance_mm
            ),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BooleanDiagnostics {
    pub events: Vec<BooleanFallback>,
    pub truncated: bool,
    /// Why candidates of the retries that led to the result were discarded before they were
    /// measured (#3020: a coarse candidate whose edges run far outside the operands, which
    /// meshing never finished). Left out of the report when there are none.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub rejected: Vec<String>,
}

impl BooleanDiagnostics {
    #[doc(hidden)]
    pub fn push(&mut self, event: BooleanFallback) {
        if self.events.len() < MAX_EVENTS {
            self.events.push(event);
        } else {
            self.truncated = true;
        }
    }

    #[doc(hidden)]
    pub fn push_rejected(&mut self, reason: String) {
        // A candidate checked twice (picked, then taken) is noted once.
        if self.rejected.contains(&reason) {
            return;
        }
        if self.rejected.len() < MAX_EVENTS {
            self.rejected.push(reason);
        } else {
            self.truncated = true;
        }
    }

    /// Why the result is not verified, when it ignores a place where the bodies seem to pass
    /// through each other ([`BooleanStrategy::UnverifiedShortcut`]).
    pub fn unverified(&self) -> Option<String> {
        self.events.iter().find(|event| event.strategy == BooleanStrategy::UnverifiedShortcut).map(BooleanFallback::description)
    }

    /// Every retry the result took, for a message about the boolean (one that failed its
    /// checks lists what was tried).
    pub fn summary(&self) -> Option<String> {
        self.described(&self.events.iter().collect::<Vec<_>>())
    }

    /// The note a feature carries: the retries that can make its result differ from the exact
    /// one ([`BooleanStrategy::alters_result`]). Exact retries are in `events` all the same
    /// (`timeline.inspect`), but they leave the result as it should be: no reason to warn.
    pub fn warning(&self) -> Option<String> {
        self.described(&self.events.iter().filter(|e| e.strategy.alters_result()).collect::<Vec<_>>())
    }

    fn described(&self, events: &[&BooleanFallback]) -> Option<String> {
        if events.is_empty() && !self.truncated {
            return None;
        }
        let mut descriptions: Vec<String> = events.iter().take(4).map(|e| e.description()).collect();
        if events.len() > 4 {
            descriptions.push(format!("{} additional retries", events.len() - 4));
        }
        if self.truncated {
            descriptions.push(format!("report capped at {MAX_EVENTS} retries"));
        }
        Some(format!("Boolean retry: {}", descriptions.join("; ")))
    }
}

thread_local! {
    static REPORT: RefCell<Option<BooleanDiagnostics>> = const { RefCell::new(None) };
}

struct Scope {
    previous: Option<Option<BooleanDiagnostics>>,
}

impl Drop for Scope {
    fn drop(&mut self) {
        if let Some(previous) = self.previous.take() {
            REPORT.with(|cell| {
                if let Ok(mut report) = cell.try_borrow_mut() {
                    *report = previous;
                }
            });
        }
    }
}

/// Capture only the events produced by this call. Nested callers explicitly merge
/// an accepted attempt; dropping a scope restores its parent even during unwinding.
pub fn capture_boolean_diagnostics<T>(run: impl FnOnce() -> T) -> (T, BooleanDiagnostics) {
    let previous = REPORT.with(|cell| cell.try_borrow_mut().ok().map(|mut report| report.replace(BooleanDiagnostics::default())));
    let scope = Scope { previous };
    let result = run();
    let report = REPORT.with(|cell| cell.try_borrow_mut().ok().and_then(|mut report| report.take())).unwrap_or_default();
    drop(scope);
    (result, report)
}

#[doc(hidden)]
pub fn record(event: BooleanFallback) {
    REPORT.with(|cell| {
        if let Ok(mut report) = cell.try_borrow_mut()
            && let Some(report) = report.as_mut()
        {
            report.push(event);
        }
    });
}

/// Note a candidate discarded before it was measured, and why.
#[doc(hidden)]
pub fn reject(reason: String) {
    REPORT.with(|cell| {
        if let Ok(mut report) = cell.try_borrow_mut()
            && let Some(report) = report.as_mut()
        {
            report.push_rejected(reason);
        }
    });
}

#[doc(hidden)]
pub fn merge(report: BooleanDiagnostics) {
    for event in report.events {
        record(event);
    }
    for reason in report.rejected {
        reject(reason);
    }
    if report.truncated {
        REPORT.with(|cell| {
            if let Ok(mut report) = cell.try_borrow_mut()
                && let Some(report) = report.as_mut()
            {
                report.truncated = true;
            }
        });
    }
}

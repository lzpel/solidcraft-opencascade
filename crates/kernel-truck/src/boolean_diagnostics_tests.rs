//! Tests of the boolean diagnostics scope against this backend's booleans.
use crate::boolean_diagnostics::*;
use solidcraft_kernel_core::boolean_diagnostics::MAX_EVENTS;

#[test]
fn nested_attempts_are_committed_explicitly_and_do_not_leak() {
    let (_, report) = capture_boolean_diagnostics(|| {
        let (_, discarded) = capture_boolean_diagnostics(|| record(BooleanFallback::new(BooleanStrategy::ToolNudge)));
        assert_eq!(discarded.events.len(), 1);
        let (_, accepted) = capture_boolean_diagnostics(|| record(BooleanFallback::new(BooleanStrategy::PlanarFallback)));
        merge(accepted);
    });
    assert_eq!(report.events.len(), 1);
    assert_eq!(report.events[0].strategy, BooleanStrategy::PlanarFallback);
    assert!(capture_boolean_diagnostics(|| {}).1.events.is_empty());
    record(BooleanFallback::new(BooleanStrategy::ToolNudge));
    assert!(capture_boolean_diagnostics(|| {}).1.events.is_empty());
}

/// Candidates discarded unmeasured (#3020) are reported with the attempt that led to the
/// result, once each, and not at all when there are none.
#[test]
fn rejected_candidates_follow_the_accepted_attempt() {
    let (_, report) = capture_boolean_diagnostics(|| {
        let (_, discarded) = capture_boolean_diagnostics(|| reject("from a discarded attempt".into()));
        assert_eq!(discarded.rejected.len(), 1);
        let (_, accepted) = capture_boolean_diagnostics(|| {
            reject("runaway edge".into());
            reject("runaway edge".into());
            record(BooleanFallback::new(BooleanStrategy::ToolNudge));
        });
        merge(accepted);
    });
    assert_eq!(report.rejected, vec!["runaway edge".to_string()]);
    assert!(serde_json::to_string(&report).unwrap().contains("\"rejected\":[\"runaway edge\"]"));
    let plain = capture_boolean_diagnostics(|| record(BooleanFallback::new(BooleanStrategy::ToolNudge))).1;
    assert!(!serde_json::to_string(&plain).unwrap().contains("rejected"));
}

/// Only retries whose result can differ from the exact one make a warning; the summary
/// lists every retry.
#[test]
fn only_retries_that_alter_the_result_warn() {
    let report = |strategies: &[BooleanStrategy]| {
        capture_boolean_diagnostics(|| {
            for s in strategies {
                record(BooleanFallback::new(*s));
            }
        })
        .1
    };
    let exact = report(&[BooleanStrategy::ShiftedOperands, BooleanStrategy::GrownBoss, BooleanStrategy::CoplanarSeparation]);
    assert!(exact.warning().is_none(), "{exact:?}");
    assert!(exact.summary().is_some_and(|s| s.contains("shifted operands") && s.contains("boss grown")));
    let nudged = report(&[BooleanStrategy::GrownBoss, BooleanStrategy::ToolNudge]);
    let warning = nudged.warning().unwrap();
    assert!(warning.contains("tool displacement") && !warning.contains("boss grown"), "{warning}");
}

#[test]
fn exact_operations_and_failed_calls_have_no_retry_report() {
    use solidcraft_geom::Vec3;
    let body = crate::box_solid(Vec3::ZERO, Vec3::new(10., 10., 10.)).unwrap();
    let other = crate::box_solid(Vec3::new(5., 0., 0.), Vec3::new(15., 10., 10.)).unwrap();
    let (result, diagnostics) = capture_boolean_diagnostics(|| crate::boolean(&body, &other, crate::BoolOp::Union));
    assert!(result.unwrap().is_some());
    assert!(diagnostics.warning().is_none(), "{diagnostics:?}");
    let mesh = crate::mesh_body(&[Vec3::ZERO, Vec3::X, Vec3::Y], &[[0, 1, 2]]).unwrap();
    let (result, diagnostics) = capture_boolean_diagnostics(|| crate::boolean(&body, &mesh, crate::BoolOp::Cut));
    assert!(result.is_err());
    assert!(diagnostics.warning().is_none(), "{diagnostics:?}");
}

#[test]
fn unwinding_restores_parent_and_reports_are_thread_local_and_bounded() {
    let (_, report) = capture_boolean_diagnostics(|| {
        let failed = crate::guard("diagnostic test", || {
            capture_boolean_diagnostics(|| -> crate::Result<()> {
                record(BooleanFallback::new(BooleanStrategy::ToolNudge));
                panic!("discard this attempt");
            })
            .0
        });
        assert!(failed.is_err());
        let separate = std::thread::spawn(|| capture_boolean_diagnostics(|| record(BooleanFallback::new(BooleanStrategy::ToolNudge))).1);
        assert_eq!(separate.join().unwrap().events.len(), 1);
        for _ in 0..MAX_EVENTS + 1 {
            record(BooleanFallback::new(BooleanStrategy::PlanarFallback));
        }
    });
    assert_eq!(report.events.len(), MAX_EVENTS);
    assert!(report.truncated);
    assert!(report.events.iter().all(|event| event.strategy == BooleanStrategy::PlanarFallback));
    assert!(report.warning().unwrap().contains("capped"));
}

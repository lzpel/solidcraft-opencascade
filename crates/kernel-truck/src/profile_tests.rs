//! Kernel time accounting (`solidcraft_kernel_core::profile`) over this backend's operations.

use solidcraft_kernel_core::profile::capture;

#[test]
fn a_faceted_boolean_is_boolean_time_and_a_reference_mesh_tessellation() {
    use crate::{BoolOp, box_solid, faceted_boolean};
    use solidcraft_geom::Vec3;
    let a = box_solid(Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0)).unwrap();
    let tool = box_solid(Vec3::new(5.0, 5.0, 5.0), Vec3::new(15.0, 15.0, 15.0)).unwrap().tessellate(0.1).unwrap();
    // The thread groove's route: the faceting it does first is the boolean's time too.
    let (r, t) = capture(|| faceted_boolean(&a, &tool, BoolOp::Cut, 0.01));
    assert!(r.unwrap().is_some());
    assert!(t.boolean_ms > 0.0 && t.tessellate_ms == 0.0, "{t:?}");
    let (m, t) = capture(|| a.reference_display_mesh_angular(0.01, crate::DISPLAY_ANGLE));
    assert!(!m.unwrap().triangles.is_empty());
    assert!(t.tessellate_ms > 0.0 && t.boolean_ms == 0.0, "{t:?}");
}

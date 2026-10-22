//! The backend against analytic values, hostile inputs and threads.

use std::f64::consts::{PI, TAU};

use solidcraft_geom::Vec3;

use crate::{
    BoolOp, FaceSource, FaceSurface, KernelError, Result, boolean, box_solid, cylinder, face_surface, measure, sphere, torus, transform,
    transform_matrix,
};

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1.0)
}

#[test]
fn a_box_measures_as_a_box() -> Result<()> {
    let b = box_solid(Vec3::ZERO, Vec3::new(40.0, 30.0, 20.0))?;
    let m = measure(&b)?;
    assert!(close(m.volume, 24_000.0, 1e-9), "{}", m.volume);
    assert!(close(m.area, 5_200.0, 1e-9), "{}", m.area);
    assert!((m.centroid - Vec3::new(20.0, 15.0, 10.0)).len() < 1e-9);
    assert_eq!((m.faces, m.edges, m.vertices, m.shells), (6, 12, 8, 1));
    assert_eq!(m.merged.faces, 6);
    assert!((m.bbox.min - Vec3::ZERO).len() < 1e-6 && (m.bbox.max - Vec3::new(40.0, 30.0, 20.0)).len() < 1e-6, "{:?}", m.bbox);
    // OpenCASCADE's bounding box is grown by its confusion tolerance.
    assert!(close(b.size(), (1600.0_f64 + 900.0 + 400.0).sqrt(), 1e-6), "{}", b.size());
    let sources = b.face_sources().ok_or(KernelError::Failed("no sources".into()))?;
    let mut sides: Vec<(u8, bool)> = sources
        .iter()
        .flatten()
        .map(|s| match s {
            FaceSource::Box { axis, high, .. } => (*axis, *high),
            other => panic_free(other),
        })
        .collect();
    sides.sort_unstable();
    assert_eq!(sides, vec![(0, false), (0, true), (1, false), (1, true), (2, false), (2, true)]);
    assert!(b.contains(Vec3::new(1.0, 1.0, 1.0)) && !b.contains(Vec3::new(-1.0, 1.0, 1.0)));
    assert!(b.validity().is_empty());
    Ok(())
}

fn panic_free(s: &FaceSource) -> (u8, bool) {
    log::error!("unexpected source {s:?}");
    (255, false)
}

#[test]
fn a_cylinder_keeps_one_wall_and_two_caps() -> Result<()> {
    let c = cylinder(Vec3::new(5.0, 5.0, -1.0), Vec3::Z, 2.0, 6.0)?;
    let m = measure(&c)?;
    assert!(close(m.volume, PI * 4.0 * 6.0, 1e-9), "{}", m.volume);
    assert!(close(m.area, 2.0 * PI * 4.0 + TAU * 2.0 * 6.0, 1e-9), "{}", m.area);
    assert_eq!(m.faces, 3);
    assert_eq!(m.merged.face_types.get("cylinder"), Some(&1));
    let caps: Vec<bool> = c
        .face_sources()
        .ok_or(KernelError::Failed("no sources".into()))?
        .iter()
        .filter_map(|s| match s {
            Some(FaceSource::ExtrudeCap { high, .. }) => Some(*high),
            _ => None,
        })
        .collect();
    assert_eq!(caps.len(), 2);
    assert!(caps.contains(&true) && caps.contains(&false));
    let certified = c.clone().with_cylinder_certificate(7)?;
    assert_eq!(
        certified.face_sources().map(|s| s.iter().filter(|s| matches!(s, Some(FaceSource::CylinderCap { producer: 7, .. }))).count()),
        Some(2)
    );
    let wall = (0..3).find(|i| matches!(face_surface(&c, *i), Some(FaceSurface::Cylinder { .. }))).ok_or(KernelError::Failed("no wall".into()))?;
    let Some(FaceSurface::Cylinder { radius, axis, .. }) = face_surface(&c, wall) else { return Err(KernelError::Failed("not a cylinder".into())) };
    assert!((radius - 2.0).abs() < 1e-9 && axis.vec().dot(Vec3::Z).abs() > 0.999);
    Ok(())
}

#[test]
fn spheres_and_tori_measure_exactly() -> Result<()> {
    let s = sphere(Vec3::new(1.0, 2.0, 3.0), 5.0)?;
    assert!(close(measure(&s)?.volume, 4.0 / 3.0 * PI * 125.0, 1e-8));
    let t = torus(Vec3::ZERO, 6.0, 1.5)?;
    assert!(close(measure(&t)?.volume, 2.0 * PI * PI * 6.0 * 1.5 * 1.5, 1e-8));
    Ok(())
}

#[test]
fn booleans_follow_the_volume_identity() -> Result<()> {
    let a = box_solid(Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0))?;
    let b = box_solid(Vec3::new(5.0, 5.0, 5.0), Vec3::new(15.0, 15.0, 15.0))?;
    let (va, vb) = (measure(&a)?.volume, measure(&b)?.volume);
    let union = boolean(&a, &b, BoolOp::Union)?.ok_or(KernelError::Failed("empty union".into()))?;
    let inter = boolean(&a, &b, BoolOp::Intersect)?.ok_or(KernelError::Failed("empty intersection".into()))?;
    let cut = boolean(&a, &b, BoolOp::Cut)?.ok_or(KernelError::Failed("empty cut".into()))?;
    let (vu, vi, vc) = (measure(&union)?.volume, measure(&inter)?.volume, measure(&cut)?.volume);
    assert!(close(vi, 125.0, 1e-9) && close(vu, va + vb - vi, 1e-9) && close(vc, va - vi, 1e-9), "{vu} {vi} {vc}");
    assert_eq!(union.lumps()?.len(), 1);
    let far = box_solid(Vec3::new(50.0, 0.0, 0.0), Vec3::new(60.0, 10.0, 10.0))?;
    assert!(boolean(&a, &far, BoolOp::Intersect)?.is_none());
    let apart = boolean(&a, &far, BoolOp::Union)?.ok_or(KernelError::Failed("empty union".into()))?;
    assert_eq!((apart.shell_count(), apart.lumps()?.len()), (2, 2));
    assert!(a.meets(&b)? && !a.meets(&far)?);
    Ok(())
}

#[test]
fn a_bore_through_a_block() -> Result<()> {
    let block = box_solid(Vec3::ZERO, Vec3::new(40.0, 30.0, 20.0))?;
    let bore = cylinder(Vec3::new(20.0, 15.0, -1.0), Vec3::Z, 5.0, 22.0)?;
    let cut = boolean(&block, &bore, BoolOp::Cut)?.ok_or(KernelError::Failed("empty cut".into()))?;
    let m = measure(&cut)?;
    assert!(close(m.volume, 24_000.0 - PI * 25.0 * 20.0, 1e-9), "{}", m.volume);
    assert_eq!(m.faces, 7);
    let mesh = cut.tessellate(0.01)?;
    assert!(!mesh.triangles.is_empty() && mesh.tri_face.iter().all(|f| (*f as usize) < 7));
    assert_eq!(mesh.edges.len(), cut.edge_count());
    assert_eq!(mesh.normals.len(), mesh.positions.len());
    assert!(mesh.seams.iter().any(|s| *s), "the bore's wall has a seam");
    let plan = cut.display_plan(0.05)?;
    assert_eq!(plan.faces.len(), 7);
    assert_eq!(plan.evaluate().triangles.len(), cut.display_mesh(0.05)?.triangles.len());
    Ok(())
}

#[test]
fn rigid_moves_keep_the_measure_and_face_order() -> Result<()> {
    let b = box_solid(Vec3::ZERO, Vec3::new(4.0, 3.0, 2.0))?;
    let moved = transform(&b, Vec3::new(10.0, 0.0, 0.0), Vec3::ZERO, Vec3::Z, PI / 2.0)?;
    let m = measure(&moved)?;
    assert!(close(m.volume, 24.0, 1e-9));
    // A quarter turn about Z then +10 in x: the box spans x 7..10, y 0..4.
    assert!((m.bbox.min - Vec3::new(7.0, 0.0, 0.0)).len() < 1e-6 && (m.bbox.max - Vec3::new(10.0, 4.0, 2.0)).len() < 1e-6, "{:?}", m.bbox);
    assert!(moved.history().is_some());
    assert_eq!(moved.face_sources().map(<[_]>::len), Some(6));
    let scaled = transform_matrix(&b, [[2.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]])?;
    assert!(close(measure(&scaled)?.volume, 24.0 * 8.0, 1e-9));
    let mirrored = transform_matrix(&b, [[-1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])?;
    let mm = measure(&mirrored)?;
    assert!(close(mm.volume, 24.0, 1e-9) && mm.bbox.max.x.abs() < 1e-6 && (mm.bbox.min.x + 4.0).abs() < 1e-6, "{:?}", mm.bbox);
    let sheared = transform_matrix(&b, [[1.0, 0.0, 0.0, 0.0], [0.5, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]);
    assert!(matches!(sheared, Err(KernelError::TooLarge(_))), "{sheared:?}");
    Ok(())
}

#[test]
fn hostile_inputs_are_refused_not_crashed() {
    let bad = [f64::NAN, f64::INFINITY, -f64::INFINITY, 1e300];
    for x in bad {
        assert!(box_solid(Vec3::new(x, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0)).is_err());
        assert!(cylinder(Vec3::ZERO, Vec3::Z, x, 1.0).is_err());
        assert!(cylinder(Vec3::ZERO, Vec3::new(x, 0.0, 0.0), 1.0, 1.0).is_err());
        assert!(sphere(Vec3::ZERO, x).is_err());
        assert!(torus(Vec3::ZERO, x, 1.0).is_err());
    }
    assert!(box_solid(Vec3::ZERO, Vec3::ZERO).is_err());
    assert!(cylinder(Vec3::ZERO, Vec3::ZERO, 1.0, 1.0).is_err());
    assert!(torus(Vec3::ZERO, 1.0, 2.0).is_err());
    let b = box_solid(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)).unwrap_or_else(|_| mesh_fallback());
    for x in bad {
        assert!(transform_matrix(&b, [[x, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]).is_err());
        assert!(transform(&b, Vec3::new(x, 0.0, 0.0), Vec3::ZERO, Vec3::Z, 1.0).is_err());
    }
    // A zero axis is no rotation: the move alone applies.
    let moved = transform(&b, Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO, Vec3::ZERO, 1.0);
    assert!(moved.is_ok_and(|m| (m.outline_bounds().min - Vec3::new(1.0, 0.0, 0.0)).len() < 1e-6));
    assert!(!b.contains(Vec3::new(f64::NAN, 0.0, 0.0)));
}

fn mesh_fallback() -> crate::Body {
    crate::mesh_body(&[Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z], &[[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]]).unwrap_or_else(|e| {
        log::error!("{e}");
        crate::Body::from_mesh(solidcraft_kernel_core::meshbody::TriMesh::default())
    })
}

#[test]
fn a_panic_inside_the_guard_is_an_internal_error() {
    let r: Result<()> = crate::guard("test", || {
        let empty: Vec<u8> = Vec::new();
        let _ = empty[3];
        Ok(())
    });
    assert!(matches!(r, Err(KernelError::Internal(_))), "{r:?}");
}

#[test]
fn one_body_is_measured_and_meshed_from_several_threads() -> Result<()> {
    let block = box_solid(Vec3::ZERO, Vec3::new(40.0, 30.0, 20.0))?;
    let bore = cylinder(Vec3::new(20.0, 15.0, -1.0), Vec3::Z, 5.0, 22.0)?;
    let cut = boolean(&block, &bore, BoolOp::Cut)?.ok_or(KernelError::Failed("empty cut".into()))?;
    let volumes: Vec<f64> = std::thread::scope(|s| {
        let handles: Vec<_> = (0..4)
            .map(|i| {
                let cut = cut.clone();
                s.spawn(move || -> Result<f64> {
                    let tol = 0.01 * (i + 1) as f64;
                    let mesh = cut.tessellate(tol)?;
                    assert!(!mesh.triangles.is_empty());
                    Ok(measure(&cut)?.volume)
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap_or(Err(KernelError::Internal("thread".into())))).collect::<Result<Vec<f64>>>()
    })?;
    assert!(volumes.iter().all(|v| close(*v, 24_000.0 - PI * 25.0 * 20.0, 1e-9)));
    Ok(())
}

#[test]
fn edges_and_faces_are_read_by_index() -> Result<()> {
    let c = cylinder(Vec3::ZERO, Vec3::Z, 2.0, 6.0)?;
    let edges = c.edges(0.01)?;
    assert_eq!(edges.len(), c.edge_count());
    let circles: Vec<_> = (0..c.edge_count()).filter_map(|i| crate::edge_circle(&c, i)).collect();
    assert_eq!(circles.len(), 2, "two rims");
    assert!(circles.iter().all(|k| (k.radius - 2.0).abs() < 1e-6 && (k.angle - TAU).abs() < 1e-6));
    let faces = c.faces(0.01)?;
    assert_eq!(faces.len(), 3);
    let normals: Vec<f64> = faces.iter().filter_map(|f| f.plane_normal).map(|n| n.z).collect();
    assert_eq!(normals.len(), 2);
    assert!(normals.contains(&1.0) || normals.iter().any(|z| (z - 1.0).abs() < 1e-9));
    let nearest = c.nearest_edge(Vec3::new(2.0, 0.0, 6.0), 0.01)?;
    assert!(nearest.is_some_and(|(_, d)| d < 0.02), "{nearest:?}");
    assert!(
        crate::cylinder_face_at(&c, Vec3::new(2.0, 0.0, 3.0))
            .is_some_and(|f| (f.radius - 2.0).abs() < 1e-9 && f.start.abs() < 1e-9 && (f.end - 6.0).abs() < 1e-9)
    );
    Ok(())
}

#[test]
fn unsupported_operations_say_so() {
    let b = box_solid(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)).unwrap_or_else(|_| mesh_fallback());
    let e = crate::fillet_g2(&b, &[Vec3::ZERO], 0.1);
    assert!(matches!(&e, Err(KernelError::TooLarge(m)) if m.contains("OpenCASCADE")), "{e:?}");
    assert!(crate::extrude(&solidcraft_geom::Plane::XY, &[], 0.0, 1.0).is_err());
}

mod sweeps {
    use std::f64::consts::{PI, TAU};

    use solidcraft_geom::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};
    use solidcraft_kernel_core::history::{EdgeOrigin, FaceOrigin};

    use crate::{FaceSource, KernelError, Result, extrude, extrude_whole_sides, measure, revolve};

    fn rect(w: f64, h: f64) -> Region2 {
        Region2 { outer: Loop2::polygon(&[Vec2::ZERO, Vec2::new(w, 0.0), Vec2::new(w, h), Vec2::new(0.0, h)]), holes: Vec::new() }
    }

    fn one(bodies: Vec<crate::Body>) -> Result<crate::Body> {
        bodies.into_iter().next().ok_or_else(|| KernelError::Failed("no body".into()))
    }

    #[test]
    fn a_rectangle_extrudes_to_a_box_with_sources_and_history() -> Result<()> {
        let b = one(extrude(&Plane::XY, &[rect(40.0, 30.0)], 0.0, 20.0)?)?;
        let m = measure(&b)?;
        assert!((m.volume - 24_000.0).abs() < 1e-6 && m.faces == 6, "{m:?}");
        let sources = b.face_sources().ok_or(KernelError::Failed("no sources".into()))?;
        let caps: Vec<bool> = sources
            .iter()
            .filter_map(|s| match s {
                Some(FaceSource::ExtrudeCap { high, .. }) => Some(*high),
                _ => None,
            })
            .collect();
        assert_eq!(caps.len(), 2);
        assert!(caps.contains(&true) && caps.contains(&false));
        let sides = sources.iter().filter(|s| matches!(s, Some(FaceSource::ExtrudeSide { plane: Some(_), .. }))).count();
        assert_eq!(sides, 4);
        let h = b.history().ok_or(KernelError::Failed("no history".into()))?;
        assert_eq!(h.faces.len(), 6);
        let mut segments: Vec<usize> = h
            .faces
            .iter()
            .flat_map(|f| f.origins.iter())
            .filter_map(|o| match o {
                FaceOrigin::Profile { segment } => Some(*segment),
                _ => None,
            })
            .collect();
        segments.sort_unstable();
        assert_eq!(segments, vec![0, 1, 2, 3]);
        assert_eq!(h.edges.len(), 12);
        assert!(h.edges.iter().all(|e| e.faces.len() == 2 && matches!(e.origins.as_slice(), [EdgeOrigin::Meet { .. }])), "{:?}", h.edges);
        let low = sources
            .iter()
            .find_map(|s| match s {
                Some(FaceSource::ExtrudeCap { high: false, plane, .. }) => Some(*plane),
                _ => None,
            })
            .ok_or(KernelError::Failed("low cap".into()))?;
        assert!(low.normal().z < 0.0, "the start cap's plane faces out");
        Ok(())
    }

    #[test]
    fn offsets_and_holes_are_honoured() -> Result<()> {
        let region = Region2 {
            outer: Loop2::polygon(&[Vec2::ZERO, Vec2::new(40.0, 0.0), Vec2::new(40.0, 30.0), Vec2::new(0.0, 30.0)]),
            holes: vec![Loop2::circle(Vec2::new(20.0, 15.0), 5.0)],
        };
        let b = one(extrude(&Plane::XY, &[region], 5.0, 15.0)?)?;
        let m = measure(&b)?;
        assert!((m.volume - (12_000.0 - PI * 25.0 * 10.0)).abs() < 1e-6, "{}", m.volume);
        assert!((m.bbox.min.z - 5.0).abs() < 1e-6 && (m.bbox.max.z - 15.0).abs() < 1e-6, "{:?}", m.bbox);
        assert_eq!(m.faces, 8, "4 sides, 2 half walls of the hole, 2 caps: {m:?}");
        assert_eq!(m.merged.faces, 7, "the two half walls count as one: {:?}", m.merged);
        let sources = b.face_sources().ok_or(KernelError::Failed("no sources".into()))?;
        let arcs = sources
            .iter()
            .filter(|s| matches!(s, Some(FaceSource::ExtrudeSide { segment: Seg2::Arc { .. }, plane: None, coalesced: false })))
            .count();
        assert_eq!(arcs, 2, "each arc of the circle sweeps its own half wall");
        let h = b.history().ok_or(KernelError::Failed("no history".into()))?;
        let walls: Vec<usize> = h
            .faces
            .iter()
            .flat_map(|f| f.origins.iter())
            .filter_map(|o| match o {
                FaceOrigin::Profile { segment } => Some(*segment),
                _ => None,
            })
            .filter(|s| *s >= 4)
            .collect();
        let mut walls = walls;
        walls.sort_unstable();
        assert_eq!(walls, vec![4, 5], "the hole's segments are numbered after the outer loop's");
        Ok(())
    }

    #[test]
    fn whole_sides_merge_collinear_lines() -> Result<()> {
        let region = Region2 {
            outer: Loop2::polygon(&[Vec2::ZERO, Vec2::new(20.0, 0.0), Vec2::new(40.0, 0.0), Vec2::new(40.0, 30.0), Vec2::new(0.0, 30.0)]),
            holes: Vec::new(),
        };
        let split = one(extrude(&Plane::XY, std::slice::from_ref(&region), 0.0, 10.0)?)?;
        assert_eq!(split.face_count(), 7);
        let whole = one(extrude_whole_sides(&Plane::XY, &[region], 0.0, 10.0)?)?;
        assert_eq!(whole.face_count(), 6);
        let h = whole.history().ok_or(KernelError::Failed("no history".into()))?;
        assert!(h.faces.iter().any(|f| f.origins.len() == 2), "the merged side comes from both segments: {:?}", h.faces);
        Ok(())
    }

    #[test]
    fn a_rectangle_revolves_into_a_tube() -> Result<()> {
        // In XZ, x is the radius and y (world z) the height: 5..10 by 0..3 about the sketch's y axis.
        let plane = Plane::new(Vec3::ZERO, Vec3::X, Vec3::Z).ok_or(KernelError::Failed("plane".into()))?;
        let region = Region2 {
            outer: Loop2::polygon(&[Vec2::new(5.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 3.0), Vec2::new(5.0, 3.0)]),
            holes: Vec::new(),
        };
        let full = one(revolve(&plane, std::slice::from_ref(&region), Vec2::ZERO, Vec2::new(0.0, 1.0), TAU)?)?;
        let m = measure(&full)?;
        assert!((m.volume - PI * (100.0 - 25.0) * 3.0).abs() < 1e-6, "{}", m.volume);
        assert_eq!(m.faces, 4, "{m:?}");
        let h = full.history().ok_or(KernelError::Failed("no history".into()))?;
        assert!(h.faces.iter().all(|f| matches!(f.origins.as_slice(), [FaceOrigin::Profile { .. }])), "{:?}", h.faces);
        let quarter = one(revolve(&plane, &[region], Vec2::ZERO, Vec2::new(0.0, 1.0), PI / 2.0)?)?;
        let q = measure(&quarter)?;
        assert!((q.volume - PI * 75.0 * 3.0 / 4.0).abs() < 1e-6, "{}", q.volume);
        assert_eq!(q.faces, 6);
        let caps = quarter.history().map(|h| h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Cap { .. }])).count());
        assert_eq!(caps, Some(2));
        Ok(())
    }

    #[test]
    fn a_whole_circle_as_one_arc_extrudes_to_a_cylinder() -> Result<()> {
        let region = Region2 {
            outer: Loop2 { segs: vec![Seg2::Arc { center: Vec2::new(10.0, 10.0), radius: 2.0, start: 0.0, sweep: TAU }] },
            holes: Vec::new(),
        };
        let b = one(extrude(&Plane::XY, &[region], 0.0, 5.0)?)?;
        let m = measure(&b)?;
        assert!((m.volume - PI * 4.0 * 5.0).abs() < 1e-6, "{}", m.volume);
        assert_eq!(m.faces, 3);
        Ok(())
    }

    #[test]
    fn merged_counts_fuse_pieces_of_one_surface() -> Result<()> {
        let circle = Region2 { outer: Loop2::circle(Vec2::new(10.0, 10.0), 15.0), holes: Vec::new() };
        let c = one(extrude(&Plane::XY, &[circle], 0.0, 50.0)?)?;
        let m = measure(&c)?;
        assert_eq!((m.faces, m.merged.faces), (4, 3), "{:?}", m.merged);
        assert_eq!(m.merged.edges, 2, "two rims: {:?}", m.merged);
        assert_eq!(m.merged.vertices, 0, "no vertex is left on a whole circle: {:?}", m.merged);
        Ok(())
    }

    #[test]
    fn bad_profiles_are_refused() {
        let empty = Region2 { outer: Loop2 { segs: Vec::new() }, holes: Vec::new() };
        assert!(extrude(&Plane::XY, &[empty], 0.0, 1.0).is_err());
        assert!(extrude(&Plane::XY, &[rect(1.0, 1.0)], 0.0, 0.0).is_err());
        assert!(extrude(&Plane::XY, &[rect(1.0, 1.0)], 0.0, f64::NAN).is_err());
        assert!(extrude(&Plane::XY, &[], 0.0, 1.0).is_err());
        let crossing = Region2 {
            outer: Loop2::polygon(&[Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(-1.0, 1.0)]),
            holes: Vec::new(),
        };
        assert!(revolve(&Plane::XY, &[crossing], Vec2::ZERO, Vec2::new(0.0, 1.0), TAU).is_err());
        assert!(revolve(&Plane::XY, &[rect(1.0, 1.0)], Vec2::ZERO, Vec2::ZERO, TAU).is_err());
    }
}

mod booleans {
    use std::f64::consts::PI;

    use solidcraft_geom::{Plane, Vec3};
    use solidcraft_kernel_core::history::{Change, EdgeOrigin, FaceOrigin};

    use crate::{BoolOp, FaceSource, KernelError, Result, boolean, box_solid, cylinder, half_space, measure, split_by_plane, union_apart};

    #[test]
    fn a_cut_records_kept_modified_and_tool_faces() -> Result<()> {
        let block = box_solid(Vec3::ZERO, Vec3::new(40.0, 30.0, 20.0))?;
        let bore = cylinder(Vec3::new(20.0, 15.0, -1.0), Vec3::Z, 5.0, 22.0)?;
        let cut = boolean(&block, &bore, BoolOp::Cut)?.ok_or(KernelError::Failed("empty cut".into()))?;
        let h = cut.history().ok_or(KernelError::Failed("no history".into()))?;
        assert_eq!((h.operands, h.faces.len(), h.edges.len()), (2, 7, cut.edge_count()));
        let kept =
            h.faces.iter().filter(|f| f.change == Change::Kept && matches!(f.origins.as_slice(), [FaceOrigin::Face { operand: 0, .. }])).count();
        let modified =
            h.faces.iter().filter(|f| f.change == Change::Modified && matches!(f.origins.as_slice(), [FaceOrigin::Face { operand: 0, .. }])).count();
        let from_tool = h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Face { operand: 1, .. }])).count();
        assert_eq!((kept, modified, from_tool), (4, 2, 1), "{:?}", h.faces);
        let kept_edges = h.edges.iter().filter(|e| matches!(e.origins.as_slice(), [EdgeOrigin::Edge { operand: 0, .. }])).count();
        let meets = h.edges.iter().filter(|e| matches!(e.origins.as_slice(), [EdgeOrigin::Meet { .. }])).count();
        assert_eq!((kept_edges, meets), (12, 2), "{:?}", h.edges);
        let sources = cut.face_sources().ok_or(KernelError::Failed("no sources".into()))?;
        assert_eq!(
            sources.iter().filter(|s| matches!(s, Some(FaceSource::Box { .. }))).count(),
            6,
            "the block's sources follow kept and modified faces"
        );
        Ok(())
    }

    #[test]
    fn a_union_fuses_faces_on_one_plane_and_merges_their_origins() -> Result<()> {
        let a = box_solid(Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0))?;
        let b = box_solid(Vec3::new(10.0, 0.0, 0.0), Vec3::new(20.0, 10.0, 10.0))?;
        let u = boolean(&a, &b, BoolOp::Union)?.ok_or(KernelError::Failed("empty union".into()))?;
        let m = measure(&u)?;
        assert!((m.volume - 2000.0).abs() < 1e-6 && m.faces == 6, "{m:?}");
        let h = u.history().ok_or(KernelError::Failed("no history".into()))?;
        let merged = h.faces.iter().filter(|f| f.change == Change::Merged && f.origins.len() == 2).count();
        let kept = h.faces.iter().filter(|f| f.change == Change::Kept).count();
        assert_eq!((merged, kept), (4, 2), "{:?}", h.faces);
        Ok(())
    }

    #[test]
    fn bodies_apart_keep_every_face_and_split_into_lumps_with_histories() -> Result<()> {
        let a = box_solid(Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0))?;
        let b = box_solid(Vec3::new(50.0, 0.0, 0.0), Vec3::new(60.0, 10.0, 10.0))?;
        let apart = union_apart(&[a.clone(), b.clone()])?.ok_or(KernelError::Failed("empty".into()))?;
        let h = apart.history().ok_or(KernelError::Failed("no history".into()))?;
        assert_eq!((h.operands, h.faces.len()), (2, 12));
        assert!(h.faces.iter().all(|f| f.change == Change::Kept));
        assert_eq!(h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Face { operand: 1, .. }])).count(), 6);
        let lumps = apart.lumps()?;
        assert_eq!(lumps.len(), 2);
        for lump in &lumps {
            let lh = lump.history().ok_or(KernelError::Failed("no lump history".into()))?;
            assert_eq!((lh.operands, lh.faces.len()), (1, 6));
            assert!(lump.face_sources().is_some_and(|s| s.iter().all(|s| matches!(s, Some(FaceSource::Box { .. })))));
        }
        let joined = boolean(&a, &b, BoolOp::Union)?.ok_or(KernelError::Failed("empty".into()))?;
        assert_eq!(joined.shell_count(), 2);
        Ok(())
    }

    #[test]
    fn a_half_space_cut_face_comes_from_the_tool() -> Result<()> {
        let c = cylinder(Vec3::ZERO, Vec3::Z, 5.0, 10.0)?;
        let plane = Plane::from_normal(Vec3::new(0.0, 0.0, 4.0), Vec3::Z).ok_or(KernelError::Failed("plane".into()))?;
        let (upper, tool) = half_space(&c, &plane, true)?;
        let upper = upper.ok_or(KernelError::Failed("nothing above".into()))?;
        assert!((measure(&upper)?.volume - PI * 25.0 * 6.0).abs() < 1e-6);
        assert_eq!(tool.face_count(), 1);
        let h = upper.history().ok_or(KernelError::Failed("no history".into()))?;
        assert!(h.faces.iter().any(|f| matches!(f.origins.as_slice(), [FaceOrigin::Face { operand: 1, face: 0 }])), "{:?}", h.faces);
        let pieces = split_by_plane(&c, &plane)?;
        assert_eq!(pieces.len(), 2);
        let total: f64 = pieces.iter().map(|p| measure(p).map(|m| m.volume).unwrap_or(0.0)).sum();
        assert!((total - PI * 25.0 * 10.0).abs() < 1e-6);
        Ok(())
    }
}

mod blends {
    use std::f64::consts::PI;

    use solidcraft_geom::Vec3;
    use solidcraft_kernel_core::history::{Change, FaceOrigin};

    use crate::{FaceSource, KernelError, Result, box_solid, chamfer, fillet, fillet_each, fillet_max, measure, offset_faces, shell};

    fn block() -> Result<crate::Body> {
        box_solid(Vec3::ZERO, Vec3::new(40.0, 30.0, 20.0))
    }

    #[test]
    fn a_fillet_takes_the_edge_it_rounds_as_its_origin() -> Result<()> {
        let b = block()?;
        // The edge along +X at y = 0, z = 20 (the top front edge).
        let r = fillet(&b, &[Vec3::new(20.0, 0.0, 20.0)], 5.0)?;
        let m = measure(&r)?;
        let removed = (1.0 - PI / 4.0) * 25.0 * 40.0;
        assert!((m.volume - (24_000.0 - removed)).abs() < 1e-6 && m.faces == 7, "{m:?}");
        let h = r.history().ok_or(KernelError::Failed("no history".into()))?;
        assert_eq!((h.operands, h.faces.len(), h.edges.len()), (1, 7, r.edge_count()));
        let blends: Vec<_> = h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Edge { operand: 0, .. }])).collect();
        assert_eq!(blends.len(), 1, "{:?}", h.faces);
        assert_eq!(blends[0].change, Change::Generated);
        // The two faces along the edge and the two ends it ran into are trimmed; the rest stay.
        assert_eq!(h.faces.iter().filter(|f| f.change == Change::Kept).count(), 2, "{:?}", h.faces);
        assert_eq!(h.faces.iter().filter(|f| f.change == Change::Modified).count(), 4, "{:?}", h.faces);
        let sources = r.face_sources().ok_or(KernelError::Failed("no sources".into()))?;
        assert_eq!(sources.iter().filter(|s| matches!(s, Some(FaceSource::Box { .. }))).count(), 6, "the block's sources follow its faces");
        assert!(h.edges.iter().all(|e| !e.origins.is_empty()), "every edge is placed: {:?}", h.edges);
        Ok(())
    }

    #[test]
    fn a_chamfer_cuts_the_corner_off() -> Result<()> {
        let r = chamfer(&block()?, &[Vec3::new(20.0, 0.0, 20.0)], 4.0)?;
        let m = measure(&r)?;
        assert!((m.volume - (24_000.0 - 8.0 * 40.0)).abs() < 1e-6 && m.faces == 7, "{m:?}");
        Ok(())
    }

    #[test]
    fn a_radius_too_large_is_refused_and_the_largest_that_fits_is_found() -> Result<()> {
        let b = block()?;
        let edges = [Vec3::new(20.0, 0.0, 20.0), Vec3::new(20.0, 30.0, 20.0)];
        assert!(matches!(fillet(&b, &edges, 16.0), Err(KernelError::TooLarge(_))), "two rounds of 16 mm overlap on a 30 mm face");
        let (r, body) = fillet_max(&b, &edges, 16.0).ok_or(KernelError::Failed("no fitting radius".into()))?;
        assert!(r > 10.0 && r < 16.0, "{r}");
        assert_eq!(measure(&body)?.faces, 8);
        Ok(())
    }

    #[test]
    fn fillets_of_two_radii_compose_their_history_over_the_input() -> Result<()> {
        let b = block()?;
        let r = fillet_each(&b, &[Vec3::new(20.0, 0.0, 20.0), Vec3::new(20.0, 30.0, 20.0)], &[5.0, 3.0])?;
        let m = measure(&r)?;
        let removed = (1.0 - PI / 4.0) * (25.0 + 9.0) * 40.0;
        assert!((m.volume - (24_000.0 - removed)).abs() < 1e-6 && m.faces == 8, "{m:?}");
        let h = r.history().ok_or(KernelError::Failed("no history".into()))?;
        assert_eq!(h.operands, 1);
        let blends = h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Edge { operand: 0, .. }])).count();
        assert_eq!((blends, h.faces.iter().filter(|f| f.change == Change::Kept).count()), (2, 1), "{:?}", h.faces);
        Ok(())
    }

    #[test]
    fn a_shell_lines_its_faces_with_offsets_and_keeps_the_rim_on_the_open_face() -> Result<()> {
        let b = block()?;
        let s = shell(&b, &[Vec3::new(20.0, 15.0, 20.0)], 2.0)?;
        let m = measure(&s)?;
        assert!((m.volume - (24_000.0 - 36.0 * 26.0 * 18.0)).abs() < 1e-6 && m.faces == 11, "{m:?}");
        let h = s.history().ok_or(KernelError::Failed("no history".into()))?;
        let offsets = h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Offset { operand: 0, .. }])).count();
        let rim =
            h.faces.iter().filter(|f| f.change == Change::Modified && matches!(f.origins.as_slice(), [FaceOrigin::Face { operand: 0, .. }])).count();
        let kept = h.faces.iter().filter(|f| f.change == Change::Kept).count();
        assert_eq!((offsets, rim, kept), (5, 1, 5), "{:?}", h.faces);
        let sources = s.face_sources().ok_or(KernelError::Failed("no sources".into()))?;
        let inner = sources.iter().filter(|s| matches!(s, Some(FaceSource::ShellPlanar { inner: true, .. }))).count();
        let outer = sources.iter().filter(|s| matches!(s, Some(FaceSource::Box { .. }))).count();
        assert_eq!((inner, outer), (5, 6), "{sources:?}");
        Ok(())
    }

    #[test]
    fn offsetting_a_face_moves_it_and_its_source_plane() -> Result<()> {
        let b = block()?;
        let r = offset_faces(&b, &[Vec3::new(20.0, 15.0, 20.0)], 5.0)?;
        let m = measure(&r)?;
        assert!((m.volume - 30_000.0).abs() < 1e-6 && m.faces == 6, "{m:?}");
        let top =
            (0..6).find(|&i| r.face_plane(i).is_some_and(|p| (p.origin.z - 25.0).abs() < 1e-9)).ok_or(KernelError::Failed("no top at 25".into()))?;
        assert!(matches!(r.face_sources().and_then(|s| s.get(top)), Some(Some(FaceSource::Box { .. }))));
        Ok(())
    }
}

mod paths {
    use std::f64::consts::{PI, TAU};

    use solidcraft_geom::{Loop2, Plane, Region2, Vec2, Vec3};
    use solidcraft_kernel_core::history::FaceOrigin;

    use crate::{KernelError, PathSeg, Result, loft, measure, sweep, sweep_helix, sweep_path};

    fn square(s: f64) -> Region2 {
        let h = s * 0.5;
        Region2 { outer: Loop2::polygon(&[Vec2::new(-h, -h), Vec2::new(h, -h), Vec2::new(h, h), Vec2::new(-h, h)]), holes: Vec::new() }
    }

    #[test]
    fn a_sweep_along_a_line_is_the_extrude() -> Result<()> {
        let b = sweep(&Plane::XY, &square(10.0), &[PathSeg::Line { a: Vec3::ZERO, b: Vec3::new(0.0, 0.0, 20.0) }])?;
        let m = measure(&b)?;
        assert!((m.volume - 2000.0).abs() < 1e-6 && m.faces == 6, "{m:?}");
        let h = b.history().ok_or(KernelError::Failed("no history".into()))?;
        let caps = h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Cap { .. }])).count();
        let sides = h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Profile { .. }])).count();
        assert_eq!((caps, sides), (2, 4), "{:?}", h.faces);
        Ok(())
    }

    #[test]
    fn a_sweep_round_a_quarter_arc_follows_pappus() -> Result<()> {
        let path = [PathSeg::Arc { a: Vec3::ZERO, center: Vec3::new(30.0, 0.0, 0.0), axis: Vec3::Y, angle: PI / 2.0 }];
        let b = sweep(&Plane::XY, &square(10.0), &path)?;
        let m = measure(&b)?;
        assert!((m.volume - 100.0 * 30.0 * PI / 2.0).abs() < 1e-3 * 4712.0 && m.faces == 6, "{m:?}");
        let h = b.history().ok_or(KernelError::Failed("no history".into()))?;
        let caps = h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Cap { .. }])).count();
        assert_eq!(caps, 2, "{:?}", h.faces);
        Ok(())
    }

    #[test]
    fn a_polyline_sweep_with_a_corner_and_a_helix_build() -> Result<()> {
        let bent = sweep_path(&Plane::XY, &square(4.0), &[Vec3::ZERO, Vec3::new(0.0, 0.0, 20.0), Vec3::new(20.0, 0.0, 20.0)])?;
        let m = measure(&bent)?;
        assert!(m.volume > 16.0 * 36.0 && m.volume < 16.0 * 44.0, "{m:?}");
        // A 4 x 4 section in the XZ plane, 20 mm from the Z axis, two turns of pitch 10.
        let plane = Plane { origin: Vec3::new(20.0, 0.0, 0.0), x: Vec3::X, y: Vec3::Z };
        let coil = sweep_helix(&plane, &square(4.0), Vec3::ZERO, Vec3::Z, 10.0, 2.0)?;
        let m = measure(&coil)?;
        let pappus = 16.0 * 20.0 * TAU * 2.0;
        assert!((m.volume - pappus).abs() < 2e-2 * pappus, "{m:?} vs {pappus}");
        Ok(())
    }

    #[test]
    fn a_loft_between_squares_is_a_frustum() -> Result<()> {
        let top = Plane { origin: Vec3::new(0.0, 0.0, 30.0), x: Vec3::X, y: Vec3::Y };
        let b = loft(&[(Plane::XY, square(20.0).outer), (top, square(10.0).outer)])?;
        let m = measure(&b)?;
        assert!((m.volume - 7000.0).abs() < 1e-6 && m.faces == 6, "{m:?}");
        let h = b.history().ok_or(KernelError::Failed("no history".into()))?;
        let caps = h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Cap { .. }])).count();
        let sides = h.faces.iter().filter(|f| matches!(f.origins.as_slice(), [FaceOrigin::Profile { .. }])).count();
        assert_eq!((caps, sides), (2, 4), "{:?}", h.faces);
        Ok(())
    }
}

mod native {
    use solidcraft_geom::Vec3;

    use crate::{BoolOp, FaceSource, KernelError, Result, boolean, box_solid, cylinder, decode_native_body, encode_native_body, measure};

    #[test]
    fn a_body_with_sources_and_history_survives_the_native_cache_exactly() -> Result<()> {
        let block = box_solid(Vec3::ZERO, Vec3::new(40.0, 30.0, 20.0))?;
        let bore = cylinder(Vec3::new(20.0, 15.0, -1.0), Vec3::Z, 5.0, 22.0)?;
        let cut = boolean(&block, &bore, BoolOp::Cut)?.ok_or(KernelError::Failed("empty cut".into()))?.with_color(Some([0.2, 0.4, 0.6]));
        let bytes = encode_native_body(&cut)?;
        assert!(bytes.starts_with(b"SCBREP05"));
        let back = decode_native_body(&bytes)?;
        let (a, b) = (measure(&cut)?, measure(&back)?);
        assert_eq!((a.volume, a.area, a.faces, a.edges), (b.volume, b.area, b.faces, b.edges));
        assert_eq!(back.color(), Some([0.2, 0.4, 0.6]));
        assert_eq!(back.history(), cut.history());
        assert_eq!(back.face_sources().map(|s| s.iter().filter(|s| matches!(s, Some(FaceSource::Box { .. }))).count()), Some(6));
        assert!((back.size() - cut.size()).abs() < 1e-12);
        // The B-rep bytes carry whatever triangulation the shape holds, so a re-encoding
        // need not repeat the bytes; the decoded geometry does.
        let again = decode_native_body(&encode_native_body(&back)?)?;
        assert_eq!(measure(&again)?.volume, a.volume);
        assert!(decode_native_body(b"SCBREP04junk").is_err());
        assert!(decode_native_body(&bytes[..bytes.len() / 2]).is_err());
        Ok(())
    }
}

mod exchange {
    use solidcraft_geom::Vec3;
    use solidcraft_kernel_core::p21;

    use crate::{
        BoolOp, ExportBody, ExportProduct, FacePaint, KernelError, Paint, Result, StepHeader, boolean, box_solid, cylinder, step_export,
        step_export_products,
    };

    #[test]
    fn bodies_export_as_named_coloured_solids_in_one_part() -> Result<()> {
        let block = box_solid(Vec3::ZERO, Vec3::new(40.0, 30.0, 20.0))?;
        let bore = cylinder(Vec3::new(20.0, 15.0, -1.0), Vec3::Z, 5.0, 22.0)?;
        let cut = boolean(&block, &bore, BoolOp::Cut)?.ok_or(KernelError::Failed("empty cut".into()))?;
        let painted = cut
            .with_color(Some([1.0, 0.0, 0.0]))
            .with_paint(Some(Paint { opacity: 1.0, faces: vec![FacePaint { face: 0, color: [0.0, 1.0, 0.0], opacity: 1.0 }] }));
        let text = step_export(&[&painted, &block], "test")?;
        assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 2, "{text}");
        assert!(text.contains("MANIFOLD_SOLID_BREP('Body1'") && text.contains("MANIFOLD_SOLID_BREP('Body2'"), "{text}");
        assert_eq!(text.matches("STYLED_ITEM(").count(), 2, "the body colour and one face colour: {text}");
        assert!(text.contains("AP242_MANAGED_MODEL_BASED_3D_ENGINEERING"));
        let ex = p21::parse(&text).map_err(KernelError::Failed)?;
        assert!(ex.duplicates.is_empty());
        let faces = ex.entities.values().filter(|e| e.name() == "ADVANCED_FACE").count();
        assert_eq!(faces, 7 + 6, "{faces}");
        Ok(())
    }

    #[test]
    fn an_assembly_exports_its_occurrences_with_placements() -> Result<()> {
        let part = box_solid(Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0))?;
        let mut m = [[0.0; 4]; 4];
        for (i, row) in m.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        m[3] = [50.0, 0.0, 0.0, 1.0];
        let products = vec![
            ExportProduct {
                name: "Assembly".into(),
                bodies: Vec::new(),
                children: vec![
                    (1, m, "Cube:1".into()),
                    (1, [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 50.0, 0.0, 1.0]], String::new()),
                ],
            },
            ExportProduct { name: "Cube".into(), bodies: vec![ExportBody { name: "Cube".into(), body: &part, color: None }], children: Vec::new() },
        ];
        let text = step_export_products(&products, 0, &StepHeader::default())?;
        assert_eq!(text.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE(").count(), 2, "{text}");
        assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 1);
        assert!(text.contains("ITEM_DEFINED_TRANSFORMATION"));
        Ok(())
    }
}

mod import {
    use solidcraft_geom::Vec3;

    use crate::{
        BoolOp, ExportBody, ExportProduct, FacePaint, FaceSource, KernelError, Paint, Result, StepHeader, boolean, box_solid, cylinder, measure,
        step_export, step_export_products, step_import, step_validate,
    };

    #[test]
    fn named_coloured_bodies_round_trip_through_step() -> Result<()> {
        let block = box_solid(Vec3::ZERO, Vec3::new(40.0, 30.0, 20.0))?;
        let bore = cylinder(Vec3::new(20.0, 15.0, -1.0), Vec3::Z, 5.0, 22.0)?;
        let cut = boolean(&block, &bore, BoolOp::Cut)?.ok_or(KernelError::Failed("empty cut".into()))?;
        let painted = cut
            .with_color(Some([1.0, 0.0, 0.0]))
            .with_paint(Some(Paint { opacity: 1.0, faces: vec![FacePaint { face: 2, color: [0.0, 1.0, 0.0], opacity: 1.0 }] }));
        let text = step_export(&[&painted, &block], "test")?;
        assert!(step_validate(&text).is_ok(), "{:?}", step_validate(&text));
        let imp = step_import(&text)?;
        assert!(imp.warnings.is_empty(), "{:?}", imp.warnings);
        assert_eq!(imp.bodies.len(), 2);
        let names: Vec<&str> = imp.bodies.iter().map(|b| b.name.as_str()).collect();
        assert_eq!(names, ["Body1", "Body2"]);
        let a = &imp.bodies[0];
        let m = measure(&a.body)?;
        let want = measure(&painted)?;
        assert!((m.volume - want.volume).abs() < 1e-6 * want.volume && m.faces == want.faces, "{m:?} vs {want:?}");
        assert_eq!(a.color, Some([1.0, 0.0, 0.0]));
        assert!(a.closed && a.file_faces == 7 && a.face_origin.len() == 7, "{} {}", a.file_faces, a.face_origin.len());
        let sources = a.body.face_sources().ok_or(KernelError::Failed("no sources".into()))?;
        assert!(sources.iter().all(|s| matches!(s, Some(FaceSource::Imported { .. }))));
        let paint = a.body.paint().ok_or(KernelError::Failed("no paint".into()))?;
        assert_eq!(paint.faces.len(), 1, "one face keeps its own colour: {paint:?}");
        assert_eq!(imp.tree.len(), 1);
        assert_eq!(imp.tree[0].bodies, vec![0, 1]);
        Ok(())
    }

    #[test]
    fn an_assembly_round_trips_with_its_placements_and_instances() -> Result<()> {
        let part = box_solid(Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0))?;
        let shift = |x: f64, y: f64| [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [x, y, 0.0, 1.0]];
        let products = vec![
            ExportProduct {
                name: "Assembly".into(),
                bodies: Vec::new(),
                children: vec![(1, shift(50.0, 0.0), "Cube:1".into()), (1, shift(0.0, 50.0), String::new())],
            },
            ExportProduct { name: "Cube".into(), bodies: vec![ExportBody { name: "Cube".into(), body: &part, color: None }], children: Vec::new() },
        ];
        let text = step_export_products(&products, 0, &StepHeader::default())?;
        let imp = step_import(&text)?;
        assert!(imp.warnings.is_empty(), "{:?}", imp.warnings);
        assert_eq!(imp.bodies.len(), 2, "one body per instance");
        let centres: Vec<Vec3> = imp.bodies.iter().map(|b| measure(&b.body).map(|m| m.centroid)).collect::<Result<_>>()?;
        assert!(centres.iter().any(|c| (*c - Vec3::new(55.0, 5.0, 5.0)).len() < 1e-6), "{centres:?}");
        assert!(centres.iter().any(|c| (*c - Vec3::new(5.0, 55.0, 5.0)).len() < 1e-6), "{centres:?}");
        assert_eq!(imp.bodies[0].path, vec!["Assembly".to_string(), "Cube".to_string()]);
        assert_eq!(imp.bodies[0].instance.as_deref(), Some("Cube:1"));
        assert_eq!(imp.tree.len(), 1);
        assert_eq!(imp.tree[0].children.len(), 2);
        assert!(imp.bodies[1].body.display_share().is_some(), "the second instance shows the first's mesh placed");
        Ok(())
    }
}

mod conformance {
    use solidcraft_geom::Vec3;

    use crate::{measure, step_import};

    fn rel(a: f64, b: f64) -> f64 {
        (a - b).abs() / b.abs().max(1e-12)
    }

    /// Parts of known shape from the step-conformance corpus (`tests/step-conformance`,
    /// generated from formulas by https://github.com/dan335/step-conformance at 96f30bb, CC0
    /// 1.0): each reads back with its exact volume, its box and its face count (merged: a
    /// cylinder's wall is one face here, two pieces on the truck backend).
    #[test]
    fn step_conformance_parts_read_back_their_exact_shape() {
        // (file, exact volume (mm³), box (min, max), merged faces, bodies)
        #[rustfmt::skip]
        let parts: [(&str, &str, f64, [f64; 6], usize, usize); 16] = [
            ("cone_point", include_str!("../../../tests/step-conformance/cone_point.step"), 1000.0 * std::f64::consts::PI, [-10.0, -10.0, 0.0, 10.0, 10.0, 30.0], 2, 1),
            ("cone_point_vertex", include_str!("../../../tests/step-conformance/cone_point_vertex.step"), 1000.0 * std::f64::consts::PI, [-10.0, -10.0, 0.0, 10.0, 10.0, 30.0], 2, 1),
            ("cylinder", include_str!("../../../tests/step-conformance/cylinder.step"), 9424.778, [-10.0, -10.0, 0.0, 10.0, 10.0, 30.0], 3, 1),
            ("cylinder_inch", include_str!("../../../tests/step-conformance/cylinder_inch.step"), 9424.778, [-10.0, -10.0, 0.0, 10.0, 10.0, 30.0], 3, 1),
            ("cylinder_nurbs", include_str!("../../../tests/step-conformance/cylinder_nurbs.step"), 9424.778, [-10.0, -10.0, 0.0, 10.0, 10.0, 30.0], 3, 1),
            ("cylinder_placed", include_str!("../../../tests/step-conformance/cylinder_placed.step"), 9424.778, [31.835, -33.165, 1.835, 65.485, 0.485, 35.485], 3, 1),
            ("cylinder_seam", include_str!("../../../tests/step-conformance/cylinder_seam.step"), 9424.778, [-10.0, -10.0, 0.0, 10.0, 10.0, 30.0], 3, 1),
            ("cylinder_twice", include_str!("../../../tests/step-conformance/cylinder_twice.step"), 18849.556, [-10.0, -10.0, 0.0, 90.0, 10.0, 30.0], 6, 2),
            ("dome", include_str!("../../../tests/step-conformance/dome.step"), 16755.161, [-20.0, -20.0, 0.0, 20.0, 20.0, 20.0], 2, 1),
            ("dome_nurbs", include_str!("../../../tests/step-conformance/dome_nurbs.step"), 16755.161, [-20.0, -20.0, 0.0, 20.0, 20.0, 20.0], 2, 1),
            ("far_cylinder", include_str!("../../../tests/step-conformance/far_cylinder.step"), 9424.778, [99990.0, -10.0, 0.0, 100010.0, 10.0, 30.0], 3, 1),
            ("grooved_shaft", include_str!("../../../tests/step-conformance/grooved_shaft.step"), 153532.251, [-10.0, -10.0, 0.0, 10.0, 10.0, 501.0], 753, 1),
            ("plate_100_holes", include_str!("../../../tests/step-conformance/plate_100_holes.step"), 71588.321, [0.0, 0.0, 0.0, 100.0, 100.0, 10.0], 110, 1),
            ("sheet_disc", include_str!("../../../tests/step-conformance/sheet_disc.step"), 193396443.755, [-1800.0, -1800.0, 0.0, 1800.0, 1800.0, 19.0], 3, 1),
            ("tiny_pin", include_str!("../../../tests/step-conformance/tiny_pin.step"), std::f64::consts::FRAC_PI_8, [-0.25, -0.25, 0.0, 0.25, 0.25, 2.0], 3, 1),
            ("washer", include_str!("../../../tests/step-conformance/washer.step"), 3534.292, [-20.0, -20.0, 0.0, 20.0, 20.0, 3.0], 4, 1),
        ];
        let mut failures = Vec::new();
        for (name, text, volume, bbox, faces, bodies) in parts {
            let imp = match step_import(text) {
                Ok(imp) => imp,
                Err(e) => {
                    failures.push(format!("{name}: {e}"));
                    continue;
                }
            };
            if !imp.warnings.is_empty() {
                failures.push(format!("{name}: warnings {:?}", imp.warnings));
            }
            if imp.bodies.len() != bodies {
                failures.push(format!("{name}: {} bodies, want {bodies}", imp.bodies.len()));
            }
            let (mut v, mut n) = (0.0, 0);
            let mut b = solidcraft_geom::Aabb3::EMPTY;
            for body in &imp.bodies {
                let Ok(m) = measure(&body.body) else {
                    failures.push(format!("{name}: a body cannot be measured"));
                    continue;
                };
                v += m.volume;
                n += m.merged.faces;
                b = b.union(&m.bbox);
            }
            if rel(v, volume) >= 1e-3 {
                failures.push(format!("{name}: volume {v} vs {volume}"));
            }
            if n != faces {
                failures.push(format!("{name}: {n} faces, want {faces}"));
            }
            let [x0, y0, z0, x1, y1, z1] = bbox;
            let size = Vec3::new(x1 - x0, y1 - y0, z1 - z0).len();
            for (got, want) in [(b.min, Vec3::new(x0, y0, z0)), (b.max, Vec3::new(x1, y1, z1))] {
                if got.dist(want) > size * 1e-3 {
                    failures.push(format!("{name}: box corner {got:?} vs {want:?}"));
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }
}

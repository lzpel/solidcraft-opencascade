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
    let e = crate::fillet(&b, &[Vec3::ZERO], 0.1);
    assert!(matches!(&e, Err(KernelError::TooLarge(m)) if m.contains("OpenCASCADE")), "{e:?}");
    assert!(crate::extrude(&solidcraft_geom::Plane::XY, &[], 0.0, 1.0).is_err());
}

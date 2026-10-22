//! Building bodies: primitives now; extrude, revolve, loft and sweep in phases 3 and 5.

use std::sync::Arc;

use solidcraft_geom::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};

use crate::body::{Body, dv, v3};
use crate::types::{PathSeg, RailFit, SweepControl};
use crate::{FaceSource, KernelError, Result, unsupported};

/// Largest coordinate or size a primitive takes (mm).
const MAX_COORD: f64 = 1e9;

fn finite(v: Vec3) -> bool {
    v.is_finite() && v.x.abs() < MAX_COORD && v.y.abs() < MAX_COORD && v.z.abs() < MAX_COORD
}

/// A box with opposite corners `a` and `b`. Its faces carry `FaceSource::Box` (axis and side).
pub fn box_solid(a: Vec3, b: Vec3) -> Result<Body> {
    if !(finite(a) && finite(b)) {
        return Err(KernelError::Invalid("box corners must be finite".into()));
    }
    let lo = Vec3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z));
    let hi = Vec3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z));
    let d = hi - lo;
    if !(d.x > 1e-9 && d.y > 1e-9 && d.z > 1e-9) {
        return Err(KernelError::Invalid("box with a zero side".into()));
    }
    let solid = crate::guard("box", || Ok(cadrum::Solid::cube(dv(lo), dv(hi))))?;
    let body = Body::from_solids(vec![solid])?;
    let sources = planar_sources(&body, |n, center| {
        let axis = if n.x.abs() >= n.y.abs() && n.x.abs() >= n.z.abs() {
            0
        } else if n.y.abs() >= n.z.abs() {
            1
        } else {
            2
        };
        let high = [n.x, n.y, n.z][axis] > 0.0;
        Plane::from_normal(center, n).map(|plane| FaceSource::Box { producer: None, axis: axis as u8, high, plane })
    })?;
    Ok(body.with_face_sources(Some(Arc::new(sources))))
}

/// A cylinder from `base` along `axis`. Its caps carry extrude-cap sources (the cylinder
/// command certifies them), its wall an extrude side of the full circle.
pub fn cylinder(base: Vec3, axis: Vec3, radius: f64, height: f64) -> Result<Body> {
    if !(finite(base) && finite(axis) && radius.is_finite() && height.is_finite()) {
        return Err(KernelError::Invalid("cylinder parameters must be finite".into()));
    }
    let Some(dir) = axis.normalized() else { return Err(KernelError::Invalid("cylinder axis must not be zero".into())) };
    if !(radius > 1e-9 && radius < MAX_COORD && height > 1e-9 && height < MAX_COORD) {
        return Err(KernelError::Invalid("cylinder radius and height must be positive".into()));
    }
    let solid = crate::guard("cylinder", || Ok(cadrum::Solid::cylinder(radius, dv(dir * height)).translate(dv(base))))?;
    let body = Body::from_solids(vec![solid])?;
    let circle = Region2 { outer: Loop2::circle(Vec2::ZERO, radius), holes: Vec::new() };
    let mut sources: Vec<Option<FaceSource>> = planar_sources(&body, |n, center| {
        let high = n.dot(dir) > 0.0;
        Plane::from_normal(center, n).map(|plane| FaceSource::ExtrudeCap { high, region: circle.clone(), plane })
    })?;
    for s in sources.iter_mut() {
        if s.is_none() {
            *s = Some(FaceSource::ExtrudeSide {
                segment: Seg2::Arc { center: Vec2::ZERO, radius, start: 0.0, sweep: std::f64::consts::TAU },
                plane: None,
                coalesced: true,
            });
        }
    }
    Ok(body.with_face_sources(Some(Arc::new(sources))))
}

pub fn sphere(center: Vec3, radius: f64) -> Result<Body> {
    if !(finite(center) && radius.is_finite() && radius > 1e-9 && radius < MAX_COORD) {
        return Err(KernelError::Invalid("sphere centre must be finite and its radius positive".into()));
    }
    let solid = crate::guard("sphere", || Ok(cadrum::Solid::sphere(radius).translate(dv(center))))?;
    Body::from_solids(vec![solid])
}

pub fn torus(center: Vec3, major: f64, minor: f64) -> Result<Body> {
    if !(finite(center) && major.is_finite() && minor.is_finite() && minor > 1e-9 && major > minor && major < MAX_COORD) {
        return Err(KernelError::Invalid("torus radii must be finite, the minor positive and smaller than the major".into()));
    }
    let solid = crate::guard("torus", || Ok(cadrum::Solid::torus(major, minor, cadrum::DVec3::Z).translate(dv(center))))?;
    Body::from_solids(vec![solid])
}

/// A source for each planar face of `body` from its outward normal and centre (`None` for the
/// others).
fn planar_sources(body: &Body, mut source: impl FnMut(Vec3, Vec3) -> Option<FaceSource>) -> Result<Vec<Option<FaceSource>>> {
    let inner = body.inner()?;
    inner.with("face sources", |solids| {
        Ok((0..inner.faces.len())
            .map(|i| {
                let face = inner.face(solids, i)?;
                let s = face.surface()?;
                if s.kind != cadrum::SurfaceKind::Plane {
                    return None;
                }
                let center = v3(face.center());
                let n = v3(face.project(dv(center)).1).normalized()?;
                source(n, center)
            })
            .collect())
    })
}

pub fn extrude(_plane: &Plane, _regions: &[Region2], _start: f64, _end: f64) -> Result<Vec<Body>> {
    Err(unsupported("extrude"))
}

pub fn extrude_whole_sides(plane: &Plane, regions: &[Region2], start: f64, end: f64) -> Result<Vec<Body>> {
    extrude(plane, regions, start, end)
}

pub fn extrude_planar_regions(_plane: &Plane, _regions: &[Region2], _start: f64, _end: f64) -> Result<Body> {
    Err(unsupported("extrude"))
}

pub fn extrude_tapered(_plane: &Plane, _region: &Region2, _length: f64, _dir_sign: f64, _taper: f64) -> Result<Body> {
    Err(unsupported("tapered extrude (draft)"))
}

pub fn extrude_tapered_between(_plane: &Plane, _region: &Region2, _from: f64, _to: f64, _foot: f64, _dir_sign: f64, _taper: f64) -> Result<Body> {
    Err(unsupported("tapered extrude (draft)"))
}

/// The revolve axis in space, from the sketch plane: the same on every backend.
pub fn canonical_revolve_axis(plane: &Plane, _region: &Region2, axis_origin: Vec2, axis_dir: Vec2) -> Result<Vec3> {
    let d = plane.to_world(axis_origin + axis_dir) - plane.to_world(axis_origin);
    d.normalized().ok_or_else(|| KernelError::Invalid("revolve axis must not be zero".into()))
}

pub fn revolve(_plane: &Plane, _regions: &[Region2], _axis_origin: Vec2, _axis_dir: Vec2, _angle: f64) -> Result<Vec<Body>> {
    Err(unsupported("revolve"))
}

pub fn revolve_whole(plane: &Plane, regions: &[Region2], axis_origin: Vec2, axis_dir: Vec2, angle: f64) -> Result<Vec<Body>> {
    revolve(plane, regions, axis_origin, axis_dir, angle)
}

pub fn loft(_sections: &[(Plane, Loop2)]) -> Result<Body> {
    Err(unsupported("loft"))
}

pub fn loft_with_rails(_sections: &[(Plane, Loop2)], _rails: &[Vec<Vec3>], _fit: RailFit) -> Result<Body> {
    Err(unsupported("loft with guide rails"))
}

pub fn loft_regions(_sections: &[(Plane, Region2)]) -> Result<Body> {
    Err(unsupported("loft"))
}

pub fn loft_to_point(_plane: &Plane, _lp: &Loop2, _apex: Vec3) -> Result<Body> {
    Err(unsupported("loft to a point"))
}

/// The points along a path, segment ends and arcs divided.
pub fn path_polyline(path: &[PathSeg]) -> Result<Vec<Vec3>> {
    let mut out = Vec::new();
    for (i, seg) in path.iter().enumerate() {
        match *seg {
            PathSeg::Line { a, b } => {
                if !(finite(a) && finite(b)) {
                    return Err(KernelError::Invalid("path points must be finite".into()));
                }
                if i == 0 {
                    out.push(a);
                }
                out.push(b);
            }
            PathSeg::Arc { a, center, axis, angle } => {
                if !(finite(a) && finite(center) && finite(axis) && angle.is_finite()) {
                    return Err(KernelError::Invalid("path arcs must be finite".into()));
                }
                let Some(k) = axis.normalized() else { return Err(KernelError::Invalid("path arc axis must not be zero".into())) };
                if i == 0 {
                    out.push(a);
                }
                let n = ((angle.abs() / 0.1).ceil() as usize).clamp(1, 512);
                for s in 1..=n {
                    out.push(rotate_about(a, center, k, angle * s as f64 / n as f64));
                }
            }
        }
    }
    Ok(out)
}

fn rotate_about(p: Vec3, center: Vec3, k: Vec3, angle: f64) -> Vec3 {
    let v = p - center;
    let (s, c) = angle.sin_cos();
    center + v * c + k.cross(v) * s + k * (k.dot(v) * (1.0 - c))
}

pub fn sweep(_plane: &Plane, _region: &Region2, _path: &[PathSeg]) -> Result<Body> {
    Err(unsupported("sweep"))
}

pub fn sweep_with(_plane: &Plane, _region: &Region2, _path: &[PathSeg], _ctl: &SweepControl) -> Result<Body> {
    Err(unsupported("sweep"))
}

pub fn sweep_path(_plane: &Plane, _region: &Region2, _path: &[Vec3]) -> Result<Body> {
    Err(unsupported("sweep along a polyline"))
}

pub fn sweep_path_with(_plane: &Plane, _region: &Region2, _path: &[Vec3], _ctl: &SweepControl) -> Result<Body> {
    Err(unsupported("sweep along a polyline"))
}

pub fn sweep_helix(_plane: &Plane, _region: &Region2, _axis_origin: Vec3, _axis: Vec3, _pitch: f64, _turns: f64) -> Result<Body> {
    Err(unsupported("helical sweep"))
}

pub fn sweep_helix_mesh(
    _plane: &Plane,
    _polygon: &[Vec2],
    _axis_origin: Vec3,
    _axis: Vec3,
    _pitch: f64,
    _turns: f64,
    _stations: usize,
) -> Result<solidcraft_geom::Mesh> {
    Err(unsupported("helical sweep"))
}

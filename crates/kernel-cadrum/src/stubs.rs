//! Operations not on this backend yet. Each keeps its signature and answers "not supported
//! yet: … on the OpenCASCADE backend"; the phase that implements it moves it to its module.

use solidcraft_geom::{Mesh, Plane, Region2, Seg2, Vec2, Vec3};

use crate::body::Body;
use crate::types::*;
use crate::{Result, unsupported};

// ---- blends (phase 5: constant-radius fillets and chamfers; the rest need cadrum work) ----

pub fn fillet(_body: &Body, _edges: &[Vec3], _radius: f64) -> Result<Body> {
    Err(unsupported("fillet"))
}

pub fn fillet_each(_body: &Body, _edges: &[Vec3], _radii: &[f64]) -> Result<Body> {
    Err(unsupported("fillet"))
}

pub fn fillet_g2(_body: &Body, _edges: &[Vec3], _radius: f64) -> Result<Body> {
    Err(unsupported("a G2 fillet"))
}

pub fn fillet_g2_corners(_body: &Body, _edges: &[Vec3], _radius: f64) -> Result<Body> {
    Err(unsupported("a G2 fillet"))
}

pub fn fillet_max(_body: &Body, _edges: &[Vec3], _radius: f64) -> Option<(f64, Body)> {
    None
}

pub fn fillet_max_each(_body: &Body, _edges: &[Vec3], _radius: f64) -> Option<(Vec<f64>, Body)> {
    None
}

pub fn fillet_radii(_body: &Body, _edges: &[(Vec3, f64)]) -> Result<Body> {
    Err(unsupported("fillets of several radii"))
}

pub fn fillet_chord(_body: &Body, _edges: &[Vec3], _chord: f64) -> Result<Body> {
    Err(unsupported("a chord fillet"))
}

pub fn fillet_tapered(_body: &Body, _edges: &[Vec3], _radius: f64) -> Result<(Body, Vec<Option<Taper>>)> {
    Err(unsupported("a tapered fillet"))
}

pub fn fillet_variable(_body: &Body, _edge_at: Vec3, _start: Vec3, _r0: f64, _r1: f64) -> Result<Body> {
    Err(unsupported("a variable-radius fillet"))
}

pub fn fillet_variable_points(_body: &Body, _edge_at: Vec3, _start: Vec3, _points: &[(f64, f64)], _smooth: bool) -> Result<Body> {
    Err(unsupported("a variable-radius fillet"))
}

pub fn fillet_variable_spline(_body: &Body, _edge_at: Vec3, _start: Vec3, _spline: &RadiusSpline) -> Result<Body> {
    Err(unsupported("a variable-radius fillet"))
}

pub fn taper(_body: &Body, _edge_at: Vec3, _radius: f64) -> Result<Taper> {
    Err(unsupported("a tapered fillet"))
}

pub fn chamfer(_body: &Body, _edges: &[Vec3], _distance: f64) -> Result<Body> {
    Err(unsupported("chamfer"))
}

pub fn chamfer_sides(_body: &Body, _edges: &[Vec3], _d1: f64, _side: ChamferSide, _flip: bool) -> Result<Body> {
    Err(unsupported("an asymmetric chamfer"))
}

// ---- shell, offset, draft, split, sew, delete (phases 5 and 7) ----

pub fn shell(_b: &Body, _open: &[Vec3], _thickness: f64) -> Result<Body> {
    Err(unsupported("shell"))
}

pub fn offset_faces(_b: &Body, _at: &[Vec3], _distance: f64) -> Result<Body> {
    Err(unsupported("offsetting faces"))
}

pub fn draft(_b: &Body, _faces: &[Vec3], _neutral: &Plane, _pull: Vec3, _angle: f64) -> Result<Body> {
    Err(unsupported("draft"))
}

pub fn convex_polyhedron(_hs: &[HalfSpace]) -> Result<Body> {
    Err(unsupported("a convex polyhedron"))
}

pub fn split_body(_body: &Body, _tool: &SplitTool) -> Result<Vec<Body>> {
    Err(unsupported("split body"))
}

pub fn split_faces(_body: &Body, _faces: &[usize], _tool: &SplitTool) -> Result<Body> {
    Err(unsupported("split face"))
}

pub fn split_faces_with_map(_body: &Body, _faces: &[usize], _tool: &SplitTool) -> Result<(Body, Vec<usize>)> {
    Err(unsupported("split face"))
}

pub fn sew(_faces: &[FaceSpec]) -> Result<Body> {
    Err(unsupported("sewing faces"))
}

pub fn sew_touching(_faces: &[FaceSpec]) -> Result<Body> {
    Err(unsupported("sewing faces"))
}

pub fn delete_faces(_body: &Body, _faces: &[usize]) -> Result<Body> {
    Err(unsupported("delete face"))
}

// ---- emboss, lip, whole-face cuts, distances (need more of cadrum) ----

pub fn cylinder_emboss_anchor(_body: &Body, _face: usize, _at: Vec3) -> Result<Vec3> {
    Err(unsupported("emboss on a cylinder"))
}

pub fn emboss_cylinder(_body: &Body, _plane: &Plane, _regions: &[Region2], _depth: f64) -> Result<Body> {
    Err(unsupported("emboss on a cylinder"))
}

pub fn emboss_cylinder_at(_body: &Body, _at: Vec3, _plane: &Plane, _regions: &[Region2], _depth: f64) -> Result<Body> {
    Err(unsupported("emboss on a cylinder"))
}

pub fn has_cylinder_emboss_wall(_body: &Body) -> Result<bool> {
    Ok(false)
}

pub fn infer_cylinder_emboss(_body: &Body, _plane: &Plane, _regions: &[Region2], _depth: f64) -> Result<Option<(usize, Vec3)>> {
    Ok(None)
}

pub fn raise_band(_body: &Body, _at: Vec3, _width: f64, _height: f64, _outside: bool) -> Result<Body> {
    Err(unsupported("a lip"))
}

pub fn rim_band(_body: &Body, _at: Vec3, _width: f64, _free: f64, _outside: bool) -> Result<(Plane, Region2)> {
    Err(unsupported("a lip"))
}

pub fn cut_takes_only_the_face(_target: &Body, _face: WholeFace, _depth: f64) -> bool {
    false
}

pub fn cut_tool(_target: &Body, _face: WholeFace, _depth: f64, _built: Option<&Body>) -> Option<(Body, f64)> {
    None
}

pub fn face_prism(_target: &Body, _face: WholeFace, _length: f64, _outward: bool, _built: &Body) -> Result<Body> {
    Err(unsupported("a face prism"))
}

pub fn matching_face(_target: &Body, _plane: &Plane, _region: &Region2, _heights: &[f64]) -> Option<WholeFace> {
    None
}

// ---- the surface workspace (needs open shells and NURBS access in cadrum) ----

pub fn blend_between(
    _a: &Body,
    _picks_a: &[Vec3],
    _b: &Body,
    _picks_b: &[Vec3],
    _orders: [Continuity; 2],
    _crown: [f64; 2],
    _align: Option<[Vec3; 2]>,
    _seam: Option<[Vec3; 2]>,
    _samples: usize,
    _stitch: bool,
) -> Result<(Body, f64)> {
    Err(unsupported("a surface blend"))
}

pub fn bridge(_a: &[Frame], _b: &[Frame], _orders: [Continuity; 2], _crown: [f64; 2]) -> Result<Body> {
    Err(unsupported("a surface bridge"))
}

pub fn sample_grid(_body: &Body, _along: usize, _across: usize) -> Result<Vec<Vec<Vec3>>> {
    Err(unsupported("sampling a surface"))
}

pub fn pick_edges(_body: &Body, _picks: &[EdgePick]) -> Result<Vec<usize>> {
    Err(unsupported("continuity analysis"))
}

pub fn seam_diagnostics(_body: &Body, _edges: &[usize], _samples: usize, _tolerances: ContinuityTolerances) -> Result<ContinuityReport> {
    Err(unsupported("continuity analysis"))
}

pub fn fair_faces(_body: &Body, _faces: &[usize], _opts: &FairOptions) -> Result<(Body, FairReport)> {
    Err(unsupported("fairing"))
}

pub fn patch_edges_crowned(_body: &Body, _at: &[Vec3], _crown: f64) -> Result<Body> {
    Err(unsupported("a patch"))
}

pub fn patch_edges_report(_body: &Body, _at: &[Vec3], _spec: &PatchSpec) -> Result<(Body, PatchReport)> {
    Err(unsupported("a patch"))
}

pub fn patch_edges_with(_body: &Body, _at: &[Vec3], _spec: &PatchSpec) -> Result<Body> {
    Err(unsupported("a patch"))
}

pub fn match_surface(_body: &Body, _faces: &[Vec3], _target: &Body, _target_pick: &[Vec3], _order: Continuity) -> Result<Body> {
    Err(unsupported("matching surfaces"))
}

pub fn seam_continuity(_made: &Body, _target: &Body, _target_pick: &[Vec3]) -> Result<(f64, f64, Continuity)> {
    Err(unsupported("matching surfaces"))
}

pub fn extend_surface(_body: &Body, _at: &[Vec3], _opts: &ExtendOptions) -> Result<Body> {
    Err(unsupported("extending a surface"))
}

pub fn offset_surface(_body: &Body, _faces: &[usize], _distance: f64, _tolerance: f64) -> Result<Body> {
    Err(unsupported("offsetting a surface"))
}

pub fn ruled_surface(_body: &Body, _at: &[Vec3], _r: &Ruled) -> Result<Body> {
    Err(unsupported("a ruled surface"))
}

pub fn mutual_trim(_a: &Body, _b: &Body, _keep_a: TrimKeep, _keep_b: TrimKeep, _extend: bool) -> Result<(Body, Body)> {
    Err(unsupported("trimming surfaces"))
}

pub fn trim(_body: &Body, _tool: &SplitTool, _keep: Vec3) -> Result<Body> {
    Err(unsupported("trimming a surface"))
}

pub fn trim_keep(_body: &Body, _tool: &SplitTool, _keep: TrimKeep) -> Result<Body> {
    Err(unsupported("trimming a surface"))
}

pub fn untrim(_body: &Body, _face: usize, _holes: UntrimHoles, _outer: bool, _in_place: bool) -> Result<Body> {
    Err(unsupported("untrimming a surface"))
}

pub fn boundary_patch(_boundaries: &[Boundary], _crown: f64, _samples: usize) -> Result<Body> {
    Err(unsupported("a boundary surface"))
}

pub fn boundary_join(_target: &Body, _made: &Body) -> Result<Body> {
    Err(unsupported("a boundary surface"))
}

pub fn boundary_surface(_u: &[BoundaryCurve], _v: &[BoundaryCurve]) -> Result<Body> {
    Err(unsupported("a boundary surface"))
}

pub fn copy_faces(_body: &Body, _at: &[Vec3]) -> Result<Body> {
    Err(unsupported("copying faces"))
}

pub fn extend(_body: &Body, _at: &[Vec3], _distance: f64) -> Result<Body> {
    Err(unsupported("extending a surface"))
}

pub fn extrude_sheet(_plane: &Plane, _chains: &[Vec<Seg2>], _start: f64, _end: f64) -> Result<Body> {
    Err(unsupported("a surface extrude"))
}

pub fn loft_sheet(_sections: &[(Plane, Vec<Seg2>)]) -> Result<Body> {
    Err(unsupported("a surface loft"))
}

pub fn loft_sheet_with_rails(_sections: &[(Plane, Vec<Seg2>)], _rails: &[Vec<Vec3>], _fit: RailFit) -> Result<Body> {
    Err(unsupported("a surface loft"))
}

pub fn patch_edges(_body: &Body, _at: &[Vec3]) -> Result<Body> {
    Err(unsupported("a patch"))
}

pub fn patch_region(_plane: &Plane, _region: &Region2) -> Result<Body> {
    Err(unsupported("a patch"))
}

pub fn remove_faces(_body: &Body, _faces: &[usize]) -> Result<Body> {
    Err(unsupported("removing faces"))
}

pub fn reverse_faces(_body: &Body, _at: &[Vec3]) -> Result<Body> {
    Err(unsupported("reversing faces"))
}

pub fn revolve_sheet(_plane: &Plane, _chains: &[Vec<Seg2>], _axis_origin: Vec2, _axis_dir: Vec2, _start: f64, _sweep: f64) -> Result<Body> {
    Err(unsupported("a surface revolve"))
}

pub fn stitch(_bodies: &[&Body], _tol: f64) -> Result<Body> {
    Err(unsupported("stitching surfaces"))
}

pub fn sweep_sheet(_plane: &Plane, _chains: &[Vec<Seg2>], _path: &[PathSeg]) -> Result<Body> {
    Err(unsupported("a surface sweep"))
}

pub fn sweep_sheet_points(_plane: &Plane, _chains: &[Vec<Seg2>], _path: &[Vec3]) -> Result<Body> {
    Err(unsupported("a surface sweep"))
}

pub fn sweep_sheet_points_with(_plane: &Plane, _chains: &[Vec<Seg2>], _path: &[Vec3], _ctl: &SweepControl) -> Result<Body> {
    Err(unsupported("a surface sweep"))
}

pub fn sweep_sheet_with(_plane: &Plane, _chains: &[Vec<Seg2>], _path: &[PathSeg], _ctl: &SweepControl) -> Result<Body> {
    Err(unsupported("a surface sweep"))
}

pub fn thicken(_body: &Body, _thickness: f64, _symmetric: bool) -> Result<Body> {
    Err(unsupported("thickening a surface"))
}

pub fn unstitch(_body: &Body, _at: &[Vec3]) -> Result<(Vec<Body>, Option<Body>)> {
    Err(unsupported("unstitching"))
}

/// Keeps `Mesh` in scope for signatures that will take one.
#[allow(dead_code)]
type MeshAlias = Mesh;

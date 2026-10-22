//! The body's metadata to and from the versioned wire records: the conversions are shared with
//! the other backend (`solidcraft_kernel_core::native_meta`); this reads and sets the fields.
use super::{Body, Result, invalid};
use solidcraft_geom::brep::envelope as w;
use solidcraft_kernel_core::native_meta::{self as meta, Fields};

pub(super) fn encode(body: &Body) -> Result<w::Metadata> {
    meta::encode(&Fields {
        color: body.color,
        paint: body.paint.as_deref().cloned(),
        splits: body.splits.as_deref().cloned(),
        face_sources: body.face_sources.as_deref().cloned(),
        history: body.history.as_deref().cloned(),
        preserve_boundaries: body.regularization_protected,
    })
}

pub(super) fn apply(body: &mut Body, saved: w::Metadata) -> Result<()> {
    if saved.version != w::VERSION {
        return Err(invalid("metadata version"));
    }
    let f = meta::decode(saved)?;
    body.color = f.color;
    body.paint = f.paint.map(std::sync::Arc::new);
    body.splits = f.splits.map(std::sync::Arc::new);
    body.face_sources = f.face_sources.map(std::sync::Arc::new);
    body.history = f.history.map(std::sync::Arc::new);
    body.regularization_protected = f.preserve_boundaries;
    Ok(())
}

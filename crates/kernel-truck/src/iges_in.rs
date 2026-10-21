//! IGES import for this backend: the text stage (`solidcraft_kernel_core::iges_in`, an IGES file
//! restated as STEP) followed by this backend's STEP reader.

pub use solidcraft_kernel_core::iges_in::{MAX_IGES_BYTES, iges_to_step};

/// Read IGES text: its solids, or its surfaces sewn into shells, as the STEP reader gives them.
pub fn iges_import(text: &str) -> crate::Result<crate::StepImport> {
    let (step, warnings) = iges_to_step(text).map_err(crate::KernelError::Invalid)?;
    let mut imp = crate::step_import(&step)?;
    imp.warnings.splice(0..0, warnings);
    imp.schema = "IGES".into();
    Ok(imp)
}

#[cfg(test)]
mod tests;

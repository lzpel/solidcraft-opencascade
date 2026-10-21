//! The kernel's error type and the panic guard every call into a third-party kernel runs under.

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum KernelError {
    #[error("invalid input: {0}")]
    Invalid(String),
    #[error("the operation failed: {0}")]
    Failed(String),
    /// A blend or other size too large for the geometry around it (a smaller size may work);
    /// shown like any feature that isn't supported yet.
    #[error("the operation failed: not supported yet: {0}")]
    TooLarge(String),
    /// The third-party kernel panicked; the model is unchanged.
    #[error("kernel internal error: {0}")]
    Internal(String),
}

pub type Result<T> = std::result::Result<T, KernelError>;

thread_local! {
    static GUARDED: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
}

/// Is this thread inside [`guard`]? (A panic now will be caught and turned into an error; the
/// engine's crash reports mark it recovered.)
pub fn guarded() -> bool {
    GUARDED.with(std::cell::Cell::get) > 0
}

/// Run third-party kernel code, converting a panic into an error.
pub fn guard<T>(what: &str, f: impl FnOnce() -> Result<T>) -> Result<T> {
    struct Depth;
    impl Drop for Depth {
        fn drop(&mut self) {
            GUARDED.with(|g| g.set(g.get().saturating_sub(1)));
        }
    }
    GUARDED.with(|g| g.set(g.get() + 1));
    let depth = Depth;
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f));
    drop(depth);
    match r {
        Ok(r) => r,
        Err(p) => {
            let msg = p.downcast_ref::<&str>().map(|s| s.to_string()).or_else(|| p.downcast_ref::<String>().cloned()).unwrap_or_default();
            log::warn!("kernel panic in {what}: {msg}");
            Err(KernelError::Internal(format!("{what}: {msg}")))
        }
    }
}

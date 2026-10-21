//! A small seeded random generator for the robustness checks (`solidcraft-cli fuzz`,
//! docs/robustness.md); the checks themselves live in each backend's `fuzz` module.

/// xorshift64*: small, fast, and the same everywhere (wasm included).
#[derive(Clone, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        // Spread nearby seeds apart (splitmix64), never zero.
        let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        Rng((z ^ (z >> 31)) | 1)
    }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// 0…n−1 (0 when n is 0).
    pub fn below(&mut self, n: usize) -> usize {
        if n == 0 { 0 } else { (self.next_u64() % n as u64) as usize }
    }
    /// In [0, 1).
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn chance(&mut self, p: f64) -> bool {
        self.unit() < p
    }
    pub fn pick<T: Copy>(&mut self, xs: &[T]) -> Option<T> {
        xs.get(self.below(xs.len())).copied()
    }
    /// A multiple of `step` in [lo, hi].
    pub fn grid(&mut self, lo: f64, hi: f64, step: f64) -> f64 {
        let n = ((hi - lo) / step).floor().max(0.0) as usize;
        lo + step * self.below(n + 1) as f64
    }
}

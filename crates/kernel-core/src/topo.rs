//! Topology counts comparable with kernels that use periodic faces and closed edges (the
//! computation lives in each backend).

use std::collections::BTreeMap;

use serde::Serialize;

#[derive(Clone, Debug, Default, Serialize, PartialEq)]
pub struct TopoCounts {
    pub faces: usize,
    pub edges: usize,
    pub vertices: usize,
    pub face_types: BTreeMap<String, usize>,
}

use std::collections::HashMap;

use glam::DVec3;

/// Merges vertices that are within 1mm of each other by snapping to a 1mm grid.
/// Returns (deduplicated vertex list, index mapping from old → new).
pub struct VertexMerger {
    map: HashMap<(i64, i64, i64), u32>,
    pub verts: Vec<DVec3>,
}

impl VertexMerger {
    pub fn new() -> Self {
        Self {
            map: HashMap::new(),
            verts: Vec::new(),
        }
    }

    /// Insert a vertex and return its canonical index (creates new entry if unseen).
    pub fn insert(&mut self, v: DVec3) -> u32 {
        let key = snap(v);
        if let Some(&idx) = self.map.get(&key) {
            return idx;
        }
        let idx = self.verts.len() as u32;
        self.verts.push(v);
        self.map.insert(key, idx);
        idx
    }

}

/// Snap a vertex to 1mm grid for deduplication.
fn snap(v: DVec3) -> (i64, i64, i64) {
    (
        (v.x * 1000.0).round() as i64,
        (v.y * 1000.0).round() as i64,
        (v.z * 1000.0).round() as i64,
    )
}

use std::collections::HashMap;

/// Warn if the mesh has boundary edges or over-shared edges (non-manifold indicators).
pub fn check_manifold(building_id: &str, tris: &[[u32; 3]]) {
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in tris {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_count.entry(key).or_insert(0) += 1;
        }
    }
    let boundary: usize = edge_count.values().filter(|&&c| c == 1).count();
    let nonmanifold: usize = edge_count.values().filter(|&&c| c > 2).count();
    if boundary > 0 || nonmanifold > 0 {
        eprintln!(
            "WARN: building '{}' has {} boundary edge(s), {} edges with >2 faces",
            building_id, boundary, nonmanifold
        );
    }
}

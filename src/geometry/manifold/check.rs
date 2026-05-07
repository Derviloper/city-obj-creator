use std::collections::HashMap;

use glam::DVec3;

/// Warn if the mesh has boundary edges or over-shared edges (non-manifold indicators).
pub(super) fn check_manifold(building_id: &str, tris: &[[u32; 3]]) {
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

/// Dump the world-space endpoints of every boundary edge for a building.
/// Helpful for debugging non-manifold meshes.
pub fn dump_boundary_edges(building_id: &str, verts: &[DVec3], tris: &[[u32; 3]]) {
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in tris {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_count.entry(key).or_insert(0) += 1;
        }
    }
    let mut boundary_edges: Vec<(u32, u32)> = edge_count
        .iter()
        .filter(|&(_, &c)| c == 1)
        .map(|(k, _)| *k)
        .collect();
    boundary_edges.sort();
    eprintln!("=== boundary edges for {} ({}) ===", building_id, boundary_edges.len());
    for (a, b) in &boundary_edges {
        let va = verts[*a as usize];
        let vb = verts[*b as usize];
        eprintln!(
            "  edge {a}->{b}  ({:.4},{:.4},{:.4}) → ({:.4},{:.4},{:.4})  len={:.4}",
            va.x, va.y, va.z, vb.x, vb.y, vb.z,
            (vb - va).length()
        );
    }
    let nonmanifold: Vec<(u32, u32, usize)> = edge_count
        .iter()
        .filter(|&(_, &c)| c > 2)
        .map(|(k, c)| (k.0, k.1, *c))
        .collect();
    if !nonmanifold.is_empty() {
        eprintln!("=== over-shared edges for {} ===", building_id);
        for (a, b, c) in &nonmanifold {
            let va = verts[*a as usize];
            let vb = verts[*b as usize];
            eprintln!(
                "  edge {a}->{b} (count {}): ({:.4},{:.4},{:.4}) → ({:.4},{:.4},{:.4})",
                c, va.x, va.y, va.z, vb.x, vb.y, vb.z
            );
        }
    }
}

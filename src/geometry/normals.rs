use glam::DVec3;

/// Compute the outward-facing normal of a polygon via Newell's method.
/// Newell's method is numerically robust for arbitrary planar polygons.
pub fn polygon_normal(verts: &[DVec3]) -> DVec3 {
    if verts.len() < 3 {
        return DVec3::Z;
    }
    let mut n = DVec3::ZERO;
    let len = verts.len();
    for i in 0..len {
        let cur = verts[i];
        let nxt = verts[(i + 1) % len];
        n.x += (cur.y - nxt.y) * (cur.z + nxt.z);
        n.y += (cur.z - nxt.z) * (cur.x + nxt.x);
        n.z += (cur.x - nxt.x) * (cur.y + nxt.y);
    }
    let len_sq = n.length_squared();
    if len_sq > 1e-20 {
        n / len_sq.sqrt()
    } else {
        DVec3::Z
    }
}

/// Fan-triangulate a convex polygon into triangles given vertex indices.
/// Emits triangles (i0, i, i+1) for i in 1..n-1.
pub fn fan_triangulate(indices: &[u32]) -> Vec<[u32; 3]> {
    if indices.len() < 3 {
        return Vec::new();
    }
    let i0 = indices[0];
    (1..indices.len() - 1)
        .map(|i| [i0, indices[i], indices[i + 1]])
        .collect()
}

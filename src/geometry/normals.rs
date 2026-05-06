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

/// Triangulate a (possibly non-convex, possibly non-planar) polygon by
/// projecting it onto its best-fit plane and running ear-clipping in 2D.
/// Returns triangles as triples of indices into the input slice. If the
/// polygon is too degenerate to ear-clip cleanly, falls back to a fan.
pub fn ear_clip_triangulate(verts: &[glam::DVec3]) -> Vec<[usize; 3]> {
    let n = verts.len();
    if n < 3 {
        return Vec::new();
    }
    if n == 3 {
        return vec![[0, 1, 2]];
    }

    // Project to the polygon's best-fit plane.
    let normal = polygon_normal(verts);
    let pick = if normal.x.abs() < 0.9 { glam::DVec3::X } else { glam::DVec3::Y };
    let u = (pick - normal * normal.dot(pick)).normalize();
    let v = normal.cross(u).normalize();
    let p2d: Vec<(f64, f64)> = verts.iter().map(|p| (p.dot(u), p.dot(v))).collect();

    // Signed area picks winding.
    let area2: f64 = (0..n)
        .map(|i| {
            let (x1, y1) = p2d[i];
            let (x2, y2) = p2d[(i + 1) % n];
            x1 * y2 - x2 * y1
        })
        .sum();
    let ccw = area2 > 0.0;

    // Doubly-linked list of remaining vertex indices.
    let mut prev: Vec<usize> = (0..n).map(|i| (i + n - 1) % n).collect();
    let mut next: Vec<usize> = (0..n).map(|i| (i + 1) % n).collect();
    let mut tris: Vec<[usize; 3]> = Vec::with_capacity(n - 2);
    let mut remaining = n;
    let mut cursor = 0;
    let mut stalled = 0;

    while remaining > 3 {
        if stalled > remaining * 2 {
            // No progress; bail out and let caller fall back.
            return Vec::new();
        }
        let i = cursor;
        let p = prev[i];
        let q = next[i];
        let pa = p2d[p];
        let pb = p2d[i];
        let pc = p2d[q];

        let cross = (pb.0 - pa.0) * (pc.1 - pa.1) - (pb.1 - pa.1) * (pc.0 - pa.0);
        let convex = if ccw { cross > 0.0 } else { cross < 0.0 };

        let mut is_ear = convex;
        if is_ear {
            let mut k = next[q];
            while k != p {
                if point_in_tri(p2d[k], pa, pb, pc) {
                    is_ear = false;
                    break;
                }
                k = next[k];
            }
        }

        if is_ear {
            tris.push([p, i, q]);
            next[p] = q;
            prev[q] = p;
            remaining -= 1;
            cursor = q;
            stalled = 0;
        } else {
            cursor = next[i];
            stalled += 1;
        }
    }

    // Last remaining triangle.
    let i = cursor;
    tris.push([prev[i], i, next[i]]);
    tris
}

fn point_in_tri(pt: (f64, f64), a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
    let sign = |p1: (f64, f64), p2: (f64, f64), p3: (f64, f64)| -> f64 {
        (p1.0 - p3.0) * (p2.1 - p3.1) - (p2.0 - p3.0) * (p1.1 - p3.1)
    };
    let d1 = sign(pt, a, b);
    let d2 = sign(pt, b, c);
    let d3 = sign(pt, c, a);
    let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_neg && has_pos)
}

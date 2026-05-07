use std::collections::HashMap;

use glam::{DVec2, DVec3};

use crate::geometry::normals::ear_clip_triangulate;
use crate::geometry::vertex_merge::VertexMerger;
use crate::model::tile::BBox2;

/// Tolerance (metres) for classifying a vertex as on-plane vs off-plane.
const PLANE_EPS: f64 = 1e-6;

/// Axis along which a clipping plane is aligned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Axis {
    X,
    Y,
}

/// Clip a 2D polygon to an axis-aligned bounding box using Sutherland-Hodgman.
/// Walks the polygon against each of the four bbox half-planes in turn. The
/// returned ring keeps the input's winding and has no closing duplicate; an
/// empty result means the polygon is fully outside the bbox.
pub fn clip_polygon_to_aabb_xy_2d(ring: &[DVec2], bbox: BBox2) -> Vec<DVec2> {
    if ring.len() < 3 {
        return Vec::new();
    }
    let mut out = ring.to_vec();
    out = clip_to_edge(&out, |p| p.x >= bbox.xmin, |a, b| intersect_x(a, b, bbox.xmin));
    out = clip_to_edge(&out, |p| p.x <= bbox.xmax, |a, b| intersect_x(a, b, bbox.xmax));
    out = clip_to_edge(&out, |p| p.y >= bbox.ymin, |a, b| intersect_y(a, b, bbox.ymin));
    out = clip_to_edge(&out, |p| p.y <= bbox.ymax, |a, b| intersect_y(a, b, bbox.ymax));
    // Sutherland-Hodgman on a non-convex polygon can emit consecutive
    // duplicate vertices where the boundary touches a clip edge tangentially.
    // Drop them so downstream side-wall meshing stays manifold.
    let eps2 = 1e-18;
    let mut deduped: Vec<DVec2> = Vec::with_capacity(out.len());
    for p in out {
        if let Some(prev) = deduped.last()
            && (*prev - p).length_squared() < eps2
        {
            continue;
        }
        deduped.push(p);
    }
    if deduped.len() >= 2 {
        let first = deduped[0];
        let last = *deduped.last().unwrap();
        if (first - last).length_squared() < eps2 {
            deduped.pop();
        }
    }
    deduped
}

fn clip_to_edge(
    poly: &[DVec2],
    inside: impl Fn(DVec2) -> bool,
    intersect: impl Fn(DVec2, DVec2) -> DVec2,
) -> Vec<DVec2> {
    let n = poly.len();
    if n == 0 {
        return Vec::new();
    }
    let mut out: Vec<DVec2> = Vec::with_capacity(n + 4);
    for i in 0..n {
        let a = poly[(i + n - 1) % n];
        let b = poly[i];
        let a_in = inside(a);
        let b_in = inside(b);
        if b_in {
            if !a_in {
                out.push(intersect(a, b));
            }
            out.push(b);
        } else if a_in {
            out.push(intersect(a, b));
        }
    }
    out
}

fn intersect_x(a: DVec2, b: DVec2, x: f64) -> DVec2 {
    let dx = b.x - a.x;
    let t = if dx.abs() < f64::EPSILON { 0.0 } else { (x - a.x) / dx };
    DVec2::new(x, a.y + t * (b.y - a.y))
}

fn intersect_y(a: DVec2, b: DVec2, y: f64) -> DVec2 {
    let dy = b.y - a.y;
    let t = if dy.abs() < f64::EPSILON { 0.0 } else { (y - a.y) / dy };
    DVec2::new(a.x + t * (b.x - a.x), y)
}

/// Clip a closed manifold mesh to a tile's XY bounding box, patching the
/// cut faces with cap polygons so each result piece remains watertight.
///
/// Applied as four sequential half-space clips: x ≥ xmin, x ≤ xmax,
/// y ≥ ymin, y ≤ ymax. Each clip preserves manifoldness by emitting cap
/// triangles along the cut plane. Vertex deduplication after each pass
/// merges coincident cap vertices contributed by adjacent triangles.
pub fn clip_mesh_to_aabb_xy(
    verts: &[DVec3],
    tris: &[[u32; 3]],
    bbox: BBox2,
) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    let (mut v, mut t) = (verts.to_vec(), tris.to_vec());
    (v, t) = clip_halfspace(&v, &t, Axis::X, bbox.xmin, false); // keep x >= xmin
    (v, t) = clip_halfspace(&v, &t, Axis::X, bbox.xmax, true); // keep x <= xmax
    (v, t) = clip_halfspace(&v, &t, Axis::Y, bbox.ymin, false); // keep y >= ymin
    (v, t) = clip_halfspace(&v, &t, Axis::Y, bbox.ymax, true); // keep y <= ymax
    (v, t)
}

/// Clip a mesh against one half-space `coord <= value` (when `keep_neg`)
/// or `coord >= value` (when not). Returns the kept fragment with cap
/// triangles closing the cut hole.
fn clip_halfspace(
    verts: &[DVec3],
    tris: &[[u32; 3]],
    axis: Axis,
    value: f64,
    keep_neg: bool,
) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    if tris.is_empty() {
        return (Vec::new(), Vec::new());
    }

    // Pre-compute signed distance and inside flag for every input vertex.
    let dist: Vec<f64> = verts.iter().map(|v| coord_of(*v, axis) - value).collect();
    let inside: Vec<bool> = dist
        .iter()
        .map(|&d| if keep_neg { d <= PLANE_EPS } else { d >= -PLANE_EPS })
        .collect();

    let mut merger = VertexMerger::new();
    let mut out_tris: Vec<[u32; 3]> = Vec::new();
    // Directed cap-polygon edges (start -> end). Each triangle contributes at
    // most one such edge along the cut plane.
    let mut cap_edges: Vec<(u32, u32)> = Vec::new();

    for tri in tris {
        let ia = tri[0] as usize;
        let ib = tri[1] as usize;
        let ic = tri[2] as usize;
        let in_a = inside[ia];
        let in_b = inside[ib];
        let in_c = inside[ic];
        let n_in = (in_a as u8) + (in_b as u8) + (in_c as u8);

        if n_in == 0 {
            continue;
        }
        if n_in == 3 {
            let a = merger.insert(verts[ia]);
            let b = merger.insert(verts[ib]);
            let c = merger.insert(verts[ic]);
            push_tri(&mut out_tris, a, b, c);
            continue;
        }

        // Build the inside-fragment polygon by walking edges in winding
        // order. Classify each cut as "leaving" (in→out) or "entering"
        // (out→in). Cap polygon edge runs from entering cut to leaving
        // cut — that's the reverse of the inside-fragment's traversal
        // along the cap, so each cap edge is paired with its
        // wall-fragment counterpart in opposite winding.
        let edges = [(ia, ib), (ib, ic), (ic, ia)];
        let mut poly_idx: Vec<u32> = Vec::with_capacity(4);
        let mut entering: Option<u32> = None;
        let mut leaving: Option<u32> = None;

        for &(u, w) in &edges {
            if inside[u] {
                poly_idx.push(merger.insert(verts[u]));
            }
            if inside[u] != inside[w] {
                let p = intersect_axis(verts[u], verts[w], dist[u], dist[w], axis, value);
                let idx = merger.insert(p);
                if inside[u] {
                    leaving = Some(idx);
                } else {
                    entering = Some(idx);
                }
                poly_idx.push(idx);
            }
        }

        // Triangulate the inside fragment as a fan from poly_idx[0]. The
        // fragment is convex (a triangle clipped by one plane is a tri or
        // quad), so a fan is correct.
        for i in 1..poly_idx.len().saturating_sub(1) {
            push_tri(
                &mut out_tris,
                poly_idx[0],
                poly_idx[i],
                poly_idx[i + 1],
            );
        }

        if let (Some(e), Some(l)) = (entering, leaving)
            && e != l
        {
            cap_edges.push((e, l));
        }
    }

    // Stitch cap edges into closed loops, triangulate each loop, append.
    if !cap_edges.is_empty() {
        let cap_tris = triangulate_cap(&merger.verts, &cap_edges, axis, keep_neg);
        out_tris.extend(cap_tris);
    }

    (merger.verts, out_tris)
}

fn coord_of(v: DVec3, axis: Axis) -> f64 {
    match axis {
        Axis::X => v.x,
        Axis::Y => v.y,
    }
}

/// Intersect segment `a→b` with the plane `coord(axis) = value`. The
/// returned point's axis coordinate is forced exactly to `value` so cap
/// vertices land on the plane to floating-point precision.
fn intersect_axis(a: DVec3, b: DVec3, da: f64, db: f64, axis: Axis, value: f64) -> DVec3 {
    let denom = da - db;
    let t = if denom.abs() < f64::EPSILON {
        0.5
    } else {
        (da / denom).clamp(0.0, 1.0)
    };
    let p = a + (b - a) * t;
    match axis {
        Axis::X => DVec3::new(value, p.y, p.z),
        Axis::Y => DVec3::new(p.x, value, p.z),
    }
}

fn push_tri(tris: &mut Vec<[u32; 3]>, a: u32, b: u32, c: u32) {
    if a == b || b == c || a == c {
        return;
    }
    tris.push([a, b, c]);
}

/// Walk directed cap edges into closed loops, then triangulate each loop
/// in the cut plane. The loop's triangulation is oriented so its outward
/// normal points away from the kept solid.
fn triangulate_cap(
    verts: &[DVec3],
    cap_edges: &[(u32, u32)],
    axis: Axis,
    keep_neg: bool,
) -> Vec<[u32; 3]> {
    // Adjacency: start vertex -> list of end vertices (usually one, but
    // multiple are possible when two disjoint loops share a corner).
    let mut next_map: HashMap<u32, Vec<u32>> = HashMap::new();
    for &(a, b) in cap_edges {
        if a != b {
            next_map.entry(a).or_default().push(b);
        }
    }

    let mut tris: Vec<[u32; 3]> = Vec::new();

    // Walk loops by repeatedly picking any starting edge and following
    // until we return to the start, consuming edges as we go.
    while let Some((&start, _)) = next_map.iter().find(|(_, v)| !v.is_empty()) {
        let mut loop_verts = Vec::new();
        let mut current = start;
        let mut steps = 0usize;
        let max_steps = cap_edges.len() + 4;
        loop {
            loop_verts.push(current);
            let nexts = match next_map.get_mut(&current) {
                Some(n) if !n.is_empty() => n,
                _ => break,
            };
            let nxt = nexts.remove(0);
            current = nxt;
            steps += 1;
            if current == start || steps > max_steps {
                break;
            }
        }

        if loop_verts.len() < 3 {
            continue;
        }

        // Project to 2D in the cut plane and check signed area to make
        // sure the loop is wound such that outward normal points outside
        // the kept solid.
        let pts2d: Vec<(f64, f64)> = loop_verts
            .iter()
            .map(|&i| project_to_cap(verts[i as usize], axis))
            .collect();
        let signed_area_2x: f64 = (0..pts2d.len())
            .map(|i| {
                let (x1, y1) = pts2d[i];
                let (x2, y2) = pts2d[(i + 1) % pts2d.len()];
                x1 * y2 - x2 * y1
            })
            .sum();
        let want_ccw = expected_ccw(axis, keep_neg);
        let is_ccw = signed_area_2x > 0.0;
        let mut ordered = loop_verts;
        if is_ccw != want_ccw {
            ordered.reverse();
        }

        let verts_3d: Vec<DVec3> = ordered.iter().map(|&i| verts[i as usize]).collect();
        let local_tris = ear_clip_triangulate(&verts_3d);
        if local_tris.is_empty() {
            // Fan fallback for nearly-degenerate loops.
            for i in 1..ordered.len() - 1 {
                push_tri(&mut tris, ordered[0], ordered[i], ordered[i + 1]);
            }
        } else {
            for t in local_tris {
                push_tri(
                    &mut tris,
                    ordered[t[0]],
                    ordered[t[1]],
                    ordered[t[2]],
                );
            }
        }
    }

    tris
}

/// Project a 3D point onto the cut plane's 2D coordinates. The chosen
/// 2D basis is the natural right-handed one looking along +axis (so that
/// CCW in 2D corresponds to outward normal = +axis).
fn project_to_cap(v: DVec3, axis: Axis) -> (f64, f64) {
    match axis {
        // Looking along +X (toward +X), Y goes right, Z goes up.
        Axis::X => (v.y, v.z),
        // Looking along +Y, Z goes right, X goes up. So right-hand basis
        // for outward = +Y is (Z, X).
        Axis::Y => (v.z, v.x),
    }
}

/// Whether a CCW loop in our 2D projection corresponds to the cap's
/// desired outward direction (away from the kept solid).
fn expected_ccw(axis: Axis, keep_neg: bool) -> bool {
    // Cap outward direction: when keep_neg, the kept solid is on the
    // -axis side, so outward = +axis. project_to_cap is built so CCW in
    // 2D corresponds to +axis normal — hence:
    //   keep_neg = true  -> want CCW
    //   keep_neg = false -> want CW
    let _ = axis;
    keep_neg
}

use std::collections::{HashMap, HashSet};

use glam::DVec3;

use super::helpers::{snap_key, GridKey};
use crate::{
    geometry::normals::ear_clip_triangulate,
    model::building::SurfaceType,
};

/// Merge clusters of near-duplicate vertices across all polygons.
///
/// Builds union-find groups over every distinct vertex: two vertices are
/// joined when their xy distance is within `xy_tol` AND their z distance is
/// within `z_tol`. Each group is then collapsed to its centroid in every
/// polygon's vertex list. After collapsing, consecutive duplicates within a
/// polygon (introduced by the merge) are removed.
pub(super) fn cluster_merge_close_vertices(
    polygons: &mut [(SurfaceType, Vec<DVec3>)],
    base_tol: f64,
    xy_tol: f64,
    z_tol: f64,
) {
    // Collect distinct vertices via the snap grid we already used.
    let mut key_to_idx: HashMap<GridKey, usize> = HashMap::new();
    let mut verts: Vec<DVec3> = Vec::new();
    for (_, vs) in polygons.iter() {
        for v in vs {
            let k = snap_key(*v, base_tol);
            if let std::collections::hash_map::Entry::Vacant(entry) = key_to_idx.entry(k) {
                entry.insert(verts.len());
                verts.push(*v);
            }
        }
    }
    let n = verts.len();
    if n < 2 {
        return;
    }

    // Bucket by an xy_tol grid for O(n) average pair search.
    let cell_xy = xy_tol;
    let cell_of = |v: DVec3| -> (i64, i64) {
        ((v.x / cell_xy).floor() as i64, (v.y / cell_xy).floor() as i64)
    };
    let mut buckets: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, v) in verts.iter().enumerate() {
        buckets.entry(cell_of(*v)).or_default().push(i);
    }

    // Union-find.
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], i: usize) -> usize {
        let mut r = i;
        while parent[r] != r {
            r = parent[r];
        }
        let mut c = i;
        while parent[c] != c {
            let next = parent[c];
            parent[c] = r;
            c = next;
        }
        r
    }
    let xy_tol_sq = xy_tol * xy_tol;
    for i in 0..n {
        let vi = verts[i];
        let (cx, cy) = cell_of(vi);
        for dx in -1..=1 {
            for dy in -1..=1 {
                let Some(others) = buckets.get(&(cx + dx, cy + dy)) else {
                    continue;
                };
                for &j in others {
                    if j <= i {
                        continue;
                    }
                    let vj = verts[j];
                    let dxv = vi.x - vj.x;
                    let dyv = vi.y - vj.y;
                    let dzv = (vi.z - vj.z).abs();
                    if dzv <= z_tol && dxv * dxv + dyv * dyv <= xy_tol_sq {
                        let ri = find(&mut parent, i);
                        let rj = find(&mut parent, j);
                        if ri != rj {
                            parent[ri] = rj;
                        }
                    }
                }
            }
        }
    }

    // Compute per-cluster centroids.
    let mut sums: HashMap<usize, (DVec3, usize)> = HashMap::new();
    for (i, v) in verts.iter().enumerate() {
        let r = find(&mut parent, i);
        let entry = sums.entry(r).or_insert((DVec3::ZERO, 0));
        entry.0 += *v;
        entry.1 += 1;
    }
    let centroid_for = |i: usize, parent: &mut [usize]| -> DVec3 {
        let r = find(parent, i);
        let (s, c) = sums[&r];
        s / c as f64
    };

    // Rewrite polygon vertex lists, collapsing consecutive duplicates.
    for (_, vs) in polygons.iter_mut() {
        let mapped: Vec<DVec3> = vs
            .iter()
            .map(|v| {
                let i = *key_to_idx.get(&snap_key(*v, base_tol)).expect("vertex in index");
                centroid_for(i, &mut parent)
            })
            .collect();
        let mut deduped: Vec<DVec3> = Vec::with_capacity(mapped.len());
        for v in mapped {
            if deduped.last().is_none_or(|last: &DVec3| {
                (*last - v).length_squared() > 1e-12
            }) {
                deduped.push(v);
            }
        }
        // Drop closing duplicate if present.
        if deduped.len() >= 2
            && (deduped[0] - *deduped.last().unwrap()).length_squared() < 1e-12
        {
            deduped.pop();
        }
        *vs = deduped;
    }
}

/// Drop ground polygons that represent internal floors of multi-story
/// buildings (where LVermGeo data sometimes mistags upper-floor slabs as
/// `GroundSurface`).
///
/// A ground polygon A is treated as internal — and dropped — only when some
/// other ground polygon B has A's xy centroid inside its 2D projection *and*
/// A sits at least 1 m above B in Z. This avoids dropping legitimate sloped
/// or stepped ground polygons whose Z varies but which never project onto
/// each other in xy.
pub(super) fn drop_internal_ground_floors(polygons: &mut Vec<(SurfaceType, Vec<DVec3>)>) {
    const Z_GAP: f64 = 1.0;

    let grounds: Vec<(usize, DVec3, f64, f64)> = polygons
        .iter()
        .enumerate()
        .filter(|(_, (t, _))| *t == SurfaceType::Ground)
        .map(|(i, (_, verts))| {
            let centroid = verts.iter().copied().sum::<DVec3>() / verts.len() as f64;
            let min_z = verts.iter().map(|v| v.z).fold(f64::INFINITY, f64::min);
            let max_z = verts.iter().map(|v| v.z).fold(f64::NEG_INFINITY, f64::max);
            (i, centroid, min_z, max_z)
        })
        .collect();

    let mut drop = vec![false; polygons.len()];
    for &(ai, ac, a_min_z, _) in &grounds {
        for &(bi, _, _, b_max_z) in &grounds {
            if ai == bi {
                continue;
            }
            if a_min_z <= b_max_z + Z_GAP {
                continue;
            }
            let (_, b_verts) = &polygons[bi];
            if point_in_polygon_xy(ac, b_verts) {
                drop[ai] = true;
                break;
            }
        }
    }

    let mut idx = 0;
    polygons.retain(|_| {
        let keep = !drop[idx];
        idx += 1;
        keep
    });
}

/// 2D point-in-polygon test using the ray-casting (even-odd) rule on xy.
fn point_in_polygon_xy(p: DVec3, polygon: &[DVec3]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = polygon[i];
        let pj = polygon[j];
        let crosses = (pi.y > p.y) != (pj.y > p.y)
            && p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x;
        if crosses {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Detect every closed loop of unmatched directed polygon edges and append a
/// cap polygon for each one. Each cap is a fan from a centroid vertex shared
/// by every triangle of that cap, so the cap itself is internally manifold
/// (every fan edge appears in exactly two cap triangles), and the perimeter
/// edges go in the reverse direction of the walked boundary so they share
/// each unmatched edge in opposite direction → manifold against the rest of
/// the building.
///
/// Operates on the polygon list before triangulation so the new cap participates
/// in the subsequent T-junction fixing and orient-consistency passes.
pub(super) fn close_open_boundary_loops(
    polygons: &mut Vec<(SurfaceType, Vec<DVec3>)>,
    tol: f64,
) {
    // Tally directed edge counts across all polygons.
    let mut directed: HashMap<(GridKey, GridKey), i32> = HashMap::new();
    let mut coord: HashMap<GridKey, DVec3> = HashMap::new();
    for (_, verts) in polygons.iter() {
        let n = verts.len();
        for i in 0..n {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            let ka = snap_key(a, tol);
            let kb = snap_key(b, tol);
            if ka == kb {
                continue;
            }
            coord.entry(ka).or_insert(a);
            coord.entry(kb).or_insert(b);
            *directed.entry((ka, kb)).or_insert(0) += 1;
        }
    }

    // Unmatched directed edges (forward count > reverse count) are the boundary.
    // Sort everything so the walk order — and therefore the resulting cap shape
    // — is deterministic regardless of HashMap iteration order.
    let mut directed_keys: Vec<(GridKey, GridKey)> = directed.keys().copied().collect();
    directed_keys.sort();
    let mut next_map: HashMap<GridKey, Vec<GridKey>> = HashMap::new();
    let mut unmatched: Vec<(GridKey, GridKey)> = Vec::new();
    for &(a, b) in &directed_keys {
        let count = directed[&(a, b)];
        let reverse = directed.get(&(b, a)).copied().unwrap_or(0);
        let extra = count - reverse;
        for _ in 0..extra.max(0) {
            unmatched.push((a, b));
            next_map.entry(a).or_default().push(b);
        }
    }
    for nexts in next_map.values_mut() {
        nexts.sort();
    }
    if unmatched.is_empty() {
        return;
    }

    let mut used: HashSet<(GridKey, GridKey)> = HashSet::new();
    for &(start_a, start_b) in &unmatched {
        if used.contains(&(start_a, start_b)) {
            continue;
        }
        used.insert((start_a, start_b));
        let mut loop_keys: Vec<GridKey> = vec![start_a];
        let mut current = start_b;
        let mut closed = false;
        // Bound the walk so a corrupt graph can't loop forever.
        for _ in 0..unmatched.len() + 1 {
            if current == start_a {
                closed = true;
                break;
            }
            loop_keys.push(current);
            let Some(nexts) = next_map.get(&current) else {
                break;
            };
            let Some(&n) = nexts.iter().find(|&&n| !used.contains(&(current, n))) else {
                break;
            };
            used.insert((current, n));
            current = n;
        }
        if !closed || loop_keys.len() < 3 {
            continue;
        }

        // Reverse the walked direction so the cap's perimeter edges are b→a,
        // cancelling the boundary edges a→b.
        loop_keys.reverse();
        let cap: Vec<DVec3> = loop_keys
            .iter()
            .map(|k| *coord.get(k).expect("loop key seen during walk"))
            .collect();

        // Ear-clip in the cap's best-fit plane so triangles follow the hole's
        // contour. A naive corner-fan would span the entire opening and emit
        // huge slivers across long, non-convex holes such as a bridge
        // underside. Each triangle is pushed as its own 3-vertex polygon — the
        // standard fan_triangulate step handles 3-vertex polygons trivially.
        let cap_tris = ear_clip_triangulate(&cap);
        if cap_tris.is_empty() {
            // Triangulation failed (degenerate loop); fall back to whole-loop
            // polygon — fan_triangulate will at least keep things manifold.
            polygons.push((SurfaceType::Closure, cap));
        } else {
            for tri in cap_tris {
                polygons.push((
                    SurfaceType::Closure,
                    vec![cap[tri[0]], cap[tri[1]], cap[tri[2]]],
                ));
            }
        }
    }
}

/// Greedy iterative pass: while any edge is shared by 3+ polygons, drop the
/// single polygon whose removal most improves the manifold metric.
///
/// "Improvement" per edge of a candidate polygon:
///   * count ≥ 3 → +1 (resolves an over-shared edge)
///   * count == 1 → +1 (removes an orphan boundary edge)
///   * count == 2 → -1 (would turn a clean edge into a boundary)
///
/// Closure surfaces get a +1 tiebreaker because they are CityGML's virtual
/// closures and are the most likely to be redundant.
pub(super) fn drop_redundant_polygons(
    polygons: &mut Vec<(SurfaceType, Vec<DVec3>)>,
    tol: f64,
) {
    loop {
        let mut edge_count: HashMap<(GridKey, GridKey), usize> = HashMap::new();
        for (_, verts) in polygons.iter() {
            let n = verts.len();
            for i in 0..n {
                let a = snap_key(verts[i], tol);
                let b = snap_key(verts[(i + 1) % n], tol);
                if a == b {
                    continue;
                }
                let key = if a < b { (a, b) } else { (b, a) };
                *edge_count.entry(key).or_insert(0) += 1;
            }
        }

        let has_overshared = edge_count.values().any(|&c| c > 2);
        if !has_overshared {
            return;
        }

        let mut best: Option<(usize, i32)> = None;
        for (idx, (surface_type, verts)) in polygons.iter().enumerate() {
            let mut benefit: i32 = 0;
            let n = verts.len();
            let mut touches_overshared = false;
            for i in 0..n {
                let a = snap_key(verts[i], tol);
                let b = snap_key(verts[(i + 1) % n], tol);
                if a == b {
                    continue;
                }
                let key = if a < b { (a, b) } else { (b, a) };
                let c = edge_count.get(&key).copied().unwrap_or(0);
                match c {
                    c if c >= 3 => {
                        benefit += 1;
                        touches_overshared = true;
                    }
                    2 => benefit -= 1,
                    1 => benefit += 1,
                    _ => {}
                }
            }
            if !touches_overshared {
                continue;
            }
            if matches!(surface_type, SurfaceType::Closure) {
                benefit += 1;
            }
            if best.is_none_or(|(_, b)| benefit > b) {
                best = Some((idx, benefit));
            }
        }

        match best {
            Some((idx, benefit)) if benefit > 0 => {
                polygons.remove(idx);
            }
            _ => return,
        }
    }
}

/// Split rings that touch themselves into separate non-self-touching rings.
///
/// If a polygon's vertex list contains the same snapped point at indices `i`
/// and `j` (i<j), the ring is two loops joined at that pinch point: the outer
/// loop `verts[0..=i] + verts[j+1..]` and the inner loop `verts[i..j]`. We
/// emit both and recurse — either piece may still self-touch.
pub(super) fn split_self_touching_polygons(
    polygons: &mut Vec<(SurfaceType, Vec<DVec3>)>,
    tol: f64,
) {
    let mut output: Vec<(SurfaceType, Vec<DVec3>)> = Vec::new();
    let mut work: Vec<(SurfaceType, Vec<DVec3>)> = std::mem::take(polygons);

    while let Some((surface_type, verts)) = work.pop() {
        if verts.len() < 3 {
            continue;
        }
        let keys: Vec<GridKey> = verts.iter().map(|v| snap_key(*v, tol)).collect();
        let n = keys.len();
        let mut found: Option<(usize, usize)> = None;
        'outer: for i in 0..n {
            for j in (i + 1)..n {
                if keys[i] == keys[j] {
                    found = Some((i, j));
                    break 'outer;
                }
            }
        }
        match found {
            None => output.push((surface_type, verts)),
            Some((i, j)) => {
                let mut outer_ring: Vec<DVec3> = verts[0..=i].to_vec();
                outer_ring.extend_from_slice(&verts[j + 1..]);
                let inner_ring: Vec<DVec3> = verts[i..j].to_vec();
                if outer_ring.len() >= 3 {
                    work.push((surface_type, outer_ring));
                }
                if inner_ring.len() >= 3 {
                    work.push((surface_type, inner_ring));
                }
            }
        }
    }
    *polygons = output;
}

/// Remove duplicate polygons from the list. A duplicate is any polygon whose
/// snapped vertex cycle equals another polygon's cycle under either rotation
/// or reversal. The first occurrence wins; subsequent copies are dropped.
pub(super) fn dedup_polygons(polygons: &mut Vec<(SurfaceType, Vec<DVec3>)>, tol: f64) {
    let canon = |verts: &[DVec3]| -> Vec<GridKey> {
        let keys: Vec<GridKey> = verts.iter().map(|v| snap_key(*v, tol)).collect();
        let n = keys.len();
        if n == 0 {
            return Vec::new();
        }
        let start = (0..n).min_by_key(|&i| keys[i]).unwrap();
        let forward: Vec<_> = (0..n).map(|i| keys[(start + i) % n]).collect();
        let reverse: Vec<_> = (0..n).map(|i| keys[(start + n - i) % n]).collect();
        if forward < reverse { forward } else { reverse }
    };
    let mut seen: HashSet<Vec<GridKey>> = HashSet::new();
    polygons.retain(|(_, verts)| {
        let c = canon(verts);
        !c.is_empty() && seen.insert(c)
    });
}

/// Insert intermediate vertices into polygon edges so that T-junctions become
/// proper shared vertices. For each edge (a, b) in every polygon, if any other
/// vertex `v` from any polygon lies on the open segment (a, b) within `tol`,
/// `v` is inserted between `a` and `b` in the polygon's vertex list.
///
/// Two passes: first collect every vertex that ever appears anywhere; then
/// rewrite every polygon's vertex list with extra splits.
pub(super) fn fix_t_junctions(polygons: &mut [(SurfaceType, Vec<DVec3>)], tol: f64) {
    // Build the global set of unique snapped vertices.
    let mut all_verts: Vec<DVec3> = Vec::new();
    let mut seen: HashSet<GridKey> = HashSet::new();
    for (_, verts) in polygons.iter() {
        for v in verts {
            if seen.insert(snap_key(*v, tol)) {
                all_verts.push(*v);
            }
        }
    }

    // Helper: is vertex `v` on the open segment (a, b)?
    // Returns Some(t) where 0 < t < 1 if yes, None otherwise.
    fn on_segment(a: DVec3, b: DVec3, v: DVec3, tol: f64) -> Option<f64> {
        let d = b - a;
        let d_len_sq = d.length_squared();
        if d_len_sq < tol * tol {
            return None;
        }
        let t = (v - a).dot(d) / d_len_sq;
        // Strictly between endpoints, with a small tolerance margin.
        let margin = (tol / d_len_sq.sqrt()).max(1e-6);
        if t <= margin || t >= 1.0 - margin {
            return None;
        }
        let p = a + t * d;
        if (v - p).length() < tol {
            Some(t)
        } else {
            None
        }
    }

    let tol_sq = tol * tol;
    for (_, verts) in polygons.iter_mut() {
        // Snapshot the polygon's existing vertex keys so we never re-insert a
        // vertex it already contains — that would happen for a degenerate
        // collinear triangle (a, m, b) where m sits on edge a→b, and would
        // turn the polygon into [a, m, b, m] whose fan-triangulation emits a
        // duplicate of the degenerate triangle.
        let own_keys: HashSet<GridKey> =
            verts.iter().map(|v| snap_key(*v, tol)).collect();

        let mut new_verts: Vec<DVec3> = Vec::with_capacity(verts.len() * 2);
        let n = verts.len();
        for i in 0..n {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            new_verts.push(a);

            // Find every other unique vertex sitting on segment (a, b).
            let mut splits: Vec<(f64, DVec3)> = Vec::new();
            for v in &all_verts {
                // Skip vertices that already appear elsewhere in this polygon.
                if own_keys.contains(&snap_key(*v, tol)) {
                    continue;
                }
                // Skip vertices that ARE a or b.
                if (*v - a).length_squared() < tol_sq {
                    continue;
                }
                if (*v - b).length_squared() < tol_sq {
                    continue;
                }
                if let Some(t) = on_segment(a, b, *v, tol) {
                    splits.push((t, *v));
                }
            }
            splits.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
            for (_, v) in splits {
                new_verts.push(v);
            }
        }
        *verts = new_verts;
    }
}

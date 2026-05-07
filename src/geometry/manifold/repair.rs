use std::collections::{HashMap, VecDeque};

use glam::DVec3;

/// Greedy manifold repair: while any undirected edge is shared by >2
/// triangles, drop the triangle whose removal most improves the metric.
/// Each candidate triangle scores +1 per over-shared edge it touches, -1
/// per currently-manifold (count=2) edge, and 0 per boundary edge. Prefer
/// strictly positive trades; accept zero-score trades (one over-shared
/// resolved at the cost of one new boundary) when nothing better exists,
/// since converting an over-shared edge to a boundary turns a non-manifold
/// solid into a non-manifold open mesh — same severity, but more amenable
/// to downstream cap-closing. Capped iteration count keeps a corrupt graph
/// from looping.
pub(super) fn repair_overshared_edges(tris: &mut Vec<[u32; 3]>) {
    let cap = tris.len() * 4;
    for _ in 0..cap {
        let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
        for tri in tris.iter() {
            for e in 0..3 {
                let a = tri[e];
                let b = tri[(e + 1) % 3];
                let key = if a < b { (a, b) } else { (b, a) };
                *edge_count.entry(key).or_insert(0) += 1;
            }
        }
        if !edge_count.values().any(|&c| c > 2) {
            return;
        }

        let mut best: Option<(usize, i32)> = None;
        for (i, tri) in tris.iter().enumerate() {
            let mut touches_overshared = false;
            let mut score: i32 = 0;
            for e in 0..3 {
                let a = tri[e];
                let b = tri[(e + 1) % 3];
                let key = if a < b { (a, b) } else { (b, a) };
                let c = edge_count.get(&key).copied().unwrap_or(0);
                match c {
                    c if c >= 3 => {
                        score += 1;
                        touches_overshared = true;
                    }
                    2 => score -= 1,
                    _ => {}
                }
            }
            if !touches_overshared {
                continue;
            }
            if best.is_none_or(|(_, b)| score > b) {
                best = Some((i, score));
            }
        }

        match best {
            Some((idx, score)) if score >= 0 => {
                tris.swap_remove(idx);
            }
            _ => return,
        }
    }
}

/// Make face winding consistent across the whole mesh, then orient each
/// connected component outward (signed volume > 0).
pub(super) fn orient_consistently(tris: &mut [[u32; 3]], verts: &[DVec3]) {
    if tris.is_empty() {
        return;
    }

    let n = tris.len();

    // Map each undirected edge to the triangles that touch it.
    let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, tri) in tris.iter().enumerate() {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            edge_to_tris.entry(key).or_default().push(i);
        }
    }

    // BFS over face adjacency. For each pair of adjacent triangles sharing
    // an edge, the shared edge must be traversed in opposite directions; if
    // not, flip the neighbour and propagate.
    let mut visited = vec![false; n];
    let mut component_of = vec![usize::MAX; n];
    let mut components: usize = 0;
    let mut queue: VecDeque<usize> = VecDeque::new();

    for seed in 0..n {
        if visited[seed] {
            continue;
        }
        visited[seed] = true;
        component_of[seed] = components;
        queue.push_back(seed);
        while let Some(t) = queue.pop_front() {
            let tri = tris[t];
            for e in 0..3 {
                let a = tri[e];
                let b = tri[(e + 1) % 3];
                let key = if a < b { (a, b) } else { (b, a) };
                let Some(neighbours) = edge_to_tris.get(&key) else {
                    continue;
                };
                for &nb in neighbours {
                    if nb == t || visited[nb] {
                        continue;
                    }
                    // Inspect the neighbour's traversal of the shared edge.
                    let ntri = tris[nb];
                    let mut same_direction = false;
                    for ne in 0..3 {
                        let na = ntri[ne];
                        let nb_v = ntri[(ne + 1) % 3];
                        if na == a && nb_v == b {
                            same_direction = true;
                            break;
                        }
                    }
                    if same_direction {
                        tris[nb] = [ntri[0], ntri[2], ntri[1]];
                    }
                    visited[nb] = true;
                    component_of[nb] = components;
                    queue.push_back(nb);
                }
            }
        }
        components += 1;
    }

    // Per-component signed-volume check: flip the whole component if it ends
    // up inward-facing. Reference point is the origin — works because the
    // formula sums signed tetrahedra and the constant offset cancels for a
    // closed mesh.
    let mut vol = vec![0.0_f64; components];
    for (i, tri) in tris.iter().enumerate() {
        let a = verts[tri[0] as usize];
        let b = verts[tri[1] as usize];
        let c = verts[tri[2] as usize];
        vol[component_of[i]] += a.dot(b.cross(c));
    }
    for (i, tri) in tris.iter_mut().enumerate() {
        if vol[component_of[i]] < 0.0 {
            *tri = [tri[0], tri[2], tri[1]];
        }
    }
}

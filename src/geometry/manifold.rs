use std::collections::HashMap;

use glam::DVec3;

use crate::model::{
    building::{Building, Polygon, Surface, SurfaceType},
    terrain::{GlobalTerrain, TerrainTile},
};
use super::{
    normals::{ear_clip_triangulate, polygon_normal},
    vertex_merge::VertexMerger,
};

// ---------------------------------------------------------------------------
// Combined-terrain manifold mesh (preferred over per-tile)
// ---------------------------------------------------------------------------

/// Build a single watertight terrain mesh from a combined elevation grid.
///
/// Output anatomy:
///   * Top surface: 2 triangles per grid cell where all four corners are valid.
///     Cells with NaN corners are simply skipped (small holes appear).
///   * Side walls: each of the four outer rectangle faces is fan-triangulated
///     from one shared bottom corner, walking the top boundary vertices.
///     This keeps every edge shared by exactly 2 triangles (manifold).
///   * Bottom cap: just two triangles using the same 4 corner vertices.
///
/// The whole mesh is one closed solid with shared vertices throughout, so all
/// tiles connect seamlessly to their neighbours.
pub fn build_manifold_global_terrain(
    grid: &GlobalTerrain,
) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    let mut tris: Vec<[u32; 3]> = Vec::new();

    if grid.cols < 2 || grid.rows < 2 {
        return (Vec::new(), tris);
    }

    let cols = grid.cols;
    let rows = grid.rows;

    let z_top_min = grid.min_z();
    let z_base = if z_top_min.is_finite() { z_top_min - 1.0 } else { -1.0 };

    // --- 1. Top vertices: one per grid pixel center, in row-major order. ---
    let mut verts: Vec<DVec3> = Vec::with_capacity(cols * rows + 4);
    for r in 0..rows {
        let y = grid.y_at(r);
        for c in 0..cols {
            let z = grid.z_at(c, r);
            verts.push(DVec3::new(grid.x_at(c), y, z));
        }
    }
    let idx_top = |r: usize, c: usize| (r * cols + c) as u32;

    // --- 2. Top surface triangles. Skip cells where any corner is NaN. ---
    for r in 0..rows - 1 {
        for c in 0..cols - 1 {
            let z00 = grid.z_at(c, r);
            let z10 = grid.z_at(c + 1, r);
            let z01 = grid.z_at(c, r + 1);
            let z11 = grid.z_at(c + 1, r + 1);
            if !(z00.is_finite() && z10.is_finite() && z01.is_finite() && z11.is_finite()) {
                continue;
            }
            let i00 = idx_top(r, c);
            let i10 = idx_top(r, c + 1);
            let i01 = idx_top(r + 1, c);
            let i11 = idx_top(r + 1, c + 1);
            // CCW from above (outward normal +Z).
            tris.push([i00, i01, i11]);
            tris.push([i00, i11, i10]);
        }
    }

    // --- 3. Four bottom corner vertices (only). ---
    let xmin = grid.x_at(0);
    let xmax = grid.x_at(cols - 1);
    let ymax = grid.y_at(0);
    let ymin = grid.y_at(rows - 1);
    let bot_nw = verts.len() as u32; verts.push(DVec3::new(xmin, ymax, z_base));
    let bot_ne = verts.len() as u32; verts.push(DVec3::new(xmax, ymax, z_base));
    let bot_se = verts.len() as u32; verts.push(DVec3::new(xmax, ymin, z_base));
    let bot_sw = verts.len() as u32; verts.push(DVec3::new(xmin, ymin, z_base));

    // --- 4. Skirt walls. Each wall is fan-triangulated from one bottom corner.
    //
    // North wall (row=0, outward +Y): fan apex = bot_nw, going west→east.
    for c in 0..cols - 1 {
        tris.push([bot_nw, idx_top(0, c), idx_top(0, c + 1)]);
    }
    tris.push([bot_nw, idx_top(0, cols - 1), bot_ne]);

    // East wall (col=cols-1, outward +X): fan apex = bot_ne, going north→south.
    for r in 0..rows - 1 {
        tris.push([bot_ne, idx_top(r, cols - 1), idx_top(r + 1, cols - 1)]);
    }
    tris.push([bot_ne, idx_top(rows - 1, cols - 1), bot_se]);

    // South wall (row=rows-1, outward -Y): fan apex = bot_se, going east→west.
    for c in (1..cols).rev() {
        tris.push([bot_se, idx_top(rows - 1, c), idx_top(rows - 1, c - 1)]);
    }
    tris.push([bot_se, idx_top(rows - 1, 0), bot_sw]);

    // West wall (col=0, outward -X): fan apex = bot_sw, going south→north.
    for r in (1..rows).rev() {
        tris.push([bot_sw, idx_top(r, 0), idx_top(r - 1, 0)]);
    }
    tris.push([bot_sw, idx_top(0, 0), bot_nw]);

    // --- 5. Bottom cap: 2 triangles, outward normal -Z (CW viewed from above). ---
    tris.push([bot_nw, bot_ne, bot_se]);
    tris.push([bot_nw, bot_se, bot_sw]);

    (verts, tris)
}

// ---------------------------------------------------------------------------
// Per-tile terrain manifold mesh (legacy; not used in the main pipeline)
// ---------------------------------------------------------------------------

#[allow(dead_code)]
pub fn build_manifold_terrain(tile: &TerrainTile) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    let mut merger = VertexMerger::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();

    // Height of the skirt bottom (1m below the tile minimum elevation).
    let z_base = tile.min_z() - 1.0;

    // --- 1. Top surface and skirt ---
    // Process each grid cell [row, col].
    // valid[row][col] = true if the cell has finite elevation.
    let valid: Vec<bool> = tile.z.iter().map(|z| z.is_finite()).collect();
    let idx = |r: usize, c: usize| r * tile.cols + c;

    // Track which top-surface edges are boundary edges (one valid cell, the other missing/OOB).
    // We'll build skirt quads for these.
    struct SkirtEdge {
        a: DVec3, // world-space top vertex 1
        b: DVec3, // world-space top vertex 2
    }
    let mut skirt_edges: Vec<SkirtEdge> = Vec::new();

    for row in 0..tile.rows.saturating_sub(1) {
        for col in 0..tile.cols.saturating_sub(1) {
            // Four corners of the cell.
            let tl = (row, col);
            let tr = (row, col + 1);
            let bl = (row + 1, col);
            let br = (row + 1, col + 1);

            let all_valid = valid[idx(tl.0, tl.1)]
                && valid[idx(tr.0, tr.1)]
                && valid[idx(bl.0, bl.1)]
                && valid[idx(br.0, br.1)];

            if !all_valid {
                continue;
            }

            let vtl = DVec3::new(tile.x_at(tl.1), tile.y_at(tl.0), tile.z_at(tl.1, tl.0));
            let vtr = DVec3::new(tile.x_at(tr.1), tile.y_at(tr.0), tile.z_at(tr.1, tr.0));
            let vbl = DVec3::new(tile.x_at(bl.1), tile.y_at(bl.0), tile.z_at(bl.1, bl.0));
            let vbr = DVec3::new(tile.x_at(br.1), tile.y_at(br.0), tile.z_at(br.1, br.0));

            // Two triangles for the top cell — CCW viewed from above.
            let itl = merger.insert(vtl);
            let itr = merger.insert(vtr);
            let ibl = merger.insert(vbl);
            let ibr = merger.insert(vbr);

            tris.push([itl, ibl, ibr]);
            tris.push([itl, ibr, itr]);

            // Collect boundary edges for the skirt.
            // Top edge (row, col) → (row, col+1): boundary if row == 0
            if row == 0 {
                skirt_edges.push(SkirtEdge { a: vtl, b: vtr });
            }
            // Bottom edge: boundary if next row is OOB or invalid
            if row + 2 == tile.rows
                || !(valid[idx(row + 2, col)] && valid[idx(row + 2, col + 1)])
            {
                skirt_edges.push(SkirtEdge { a: vbr, b: vbl }); // reversed for outward normal
            }
            // Left edge: boundary if col == 0
            if col == 0 {
                skirt_edges.push(SkirtEdge { a: vbl, b: vtl });
            }
            // Right edge: boundary if next col is OOB or invalid
            if col + 2 == tile.cols
                || !(valid[idx(row, col + 2)] && valid[idx(row + 1, col + 2)])
            {
                skirt_edges.push(SkirtEdge { a: vtr, b: vbr });
            }
        }
    }

    // --- 2. Skirt walls ---
    // For each boundary edge, emit a quad (two triangles) dropping to z_base.
    let mut bottom_perimeter: Vec<DVec3> = Vec::new();
    for edge in &skirt_edges {
        let a_top = edge.a;
        let b_top = edge.b;
        let a_bot = DVec3::new(a_top.x, a_top.y, z_base);
        let b_bot = DVec3::new(b_top.x, b_top.y, z_base);

        let ia_top = merger.insert(a_top);
        let ib_top = merger.insert(b_top);
        let ia_bot = merger.insert(a_bot);
        let ib_bot = merger.insert(b_bot);

        // Quad: a_top, b_top, b_bot, a_bot — CCW from outside (outward normal points outward)
        tris.push([ia_top, ia_bot, ib_bot]);
        tris.push([ia_top, ib_bot, ib_top]);

        bottom_perimeter.push(a_bot);
    }

    // --- 3. Bottom cap ---
    // Collect unique bottom perimeter vertices and fan-triangulate.
    if !bottom_perimeter.is_empty() {
        // Compute centroid of bottom perimeter.
        let centroid = bottom_perimeter.iter().copied().sum::<DVec3>()
            / bottom_perimeter.len() as f64;
        let centroid = DVec3::new(centroid.x, centroid.y, z_base);
        let ic = merger.insert(centroid);

        // Fan from centroid to each consecutive pair of perimeter vertices.
        // Reverse winding (CW from above = CCW from below = outward normal down).
        for i in 0..bottom_perimeter.len() {
            let a = bottom_perimeter[i];
            let b = bottom_perimeter[(i + 1) % bottom_perimeter.len()];
            let ia = merger.insert(a);
            let ib = merger.insert(b);
            tris.push([ic, ib, ia]); // reversed for downward-facing normal
        }
    }

    (merger.verts, tris)
}

// ---------------------------------------------------------------------------
// Building manifold mesh
// ---------------------------------------------------------------------------

/// Vertex-snap tolerance used by the building merger and T-junction fixer (5 mm).
const BUILDING_TOL: f64 = 0.005;

/// Convert a building's surfaces into a manifold mesh.
/// Pipeline:
///  1. Snap vertices in every polygon to a shared 5 mm grid (collapses near-duplicates).
///  2. Insert intermediate vertices to remove T-junctions (the dominant source
///     of non-manifold edges in CityGML LoD2 data).
///  3. Synthesize a ground surface if the source has none.
///  4. Re-orient face windings so all normals point outward.
///  5. Warn if any boundary edges remain after all of the above.
pub fn build_manifold_building(building: &Building) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    // ---- (1) Snap vertices on a per-polygon basis to a global 5mm grid. ----
    let mut polygons: Vec<(SurfaceType, Vec<DVec3>)> = Vec::new();
    for surface in &building.surfaces {
        for polygon in &surface.polygons {
            if polygon.0.len() < 3 {
                continue;
            }
            let snapped: Vec<DVec3> = polygon.0.iter().map(|v| snap_vec(*v, BUILDING_TOL)).collect();
            polygons.push((surface.surface_type, snapped));
        }
    }

    // ---- (1b) Split self-touching polygons.
    //
    // LVermGeo encodes some roof surfaces as a single ring that revisits the
    // same vertex twice — used to lasso around a tiny pillar/turret. Fan-
    // triangulating such a ring produces overlapping triangles and over-shared
    // edges. We split each self-touching ring into an outer ring (with the
    // detour removed) and the detour itself as its own closed loop. Each piece
    // is recursively re-checked because it may still self-touch.
    split_self_touching_polygons(&mut polygons, BUILDING_TOL);

    // ---- (1c) Drop duplicate polygons.
    //
    // LVermGeo LoD2 data sometimes encodes the same quadrilateral as both a
    // `WallSurface` and a `ClosureSurface`, or as two `ClosureSurface`s with
    // opposite winding. Each duplicate emits its own triangulation, so every
    // interior edge ends up shared by 4 triangles (over-shared, non-manifold).
    // Two polygons are duplicates if their cyclic vertex sequences match — up
    // to rotation and direction reversal — once snapped to the building grid.
    dedup_polygons(&mut polygons, BUILDING_TOL);

    // ---- (2) T-junction fixing: ensure every vertex on a polygon edge is
    //          part of every other polygon that touches that edge. ----
    fix_t_junctions(&mut polygons, BUILDING_TOL);

    // ---- (2b) Drop redundant polygons that create non-manifold edges.
    //
    // CityGML LoD2 from LVermGeo sometimes encodes the same wall as both a
    // semantic `WallSurface` and a `ClosureSurface` (or as two closures), or
    // packs three coplanar walls onto the same vertical edge at a tight corner.
    // Each redundant polygon adds an extra triangulation to edges that are
    // already covered, pushing edge count past 2 (over-shared, non-manifold).
    //
    // We iteratively remove the polygon whose deletion most improves the
    // manifold metric (reduces over-shared edges without introducing new
    // boundary ones). Closures are preferred over walls when tied because
    // they're virtual closures rather than physical surfaces.
    drop_redundant_polygons(&mut polygons, BUILDING_TOL);

    // ---- (3) Drop spurious internal horizontal "floors".
    //
    // Some LVermGeo CityGML files mark a BuildingPart's upper-level floor as
    // a `GroundSurface`. Including it produces an internal slab (faces shared
    // by 3+ triangles, non-manifold). We only want exterior-skin polygons:
    //   * Keep ground polygons at min_z (the real bottom).
    //   * Drop any `GroundSurface` polygon whose vertices lie >1 m above min_z.
    //   * Likewise, drop any flat horizontal polygon (any type) whose vertices
    //     sit at the same Z, that Z isn't min_z, and there are walls extending
    //     both above and below it.
    let min_z = polygons
        .iter()
        .flat_map(|(_, p)| p.iter())
        .map(|v| v.z)
        .fold(f64::INFINITY, f64::min);

    polygons.retain(|(t, verts)| {
        if *t == SurfaceType::Ground {
            // Keep only if at the actual ground.
            verts.iter().all(|v| (v.z - min_z).abs() < 0.5)
        } else {
            true
        }
    });

    // ---- (4) Close any open boundary loops with cap polygons.
    //
    // Some LVermGeo buildings (notably bridges) ship without a polygon for the
    // underside, leaving a hole bordered by a closed loop of unmatched polygon
    // edges. Detect every such loop in the directed-edge graph, ear-clip-
    // triangulate it in its best-fit plane, and emit the resulting triangles
    // as cap polygons.
    let _ = min_z; // kept for clarity; min_z used by earlier filter step.
    close_open_boundary_loops(&mut polygons, BUILDING_TOL);
    // No second T-junction pass: the cap is built from the same vertices that
    // already exist on the wall side, so cap edges and wall edges already
    // share endpoints. Re-running fix_t_junctions here would *re-insert* a
    // collinear cap-perimeter vertex into another cap triangle's internal
    // diagonal, creating duplicate degenerate triangles.

    // ---- (4) Build the actual mesh: vertex-merge + fan-triangulate + winding. ----
    let mut merger = VertexMerger::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();

    let centroid = building.centroid_xy().unwrap_or(DVec3::ZERO);

    for (surface_type, verts) in &polygons {
        if verts.len() < 3 {
            continue;
        }
        let indices: Vec<u32> = verts.iter().map(|v| merger.insert(*v)).collect();

        let normal = polygon_normal(verts);
        let expected_outward = expected_normal_direction(*surface_type, centroid, verts);

        // Ear-clipping (rather than naive fan-triangulation) picks each diagonal
        // based on local convexity, so two unrelated polygons that happen to
        // share two non-adjacent vertices won't both produce the same internal
        // diagonal — which would otherwise cause that edge to be shared by
        // 4 triangles (non-manifold). If ear-clipping stalls on a degenerate
        // polygon, fall back to a fan from vertex 0.
        let raw_tri_idx = ear_clip_triangulate(verts);
        let raw_tris: Vec<[u32; 3]> = if raw_tri_idx.is_empty() && verts.len() >= 3 {
            (1..verts.len() - 1)
                .map(|i| [indices[0], indices[i], indices[i + 1]])
                .collect()
        } else {
            raw_tri_idx
                .into_iter()
                .map(|t| [indices[t[0]], indices[t[1]], indices[t[2]]])
                .collect()
        };
        let flip = normal.dot(expected_outward) < 0.0;

        for tri in raw_tris {
            // Drop degenerate triangles created by collinear/duplicate vertices.
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                continue;
            }
            if flip {
                tris.push([tri[0], tri[2], tri[1]]);
            } else {
                tris.push(tri);
            }
        }
    }

    // ---- (5) Consistent orientation pass. ----
    //
    // The per-face "expected outward normal" heuristic above is local and gets
    // walls at concave corners wrong (the building centroid sits on the inside
    // of the corner, so `face_center − centroid` points the wrong way). That
    // produces edges traversed in the *same* direction by both adjacent
    // triangles — a winding inconsistency Blender flags as non-manifold even
    // though edge-count is exactly 2.
    //
    // BFS over face adjacency: starting from any seed, every neighbour that
    // shares an edge in the same direction gets flipped. After the walk each
    // connected component is internally consistent. Then a single signed-volume
    // check per component flips it if it ended up inward-facing.
    orient_consistently(&mut tris, &merger.verts);

    // ---- (7) Manifold check. ----
    check_manifold(building.id.as_str(), &tris, merger.len());

    (merger.verts, tris)
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
fn close_open_boundary_loops(polygons: &mut Vec<(SurfaceType, Vec<DVec3>)>, tol: f64) {
    use std::collections::{HashMap, HashSet};

    let inv = 1.0 / tol;
    let key_of = |v: DVec3| -> (i64, i64, i64) {
        (
            (v.x * inv).round() as i64,
            (v.y * inv).round() as i64,
            (v.z * inv).round() as i64,
        )
    };

    // Tally directed edge counts across all polygons.
    let mut directed: HashMap<((i64, i64, i64), (i64, i64, i64)), i32> = HashMap::new();
    let mut coord: HashMap<(i64, i64, i64), DVec3> = HashMap::new();
    for (_, verts) in polygons.iter() {
        let n = verts.len();
        for i in 0..n {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            let ka = key_of(a);
            let kb = key_of(b);
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
    let mut directed_keys: Vec<((i64, i64, i64), (i64, i64, i64))> =
        directed.keys().copied().collect();
    directed_keys.sort();
    let mut next_map: HashMap<(i64, i64, i64), Vec<(i64, i64, i64)>> = HashMap::new();
    let mut unmatched: Vec<((i64, i64, i64), (i64, i64, i64))> = Vec::new();
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

    let mut used: HashSet<((i64, i64, i64), (i64, i64, i64))> = HashSet::new();
    for &(start_a, start_b) in &unmatched {
        if used.contains(&(start_a, start_b)) {
            continue;
        }
        used.insert((start_a, start_b));
        let mut loop_keys: Vec<(i64, i64, i64)> = vec![start_a];
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

/// Make face winding consistent across the whole mesh, then orient each
/// connected component outward (signed volume > 0).
fn orient_consistently(tris: &mut [[u32; 3]], verts: &[DVec3]) {
    use std::collections::VecDeque;

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

/// Snap a vertex to a fixed grid for stable equality testing.
fn snap_vec(v: DVec3, tol: f64) -> DVec3 {
    let inv = 1.0 / tol;
    DVec3::new(
        (v.x * inv).round() / inv,
        (v.y * inv).round() / inv,
        (v.z * inv).round() / inv,
    )
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
fn drop_redundant_polygons(polygons: &mut Vec<(SurfaceType, Vec<DVec3>)>, tol: f64) {
    use std::collections::HashMap;
    let inv = 1.0 / tol;
    let snap = |v: DVec3| -> (i64, i64, i64) {
        (
            (v.x * inv).round() as i64,
            (v.y * inv).round() as i64,
            (v.z * inv).round() as i64,
        )
    };

    loop {
        let mut edge_count: HashMap<((i64, i64, i64), (i64, i64, i64)), usize> = HashMap::new();
        for (_, verts) in polygons.iter() {
            let n = verts.len();
            for i in 0..n {
                let a = snap(verts[i]);
                let b = snap(verts[(i + 1) % n]);
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
                let a = snap(verts[i]);
                let b = snap(verts[(i + 1) % n]);
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
            if best.map_or(true, |(_, b)| benefit > b) {
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
fn split_self_touching_polygons(polygons: &mut Vec<(SurfaceType, Vec<DVec3>)>, tol: f64) {
    let inv = 1.0 / tol;
    let key_of = |v: DVec3| -> (i64, i64, i64) {
        (
            (v.x * inv).round() as i64,
            (v.y * inv).round() as i64,
            (v.z * inv).round() as i64,
        )
    };

    let mut output: Vec<(SurfaceType, Vec<DVec3>)> = Vec::new();
    let mut work: Vec<(SurfaceType, Vec<DVec3>)> = std::mem::take(polygons);

    while let Some((surface_type, verts)) = work.pop() {
        if verts.len() < 3 {
            continue;
        }
        let keys: Vec<(i64, i64, i64)> = verts.iter().map(|v| key_of(*v)).collect();
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
fn dedup_polygons(polygons: &mut Vec<(SurfaceType, Vec<DVec3>)>, tol: f64) {
    use std::collections::HashSet;
    let inv = 1.0 / tol;
    let key_of = |v: DVec3| -> (i64, i64, i64) {
        (
            (v.x * inv).round() as i64,
            (v.y * inv).round() as i64,
            (v.z * inv).round() as i64,
        )
    };
    let canon = |verts: &[DVec3]| -> Vec<(i64, i64, i64)> {
        let keys: Vec<(i64, i64, i64)> = verts.iter().map(|v| key_of(*v)).collect();
        let n = keys.len();
        if n == 0 {
            return Vec::new();
        }
        let start = (0..n).min_by_key(|&i| keys[i]).unwrap();
        let forward: Vec<_> = (0..n).map(|i| keys[(start + i) % n]).collect();
        let reverse: Vec<_> = (0..n).map(|i| keys[(start + n - i) % n]).collect();
        if forward < reverse { forward } else { reverse }
    };
    let mut seen: HashSet<Vec<(i64, i64, i64)>> = HashSet::new();
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
fn fix_t_junctions(polygons: &mut [(SurfaceType, Vec<DVec3>)], tol: f64) {
    // Build the global set of unique snapped vertices.
    use std::collections::HashSet;
    let mut all_verts: Vec<DVec3> = Vec::new();
    let mut seen: HashSet<(i64, i64, i64)> = HashSet::new();
    let key_of = |v: DVec3, tol: f64| -> (i64, i64, i64) {
        let inv = 1.0 / tol;
        (
            (v.x * inv).round() as i64,
            (v.y * inv).round() as i64,
            (v.z * inv).round() as i64,
        )
    };
    for (_, verts) in polygons.iter() {
        for v in verts {
            if seen.insert(key_of(*v, tol)) {
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
        let own_keys: HashSet<(i64, i64, i64)> =
            verts.iter().map(|v| key_of(*v, tol)).collect();

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
                if own_keys.contains(&key_of(*v, tol)) {
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

/// Determine the expected outward normal direction for a surface.
fn expected_normal_direction(
    surface_type: SurfaceType,
    centroid: DVec3,
    polygon_verts: &[DVec3],
) -> DVec3 {
    match surface_type {
        SurfaceType::Roof => DVec3::new(0.0, 0.0, 1.0), // upward
        SurfaceType::Ground => DVec3::new(0.0, 0.0, -1.0), // downward
        SurfaceType::Wall | SurfaceType::Closure | SurfaceType::Unknown => {
            // Outward from building centroid.
            let face_center = polygon_verts.iter().copied().sum::<DVec3>()
                / polygon_verts.len() as f64;
            let outward = face_center - DVec3::new(centroid.x, centroid.y, face_center.z);
            if outward.length_squared() > 1e-10 {
                outward.normalize()
            } else {
                DVec3::X
            }
        }
    }
}

/// Stitch wall-bottom edges from the polygon list into a closed ground polygon.
///
/// Superseded by `close_open_boundary_loops`, which handles arbitrary
/// (including non-planar) boundary loops. Kept here as a simpler reference
/// for the flat-bottom-only case.
#[allow(dead_code)]
fn synthesize_ground_from_polygons(
    polygons: &[(SurfaceType, Vec<DVec3>)],
    min_z: f64,
) -> Option<Vec<DVec3>> {
    let z_tol = 0.05;

    // Collect every polygon edge whose two endpoints sit at min_z (within tol).
    // Each edge is stored as (from, to) world coords.
    let mut edges: Vec<(DVec3, DVec3)> = Vec::new();
    for (_, verts) in polygons {
        let n = verts.len();
        for i in 0..n {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            if (a.z - min_z).abs() < z_tol && (b.z - min_z).abs() < z_tol {
                // Skip degenerate edges
                if (a - b).length_squared() < 1e-8 {
                    continue;
                }
                edges.push((a, b));
            }
        }
    }
    if edges.len() < 3 {
        return None;
    }

    // Snap all coordinates so we can use them as map keys.
    let snap_xy = |v: DVec3| -> (i64, i64) {
        ((v.x * 1000.0).round() as i64, (v.y * 1000.0).round() as i64)
    };

    // Build adjacency map: vertex → list of next vertices.
    use std::collections::HashMap;
    let mut next: HashMap<(i64, i64), Vec<DVec3>> = HashMap::new();
    let mut all_points: HashMap<(i64, i64), DVec3> = HashMap::new();
    for (a, b) in &edges {
        let ka = snap_xy(*a);
        all_points.entry(ka).or_insert(*a);
        all_points.entry(snap_xy(*b)).or_insert(*b);
        next.entry(ka).or_default().push(*b);
    }

    // Each edge that's used by exactly one polygon (i.e. an outer perimeter
    // edge) appears in our list once; an internal edge between two adjacent
    // polygons appears twice in opposite directions. We want only the
    // perimeter edges (those whose reverse is NOT in the list).
    let mut edge_keys: HashMap<((i64, i64), (i64, i64)), i32> = HashMap::new();
    for (a, b) in &edges {
        let ka = snap_xy(*a);
        let kb = snap_xy(*b);
        *edge_keys.entry((ka, kb)).or_insert(0) += 1;
    }
    let perimeter_edges: Vec<(DVec3, DVec3)> = edges
        .iter()
        .copied()
        .filter(|(a, b)| {
            let ka = snap_xy(*a);
            let kb = snap_xy(*b);
            // Perimeter edge: forward count = 1 and reverse count = 0
            edge_keys.get(&(ka, kb)).copied().unwrap_or(0) == 1
                && edge_keys.get(&(kb, ka)).copied().unwrap_or(0) == 0
        })
        .collect();

    if perimeter_edges.len() < 3 {
        return None;
    }

    // Walk the perimeter graph: start from any edge and follow next-edges
    // until we close the loop.
    let mut next_perim: HashMap<(i64, i64), DVec3> = HashMap::new();
    for (a, b) in &perimeter_edges {
        next_perim.insert(snap_xy(*a), *b);
    }

    let start = perimeter_edges[0].0;
    let start_key = snap_xy(start);
    let mut loop_verts: Vec<DVec3> = vec![start];
    let mut current = start;
    for _ in 0..perimeter_edges.len() + 1 {
        let key = snap_xy(current);
        let Some(nxt) = next_perim.get(&key).copied() else {
            return None;
        };
        if snap_xy(nxt) == start_key {
            break;
        }
        loop_verts.push(nxt);
        current = nxt;
    }

    if loop_verts.len() < 3 {
        return None;
    }

    // Project all vertices to the exact min_z so the ground is perfectly flat.
    let flat: Vec<DVec3> = loop_verts
        .into_iter()
        .map(|v| DVec3::new(v.x, v.y, min_z))
        .collect();

    Some(flat)
}

/// Legacy convex-hull ground synthesizer (kept for reference, currently unused).
#[allow(dead_code)]
fn synthesize_ground(building: &Building, min_z: f64) -> Surface {
    // Collect all vertices at or near the minimum Z.
    let threshold = min_z + 0.1; // 10cm tolerance
    let mut base_verts: Vec<DVec3> = building
        .surfaces
        .iter()
        .flat_map(|s| s.polygons.iter())
        .flat_map(|p| p.0.iter())
        .filter(|v| v.z <= threshold)
        .map(|v| DVec3::new(v.x, v.y, min_z))
        .collect();

    if base_verts.is_empty() {
        return Surface {
            surface_type: SurfaceType::Ground,
            polygons: Vec::new(),
        };
    }

    // Deduplicate
    base_verts.dedup_by(|a, b| {
        (a.x - b.x).abs() < 0.01 && (a.y - b.y).abs() < 0.01
    });

    // Compute convex hull (simple 2D Graham scan).
    let hull = convex_hull_2d(&base_verts);

    Surface {
        surface_type: SurfaceType::Ground,
        polygons: if hull.len() >= 3 {
            vec![Polygon(hull)]
        } else {
            Vec::new()
        },
    }
}

/// 2D convex hull (Graham scan). Returns CCW-ordered vertices.
fn convex_hull_2d(points: &[DVec3]) -> Vec<DVec3> {
    if points.len() < 3 {
        return points.to_vec();
    }

    let mut pts: Vec<DVec3> = points.to_vec();
    // Find the point with the lowest Y (then lowest X) as pivot.
    let pivot_idx = pts
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            a.y.partial_cmp(&b.y)
                .unwrap()
                .then(a.x.partial_cmp(&b.x).unwrap())
        })
        .map(|(i, _)| i)
        .unwrap();
    pts.swap(0, pivot_idx);
    let pivot = pts[0];

    // Sort by polar angle from pivot.
    pts[1..].sort_by(|a, b| {
        let angle_a = (a.y - pivot.y).atan2(a.x - pivot.x);
        let angle_b = (b.y - pivot.y).atan2(b.x - pivot.x);
        angle_a.partial_cmp(&angle_b).unwrap()
    });

    // Graham scan.
    let mut hull: Vec<DVec3> = Vec::new();
    for p in pts {
        while hull.len() >= 2 {
            let n = hull.len();
            let cross = cross_z(hull[n - 2], hull[n - 1], p);
            if cross <= 0.0 {
                hull.pop();
            } else {
                break;
            }
        }
        hull.push(p);
    }

    hull
}

fn cross_z(o: DVec3, a: DVec3, b: DVec3) -> f64 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

/// Warn if the mesh has boundary edges (non-manifold indicator).
/// If the env var `DEBUG_MANIFOLD` is set, the boundary edge endpoints are dumped.
fn check_manifold(building_id: &str, tris: &[[u32; 3]], _vertex_count: usize) {
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
pub fn dump_boundary_edges(
    building_id: &str,
    verts: &[DVec3],
    tris: &[[u32; 3]],
) {
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

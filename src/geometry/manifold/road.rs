use std::collections::{HashMap, HashSet, VecDeque};

use glam::{DVec2, DVec3};
use spade::handles::{FixedFaceHandle, FixedVertexHandle, InnerTag};
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

use crate::geometry::clip::clip_polygon_to_aabb_xy_2d;
use crate::model::alkis::SurfaceFeature;
use crate::model::terrain::GlobalTerrain;
use crate::model::tile::BBox2;

const ROAD_ABOVE_M: f64 = 0.50;
const ROAD_BELOW_M: f64 = 1.74;

/// Maximum length (m) of a ring edge after densification. Bounding ring edges
/// to ~1 m is what makes the road's vertical sides follow terrain along the
/// road's long direction, and provides one CDT constraint vertex per DGM1
/// pixel along the boundary.
const ROAD_RING_DENSIFY_M: f64 = 1.0;

/// Spacing (m) of interior Steiner points fed into the CDT. Defaults to the
/// DGM1 native resolution so the top/bottom caps follow terrain at source
/// fidelity.
const ROAD_GRID_RES_M: f64 = 1.0;

/// Build a watertight manifold solid for one ALKIS surface feature, restricted
/// to a single tile's XY footprint.
///
/// Pipeline: 2D-clip → densify ring → constrained-Delaunay triangulate with
/// grid-spaced Steiner points → extrude top/bottom + side walls. Per-vertex
/// terrain sampling at the densified ring AND interior grid points means
/// the top cap follows DGM1 terrain at native resolution.
pub fn build_manifold_road(
    feature: &SurfaceFeature,
    terrain: &GlobalTerrain,
    tile_bbox: BBox2,
) -> (Vec<DVec3>, Vec<[u32; 3]>) {
    if feature.exterior.len() < 3 {
        return (Vec::new(), Vec::new());
    }

    let mut ring: Vec<DVec2> = feature.exterior.clone();
    if signed_area_2d(&ring) < 0.0 {
        ring.reverse();
    }

    let clipped = clip_polygon_to_aabb_xy_2d(&ring, tile_bbox);
    if clipped.len() < 3 {
        return (Vec::new(), Vec::new());
    }

    let densified = densify_ring(&clipped, ROAD_RING_DENSIFY_M);
    let n_ring = densified.len();
    if n_ring < 3 {
        return (Vec::new(), Vec::new());
    }

    // Sample terrain at the ring; abort if any ring vertex is in NaN.
    let mut ring_zs: Vec<f64> = Vec::with_capacity(n_ring);
    for p in &densified {
        match terrain.sample_z(p.x, p.y) {
            Some(z) => ring_zs.push(z),
            None => {
                eprintln!(
                    "WARN: skipping road feature '{}' — ring vertex in NaN terrain",
                    feature.id
                );
                return (Vec::new(), Vec::new());
            }
        }
    }

    // Interior Steiner points on a regular grid, point-in-polygon filtered.
    let bb = ring_bbox(&densified);
    let res = ROAD_GRID_RES_M;
    let x0 = (bb.xmin / res).ceil() * res;
    let y0 = (bb.ymin / res).ceil() * res;
    let mut steiner: Vec<DVec2> = Vec::new();
    let mut steiner_zs: Vec<f64> = Vec::new();
    let mut x = x0;
    while x <= bb.xmax {
        let mut y = y0;
        while y <= bb.ymax {
            let p = DVec2::new(x, y);
            if point_in_polygon(p, &densified)
                && let Some(z) = terrain.sample_z(p.x, p.y)
            {
                steiner.push(p);
                steiner_zs.push(z);
            }
            y += res;
        }
        x += res;
    }

    // Build CDT.
    let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> =
        ConstrainedDelaunayTriangulation::new();
    let mut handle_to_idx: HashMap<FixedVertexHandle, usize> = HashMap::new();
    let mut all_pts: Vec<DVec2> = Vec::with_capacity(n_ring + steiner.len());
    let mut all_zs: Vec<f64> = Vec::with_capacity(n_ring + steiner.len());
    let mut ring_handles: Vec<FixedVertexHandle> = Vec::with_capacity(n_ring);

    for (p, z) in densified.iter().zip(ring_zs.iter()) {
        let h = match cdt.insert(Point2::new(p.x, p.y)) {
            Ok(h) => h,
            Err(e) => {
                eprintln!(
                    "WARN: skipping road feature '{}' — CDT ring insert failed: {e:?}",
                    feature.id
                );
                return (Vec::new(), Vec::new());
            }
        };
        handle_to_idx.entry(h).or_insert_with(|| {
            all_pts.push(*p);
            all_zs.push(*z);
            all_pts.len() - 1
        });
        ring_handles.push(h);
    }

    let mut steiner_handles: Vec<FixedVertexHandle> = Vec::with_capacity(steiner.len());
    for (p, z) in steiner.iter().zip(steiner_zs.iter()) {
        let h = match cdt.insert(Point2::new(p.x, p.y)) {
            Ok(h) => h,
            Err(_) => continue, // Skip Steiner points the CDT rejects (e.g. NaN).
        };
        handle_to_idx.entry(h).or_insert_with(|| {
            all_pts.push(*p);
            all_zs.push(*z);
            all_pts.len() - 1
        });
        steiner_handles.push(h);
    }

    // Constrain ring edges. add_constraint may fail if the segment crosses an
    // existing constraint — for a simple ring this only happens with bad data;
    // log and continue with whatever fraction of the ring did register.
    for i in 0..n_ring {
        let j = (i + 1) % n_ring;
        if ring_handles[i] != ring_handles[j] {
            let _ = cdt.add_constraint(ring_handles[i], ring_handles[j]);
        }
    }

    // Classify CDT inner faces as inside-vs-outside the polygon via BFS that
    // refuses to cross constraint (ring) edges. Seed from EVERY Steiner
    // vertex's adjacent faces (Steiner points are inside by construction).
    // Seeding from all of them — not just one — handles polygons that the
    // Sutherland-Hodgman tile clip leaves as multi-component figure-eight
    // rings: each component must get its own seed.
    let mut inside_faces: HashSet<FixedFaceHandle<InnerTag>> = HashSet::new();
    let mut seeds: Vec<FixedFaceHandle<InnerTag>> = Vec::new();
    for sh in &steiner_handles {
        let v = cdt.vertex(*sh);
        for edge in v.out_edges() {
            if let Some(face) = edge.face().as_inner() {
                seeds.push(face.fix());
            }
        }
    }
    if seeds.is_empty() {
        // No Steiner points or none had inner-face neighbors. Fall back to a
        // centroid scan over all inner faces.
        for face in cdt.inner_faces() {
            let [v0, v1, v2] = face.vertices();
            let p0 = v0.position();
            let p1 = v1.position();
            let p2 = v2.position();
            let centroid = DVec2::new(
                (p0.x + p1.x + p2.x) / 3.0,
                (p0.y + p1.y + p2.y) / 3.0,
            );
            if point_in_polygon(centroid, &densified) {
                seeds.push(face.fix());
            }
        }
    }
    let mut queue: VecDeque<FixedFaceHandle<InnerTag>> = VecDeque::new();
    for s in seeds {
        if inside_faces.insert(s) {
            queue.push_back(s);
        }
    }
    while let Some(fh) = queue.pop_front() {
        let face = cdt.face(fh);
        for edge in face.adjacent_edges() {
            if edge.is_constraint_edge() {
                continue;
            }
            if let Some(neighbor) = edge.rev().face().as_inner() {
                let nh = neighbor.fix();
                if inside_faces.insert(nh) {
                    queue.push_back(nh);
                }
            }
        }
    }

    // Build top + bot DVec3 pairs in insertion order (matches handle_to_idx).
    let total_pts = all_pts.len();
    let mut verts: Vec<DVec3> = Vec::with_capacity(2 * total_pts);
    let mut top_idx: Vec<u32> = Vec::with_capacity(total_pts);
    let mut bot_idx: Vec<u32> = Vec::with_capacity(total_pts);
    for (p, z) in all_pts.iter().zip(all_zs.iter()) {
        let ti = verts.len() as u32;
        verts.push(DVec3::new(p.x, p.y, z + ROAD_ABOVE_M));
        let bi = verts.len() as u32;
        verts.push(DVec3::new(p.x, p.y, z - ROAD_BELOW_M));
        top_idx.push(ti);
        bot_idx.push(bi);
    }

    let mut tris: Vec<[u32; 3]> = Vec::new();

    for face in cdt.inner_faces() {
        if !inside_faces.contains(&face.fix()) {
            continue;
        }
        let [v0, v1, v2] = face.vertices();
        let i0 = handle_to_idx[&v0.fix()];
        let i1 = handle_to_idx[&v1.fix()];
        let i2 = handle_to_idx[&v2.fix()];
        let p0 = all_pts[i0];
        let p1 = all_pts[i1];
        let p2 = all_pts[i2];

        // spade returns face vertices in CCW order, so this should always be
        // positive — but defending against degenerate inputs costs nothing.
        let signed = (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
        let (a, b, c) = if signed > 0.0 {
            (i0, i1, i2)
        } else {
            (i0, i2, i1)
        };
        tris.push([top_idx[a], top_idx[b], top_idx[c]]);
        tris.push([bot_idx[a], bot_idx[c], bot_idx[b]]);
    }

    // Side walls along the inside/outside boundary in the CDT — i.e. every
    // edge of an inside face whose neighbor face is outside the polygon.
    // Walking the *CDT's* boundary (rather than the input ring) is essential:
    // when a Steiner point happens to land exactly on a ring constraint, spade
    // splits the constraint into sub-edges, and the top cap uses those
    // sub-edges. The side wall has to follow the same sub-edges or the wall
    // top edges won't share faces with the cap and we get boundary edges.
    for face in cdt.inner_faces() {
        if !inside_faces.contains(&face.fix()) {
            continue;
        }
        for edge in face.adjacent_edges() {
            let neighbor_inside = edge
                .rev()
                .face()
                .as_inner()
                .map(|f| inside_faces.contains(&f.fix()))
                .unwrap_or(false);
            if neighbor_inside {
                continue;
            }
            // edge.from() → edge.to() is CCW around this inside face, so the
            // outside neighbor lies to the right of from→to: the right
            // orientation for an outward-facing wall.
            let [v_from, v_to] = edge.vertices();
            let i_from = handle_to_idx[&v_from.fix()];
            let i_to = handle_to_idx[&v_to.fix()];
            if i_from == i_to {
                continue;
            }
            let ti = top_idx[i_from];
            let tj = top_idx[i_to];
            let bi = bot_idx[i_from];
            let bj = bot_idx[i_to];
            tris.push([ti, bj, tj]);
            tris.push([ti, bi, bj]);
        }
    }

    (verts, tris)
}

fn densify_ring(ring: &[DVec2], max_edge: f64) -> Vec<DVec2> {
    let n = ring.len();
    if n < 2 || max_edge <= 0.0 {
        return ring.to_vec();
    }
    let mut out: Vec<DVec2> = Vec::with_capacity(n);
    for i in 0..n {
        let a = ring[i];
        let b = ring[(i + 1) % n];
        out.push(a);
        let d = (b - a).length();
        if d > max_edge {
            let segs = (d / max_edge).ceil() as usize;
            for k in 1..segs {
                let t = k as f64 / segs as f64;
                out.push(a + (b - a) * t);
            }
        }
    }
    out
}

fn point_in_polygon(p: DVec2, ring: &[DVec2]) -> bool {
    // Standard ray-cast: count crossings of horizontal ray to +x.
    let n = ring.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = ring[i];
        let pj = ring[j];
        let cond = (pi.y > p.y) != (pj.y > p.y);
        if cond {
            let x_cross = pi.x + (p.y - pi.y) * (pj.x - pi.x) / (pj.y - pi.y);
            if p.x < x_cross {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

fn signed_area_2d(ring: &[DVec2]) -> f64 {
    let n = ring.len();
    let mut s = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        s += ring[i].x * ring[j].y - ring[j].x * ring[i].y;
    }
    s * 0.5
}

fn ring_bbox(ring: &[DVec2]) -> BBox2 {
    let mut xmin = f64::INFINITY;
    let mut ymin = f64::INFINITY;
    let mut xmax = f64::NEG_INFINITY;
    let mut ymax = f64::NEG_INFINITY;
    for p in ring {
        if p.x < xmin {
            xmin = p.x;
        }
        if p.y < ymin {
            ymin = p.y;
        }
        if p.x > xmax {
            xmax = p.x;
        }
        if p.y > ymax {
            ymax = p.y;
        }
    }
    BBox2 {
        xmin,
        ymin,
        xmax,
        ymax,
    }
}

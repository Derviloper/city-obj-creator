use std::collections::{HashMap, HashSet, VecDeque};

use glam::{DVec2, DVec3};
use spade::handles::{FixedFaceHandle, FixedVertexHandle, InnerTag};
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

use crate::geometry::clip::clip_polygon_to_aabb_xy_2d;
use crate::model::alkis::{AlkisKind, SurfaceFeature};
use crate::model::terrain::GlobalTerrain;
use crate::model::tile::BBox2;

/// Clearance above the per-region top-percentile terrain sample. Small lift
/// so the water surface clears terrain in the polygon while remaining robust
/// to single tall outliers (bridge pixels, vegetation, data noise).
const WATER_TOP_MARGIN_M: f64 = 0.05;

/// Depth below the lowest terrain sample in a region. Ensures the water solid
/// extends well below the terrain surface everywhere within its footprint, so
/// no air gap appears between water bottom and terrain. Matches ROAD_BELOW_M.
const WATER_BOTTOM_MARGIN_M: f64 = 1.74;

/// Percentile of terrain samples used to set the water surface height.
/// 95th percentile covers nearly all terrain in the region while ignoring
/// a handful of outlier-high samples that would lift the whole surface.
const WATER_TOP_PERCENTILE: f64 = 0.95;

const WATER_RING_DENSIFY_M: f64 = 1.0;
const WATER_GRID_RES_M: f64 = 1.0;

/// Bin width (m) when projecting a river's samples onto its principal axis
/// to decide per-station water levels. Smaller than typical river widths but
/// large enough that each bin has many DGM1 samples on the 1 m Steiner grid.
const RIVER_STATION_BIN_M: f64 = 5.0;

/// Build a watertight manifold solid for one ALKIS water feature, restricted
/// to a single tile's XY footprint.
///
/// Mirrors `build_manifold_road`'s CDT scaffolding: 2D-clip → densify ring →
/// terrain-sample at ring + interior grid points → constrained-Delaunay
/// triangulate → inside/outside BFS → extrude. The only structural difference
/// is the per-vertex z assigned to the top cap: rather than draping every
/// vertex on its terrain sample, we collapse the samples into a flat
/// cross-section (a single level for lakes; a per-along-axis-station level
/// for rivers).
pub fn build_manifold_water(
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

    let densified = densify_ring(&clipped, WATER_RING_DENSIFY_M);
    let n_ring = densified.len();
    if n_ring < 3 {
        return (Vec::new(), Vec::new());
    }

    let mut ring_zs: Vec<f64> = Vec::with_capacity(n_ring);
    for p in &densified {
        match terrain.sample_z(p.x, p.y) {
            Some(z) => ring_zs.push(z),
            None => {
                eprintln!(
                    "WARN: skipping water feature '{}' — ring vertex in NaN terrain",
                    feature.id
                );
                return (Vec::new(), Vec::new());
            }
        }
    }

    // Prepare interior rings (holes) — see build_manifold_road for the rationale.
    let mut interiors_prepared: Vec<(Vec<DVec2>, Vec<f64>)> =
        Vec::with_capacity(feature.interiors.len());
    let mut straddle_warned = false;
    for interior in &feature.interiors {
        if interior.len() < 3 {
            continue;
        }
        let mut iring: Vec<DVec2> = interior.clone();
        if signed_area_2d(&iring) > 0.0 {
            iring.reverse();
        }
        let iclipped = clip_polygon_to_aabb_xy_2d(&iring, tile_bbox);
        if iclipped.len() < 3 {
            continue;
        }
        if iclipped.iter().any(|v| !point_in_polygon(*v, &densified)) {
            if !straddle_warned {
                eprintln!(
                    "WARN: ALKIS feature '{}' has an interior ring straddling tile bbox — hole dropped on this tile",
                    feature.id
                );
                straddle_warned = true;
            }
            continue;
        }
        let idensified = densify_ring(&iclipped, WATER_RING_DENSIFY_M);
        if idensified.len() < 3 {
            continue;
        }
        let mut izs: Vec<f64> = Vec::with_capacity(idensified.len());
        let mut nan_hit = false;
        for p in &idensified {
            match terrain.sample_z(p.x, p.y) {
                Some(z) => izs.push(z),
                None => {
                    eprintln!(
                        "WARN: water feature '{}' — interior ring vertex in NaN terrain; hole dropped",
                        feature.id
                    );
                    nan_hit = true;
                    break;
                }
            }
        }
        if nan_hit {
            continue;
        }
        interiors_prepared.push((idensified, izs));
    }

    let bb = ring_bbox(&densified);
    let res = WATER_GRID_RES_M;
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
                && !interiors_prepared
                    .iter()
                    .any(|(ring, _)| point_in_polygon(p, ring))
                && let Some(z) = terrain.sample_z(p.x, p.y)
            {
                steiner.push(p);
                steiner_zs.push(z);
            }
            y += res;
        }
        x += res;
    }

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
                    "WARN: skipping water feature '{}' — CDT ring insert failed: {e:?}",
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

    let mut interior_handles: Vec<Vec<FixedVertexHandle>> =
        Vec::with_capacity(interiors_prepared.len());
    for (iring, izs) in &interiors_prepared {
        let mut ih: Vec<FixedVertexHandle> = Vec::with_capacity(iring.len());
        let mut broken = false;
        for (p, z) in iring.iter().zip(izs.iter()) {
            match cdt.insert(Point2::new(p.x, p.y)) {
                Ok(h) => {
                    handle_to_idx.entry(h).or_insert_with(|| {
                        all_pts.push(*p);
                        all_zs.push(*z);
                        all_pts.len() - 1
                    });
                    ih.push(h);
                }
                Err(e) => {
                    eprintln!(
                        "WARN: water feature '{}' — CDT interior insert failed: {e:?}; hole dropped",
                        feature.id
                    );
                    broken = true;
                    break;
                }
            }
        }
        if !broken && ih.len() >= 3 {
            interior_handles.push(ih);
        }
    }

    let mut steiner_handles: Vec<FixedVertexHandle> = Vec::with_capacity(steiner.len());
    for (p, z) in steiner.iter().zip(steiner_zs.iter()) {
        let h = match cdt.insert(Point2::new(p.x, p.y)) {
            Ok(h) => h,
            Err(_) => continue,
        };
        handle_to_idx.entry(h).or_insert_with(|| {
            all_pts.push(*p);
            all_zs.push(*z);
            all_pts.len() - 1
        });
        steiner_handles.push(h);
    }

    for i in 0..n_ring {
        let j = (i + 1) % n_ring;
        if ring_handles[i] != ring_handles[j] {
            let _ = cdt.add_constraint(ring_handles[i], ring_handles[j]);
        }
    }

    for ih in &interior_handles {
        let m = ih.len();
        for i in 0..m {
            let j = (i + 1) % m;
            if ih[i] != ih[j] {
                let _ = cdt.add_constraint(ih[i], ih[j]);
            }
        }
    }

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
        for face in cdt.inner_faces() {
            let [v0, v1, v2] = face.vertices();
            let p0 = v0.position();
            let p1 = v1.position();
            let p2 = v2.position();
            let centroid = DVec2::new(
                (p0.x + p1.x + p2.x) / 3.0,
                (p0.y + p1.y + p2.y) / 3.0,
            );
            if point_in_polygon(centroid, &densified)
                && !interiors_prepared
                    .iter()
                    .any(|(ring, _)| point_in_polygon(centroid, ring))
            {
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

    // Replace the per-vertex terrain samples with a flat-cross-section
    // envelope (top, bottom) that brackets all terrain in the region:
    // top above max terrain, bottom below min terrain. Lakes: one envelope
    // for the whole polygon. Rivers: per-bin envelope along the principal
    // axis. Margins are folded into the returned z values.
    let (top_zs, bot_zs): (Vec<f64>, Vec<f64>) = match feature.kind {
        AlkisKind::StehendesGewaesser => {
            let zmin = finite_min(&all_zs);
            let z_top = percentile_finite(&all_zs, WATER_TOP_PERCENTILE);
            let top = z_top + WATER_TOP_MARGIN_M;
            let bot = zmin - WATER_BOTTOM_MARGIN_M;
            (vec![top; all_pts.len()], vec![bot; all_pts.len()])
        }
        AlkisKind::Fliessgewaesser => {
            compute_river_envelope(&all_pts, &all_zs, &feature.exterior, RIVER_STATION_BIN_M)
        }
        _ => unreachable!("build_manifold_water dispatched on non-water AlkisKind"),
    };

    let total_pts = all_pts.len();
    let mut verts: Vec<DVec3> = Vec::with_capacity(2 * total_pts);
    let mut top_idx: Vec<u32> = Vec::with_capacity(total_pts);
    let mut bot_idx: Vec<u32> = Vec::with_capacity(total_pts);
    for (p, (&tz, &bz)) in all_pts.iter().zip(top_zs.iter().zip(bot_zs.iter())) {
        let ti = verts.len() as u32;
        verts.push(DVec3::new(p.x, p.y, tz));
        let bi = verts.len() as u32;
        verts.push(DVec3::new(p.x, p.y, bz));
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

        let signed = (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
        let (a, b, c) = if signed > 0.0 {
            (i0, i1, i2)
        } else {
            (i0, i2, i1)
        };
        tris.push([top_idx[a], top_idx[b], top_idx[c]]);
        tris.push([bot_idx[a], bot_idx[c], bot_idx[b]]);
    }

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

fn finite_min(values: &[f64]) -> f64 {
    let mut zmin = f64::INFINITY;
    for &z in values {
        if z.is_finite() && z < zmin {
            zmin = z;
        }
    }
    if zmin.is_finite() { zmin } else { 0.0 }
}

/// Linearly-interpolated `q`-quantile of finite values in `values`. `q` is
/// clamped to `[0, 1]`; returns 0.0 for an all-non-finite input.
fn percentile_finite(values: &[f64], q: f64) -> f64 {
    let mut v: Vec<f64> = values.iter().copied().filter(|x| x.is_finite()).collect();
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = v.len();
    let rank = (n - 1) as f64 * q.clamp(0.0, 1.0);
    let lo = rank.floor() as usize;
    let hi = (lo + 1).min(n - 1);
    let t = rank - lo as f64;
    v[lo] * (1.0 - t) + v[hi] * t
}

/// Project samples onto the polygon's principal axis (PCA), bin along the
/// axis, and return per-sample `(top_z, bot_z)`: top is `p95(bin) + margin`,
/// bottom is `min(bin) - margin`. Empty bins are filled by linear
/// interpolation between the nearest known neighbours.
///
/// PCA runs on the original exterior ring (not on the dense `pts` array) so
/// the axis follows the river's geometry rather than the uniform 2D
/// distribution of Steiner points, which would bias the axis toward the
/// river's width.
fn compute_river_envelope(
    pts: &[DVec2],
    zs: &[f64],
    ring: &[DVec2],
    bin_width: f64,
) -> (Vec<f64>, Vec<f64>) {
    let n = ring.len() as f64;
    if n < 2.0 || bin_width <= 0.0 {
        let zmin = finite_min(zs);
        let z_top = percentile_finite(zs, WATER_TOP_PERCENTILE);
        return (
            vec![z_top + WATER_TOP_MARGIN_M; pts.len()],
            vec![zmin - WATER_BOTTOM_MARGIN_M; pts.len()],
        );
    }
    let mean = ring.iter().fold(DVec2::ZERO, |a, b| a + *b) / n;
    let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
    for p in ring {
        let d = *p - mean;
        sxx += d.x * d.x;
        sxy += d.x * d.y;
        syy += d.y * d.y;
    }
    sxx /= n;
    sxy /= n;
    syy /= n;
    let trace = sxx + syy;
    let det = sxx * syy - sxy * sxy;
    let disc = (trace * trace * 0.25 - det).max(0.0).sqrt();
    let lambda_max = trace * 0.5 + disc;
    let axis = if sxy.abs() > 1e-12 {
        DVec2::new(lambda_max - syy, sxy).normalize()
    } else if sxx >= syy {
        DVec2::new(1.0, 0.0)
    } else {
        DVec2::new(0.0, 1.0)
    };

    let mut stations: Vec<f64> = Vec::with_capacity(pts.len());
    let (mut smin, mut smax) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in pts {
        let s = (*p - mean).dot(axis);
        stations.push(s);
        if s < smin {
            smin = s;
        }
        if s > smax {
            smax = s;
        }
    }
    if !smin.is_finite() || !smax.is_finite() || smax <= smin {
        let zmin = finite_min(zs);
        let z_top = percentile_finite(zs, WATER_TOP_PERCENTILE);
        return (
            vec![z_top + WATER_TOP_MARGIN_M; pts.len()],
            vec![zmin - WATER_BOTTOM_MARGIN_M; pts.len()],
        );
    }

    let n_bins = (((smax - smin) / bin_width).ceil() as usize).max(1);
    let mut bin_zs: Vec<Vec<f64>> = vec![Vec::new(); n_bins];
    for (s, z) in stations.iter().zip(zs.iter()) {
        if !z.is_finite() {
            continue;
        }
        let mut bi = ((*s - smin) / bin_width).floor() as isize;
        if bi < 0 {
            bi = 0;
        }
        if bi as usize >= n_bins {
            bi = n_bins as isize - 1;
        }
        bin_zs[bi as usize].push(*z);
    }

    let mut bin_min: Vec<Option<f64>> = Vec::with_capacity(n_bins);
    let mut bin_top: Vec<Option<f64>> = Vec::with_capacity(n_bins);
    for v in &bin_zs {
        if v.is_empty() {
            bin_min.push(None);
            bin_top.push(None);
        } else {
            let mut lo = f64::INFINITY;
            for &z in v {
                if z < lo {
                    lo = z;
                }
            }
            bin_min.push(Some(lo));
            bin_top.push(Some(percentile_finite(v, WATER_TOP_PERCENTILE)));
        }
    }
    fill_gaps_linear(&mut bin_min);
    fill_gaps_linear(&mut bin_top);

    let fb_zmin = finite_min(zs);
    let fb_z_top = percentile_finite(zs, WATER_TOP_PERCENTILE);
    let mins: Vec<f64> = bin_min.into_iter().map(|o| o.unwrap_or(fb_zmin)).collect();
    let tops: Vec<f64> = bin_top.into_iter().map(|o| o.unwrap_or(fb_z_top)).collect();

    let mut top_zs: Vec<f64> = Vec::with_capacity(pts.len());
    let mut bot_zs: Vec<f64> = Vec::with_capacity(pts.len());
    for s in &stations {
        let mut bi = ((*s - smin) / bin_width).floor() as isize;
        if bi < 0 {
            bi = 0;
        }
        if bi as usize >= n_bins {
            bi = n_bins as isize - 1;
        }
        top_zs.push(tops[bi as usize] + WATER_TOP_MARGIN_M);
        bot_zs.push(mins[bi as usize] - WATER_BOTTOM_MARGIN_M);
    }
    (top_zs, bot_zs)
}

fn fill_gaps_linear(levels: &mut [Option<f64>]) {
    let n = levels.len();
    let mut last_known: Option<(usize, f64)> = None;
    for i in 0..n {
        if let Some(v) = levels[i] {
            if let Some((j, vj)) = last_known
                && i > j + 1
            {
                for k in (j + 1)..i {
                    let t = (k - j) as f64 / (i - j) as f64;
                    levels[k] = Some(vj * (1.0 - t) + v * t);
                }
            }
            last_known = Some((i, v));
        }
    }
    let first = levels.iter().enumerate().find_map(|(i, v)| v.map(|x| (i, x)));
    if let Some((i, v)) = first {
        for k in 0..i {
            levels[k] = Some(v);
        }
    }
    let last = levels
        .iter()
        .enumerate()
        .rev()
        .find_map(|(i, v)| v.map(|x| (i, x)));
    if let Some((i, v)) = last {
        for k in (i + 1)..n {
            levels[k] = Some(v);
        }
    }
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

use std::{
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result};
use glam::DVec2;
use quick_xml::{
    NsReader,
    events::Event,
    name::ResolveResult,
};

use crate::model::alkis::{AlkisKind, SurfaceFeature};

const ADV_NS: &[u8] = b"http://www.adv-online.de/namespaces/adv/gid/7.1";
const GML_NS: &[u8] = b"http://www.opengis.net/gml";
const GML_NS_32: &[u8] = b"http://www.opengis.net/gml/3.2";

fn is_adv_ns(ns: &[u8]) -> bool {
    ns == ADV_NS
}

fn is_gml_ns(ns: &[u8]) -> bool {
    ns == GML_NS || ns == GML_NS_32
}

/// Map an ALKIS feature element name to its kind. Adding a new ALKIS feature
/// type is one match arm here plus one variant on `AlkisKind`.
fn alkis_kind_from_local(local: &[u8]) -> Option<AlkisKind> {
    match local {
        b"AX_Strassenverkehr" => Some(AlkisKind::Strassenverkehr),
        b"AX_Bahnverkehr" => Some(AlkisKind::Bahnverkehr),
        b"AX_Fliessgewaesser" => Some(AlkisKind::Fliessgewaesser),
        b"AX_StehendesGewaesser" => Some(AlkisKind::StehendesGewaesser),
        _ => None,
    }
}

/// Parse a 2D `gml:posList` text into (x, y) pairs.
fn parse_pos_list_2d(text: &str) -> Vec<DVec2> {
    let nums: Vec<f64> = text
        .split_ascii_whitespace()
        .filter_map(|s| s.parse().ok())
        .collect();
    nums.chunks_exact(2)
        .map(|c| DVec2::new(c[0], c[1]))
        .collect()
}

/// Tolerance (metres) for matching the start of one curve segment to the
/// end of the previous one when concatenating an ALKIS `gml:Ring`.
const RING_JOIN_EPS: f64 = 1e-6;

/// Concatenate ordered `gml:LineStringSegment` posLists into a single ring.
/// Each segment ends where the next begins by design; the duplicate join
/// vertex is dropped. Reverses a segment when it abuts in the opposite
/// orientation. Falls back to verbatim append (with a one-time warning per
/// ring) if neither end matches the running tail.
fn concat_curve_members(segments: Vec<Vec<DVec2>>) -> Vec<DVec2> {
    let mut ring: Vec<DVec2> = Vec::new();
    let mut warned = false;
    for (i, mut seg) in segments.into_iter().enumerate() {
        if seg.len() < 2 {
            continue;
        }
        if i == 0 {
            ring.append(&mut seg);
            continue;
        }
        let tail = *ring.last().unwrap();
        let head_match =
            (seg[0] - tail).length_squared() <= RING_JOIN_EPS * RING_JOIN_EPS;
        let tail_match =
            (*seg.last().unwrap() - tail).length_squared() <= RING_JOIN_EPS * RING_JOIN_EPS;
        if head_match {
            ring.extend(seg.into_iter().skip(1));
        } else if tail_match {
            seg.reverse();
            ring.extend(seg.into_iter().skip(1));
        } else {
            if !warned {
                eprintln!(
                    "WARN: ALKIS curveMember segment #{i} does not abut its predecessor; appending verbatim"
                );
                warned = true;
            }
            ring.extend(seg);
        }
    }
    drop_closing_duplicate(&mut ring);
    ring
}

fn drop_closing_duplicate(ring: &mut Vec<DVec2>) {
    if ring.len() >= 2 {
        let first = ring[0];
        let last = *ring.last().unwrap();
        if (first - last).length_squared() < 1e-10 {
            ring.pop();
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Root,
    InFeature,
    InPatch,
    InExterior,
    InInterior,
    InRing,
    InCurveSeg,
    InPosListSeg,
    InLinearRing,
    InPosListLr,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RingRole {
    Exterior,
    Interior,
}

struct ParserState {
    state: State,
    depth: usize,
    feature_depth: usize,

    feature_id: String,
    feature_kind: Option<AlkisKind>,
    ring_role: Option<RingRole>,

    /// Vertices accumulating for the current `gml:posList`.
    current_segment: Vec<DVec2>,
    /// LineStringSegment posLists waiting to be concatenated into the current `gml:Ring`.
    current_segments: Vec<Vec<DVec2>>,

    exterior: Option<Vec<DVec2>>,
    interiors: Vec<Vec<DVec2>>,
}

impl ParserState {
    fn new() -> Self {
        Self {
            state: State::Root,
            depth: 0,
            feature_depth: 0,
            feature_id: String::new(),
            feature_kind: None,
            ring_role: None,
            current_segment: Vec::new(),
            current_segments: Vec::new(),
            exterior: None,
            interiors: Vec::new(),
        }
    }

    fn finalize_ring(&mut self, ring: Vec<DVec2>) {
        if ring.len() < 3 {
            return;
        }
        match self.ring_role {
            Some(RingRole::Exterior) => self.exterior = Some(ring),
            Some(RingRole::Interior) => self.interiors.push(ring),
            None => {}
        }
    }

    fn ring_parent_state(&self) -> State {
        match self.ring_role {
            Some(RingRole::Exterior) => State::InExterior,
            Some(RingRole::Interior) => State::InInterior,
            None => State::InPatch,
        }
    }
}

/// Stream all supported ALKIS surface features from a single XML file,
/// invoking `visitor` for each. One feature is held in memory at a time.
pub fn stream_alkis_features(
    path: &Path,
    mut visitor: impl FnMut(SurfaceFeature) -> Result<()>,
) -> Result<()> {
    let file = File::open(path)
        .with_context(|| format!("opening ALKIS file: {}", path.display()))?;
    let reader = BufReader::new(file);
    let mut xml = NsReader::from_reader(reader);
    xml.config_mut().trim_text(true);

    let mut ps = ParserState::new();
    let mut buf = Vec::with_capacity(8 * 1024);

    loop {
        buf.clear();
        match xml.read_resolved_event_into(&mut buf) {
            Ok((ns_result, Event::Start(e))) => {
                ps.depth += 1;
                let local = e.local_name();
                let local_bytes = local.as_ref();
                let ns_bytes: &[u8] = match &ns_result {
                    ResolveResult::Bound(ns) => ns.as_ref(),
                    _ => b"",
                };

                // Detect the start of a supported feature element at root scope.
                if matches!(ps.state, State::Root)
                    && is_adv_ns(ns_bytes)
                    && let Some(kind) = alkis_kind_from_local(local_bytes)
                {
                    let mut id = String::new();
                    for attr in e.attributes().flatten() {
                        if attr.key.local_name().as_ref() == b"id" {
                            id = String::from_utf8_lossy(&attr.value).into_owned();
                        }
                    }
                    ps.state = State::InFeature;
                    ps.feature_depth = ps.depth;
                    ps.feature_id = id;
                    ps.feature_kind = Some(kind);
                    ps.exterior = None;
                    ps.interiors.clear();
                    continue;
                }

                match ps.state {
                    State::InFeature => {
                        // Skip wrapper elements (position, gml:Surface, gml:patches);
                        // jump straight in on PolygonPatch.
                        if is_gml_ns(ns_bytes) && local_bytes == b"PolygonPatch" {
                            ps.state = State::InPatch;
                        }
                    }
                    State::InPatch => {
                        if is_gml_ns(ns_bytes) {
                            if local_bytes == b"exterior" {
                                ps.state = State::InExterior;
                                ps.ring_role = Some(RingRole::Exterior);
                                ps.current_segments.clear();
                                ps.current_segment.clear();
                            } else if local_bytes == b"interior" {
                                ps.state = State::InInterior;
                                ps.ring_role = Some(RingRole::Interior);
                                ps.current_segments.clear();
                                ps.current_segment.clear();
                            }
                        }
                    }
                    State::InExterior | State::InInterior => {
                        if is_gml_ns(ns_bytes) {
                            if local_bytes == b"Ring" {
                                ps.state = State::InRing;
                                ps.current_segments.clear();
                            } else if local_bytes == b"LinearRing" {
                                ps.state = State::InLinearRing;
                                ps.current_segment.clear();
                            }
                        }
                    }
                    State::InRing => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"LineStringSegment" {
                            ps.state = State::InCurveSeg;
                            ps.current_segment.clear();
                        }
                    }
                    State::InCurveSeg => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"posList" {
                            ps.state = State::InPosListSeg;
                        }
                    }
                    State::InLinearRing => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"posList" {
                            ps.state = State::InPosListLr;
                        }
                    }
                    _ => {}
                }
            }

            Ok((_, Event::Text(e))) => {
                if matches!(ps.state, State::InPosListSeg | State::InPosListLr) {
                    let text = e.unescape().unwrap_or_default();
                    let new_pts = parse_pos_list_2d(&text);
                    ps.current_segment.extend(new_pts);
                }
            }

            Ok((ns_result, Event::End(e))) => {
                let local = e.local_name();
                let local_bytes = local.as_ref();
                let ns_bytes: &[u8] = match &ns_result {
                    ResolveResult::Bound(ns) => ns.as_ref(),
                    _ => b"",
                };

                match ps.state {
                    State::InPosListSeg => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"posList" {
                            ps.state = State::InCurveSeg;
                        }
                    }
                    State::InPosListLr => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"posList" {
                            ps.state = State::InLinearRing;
                        }
                    }
                    State::InCurveSeg => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"LineStringSegment" {
                            let seg = std::mem::take(&mut ps.current_segment);
                            if seg.len() >= 2 {
                                ps.current_segments.push(seg);
                            }
                            ps.state = State::InRing;
                        }
                    }
                    State::InRing => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"Ring" {
                            let segs = std::mem::take(&mut ps.current_segments);
                            let ring = concat_curve_members(segs);
                            ps.finalize_ring(ring);
                            ps.state = ps.ring_parent_state();
                        }
                    }
                    State::InLinearRing => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"LinearRing" {
                            let mut ring = std::mem::take(&mut ps.current_segment);
                            drop_closing_duplicate(&mut ring);
                            ps.finalize_ring(ring);
                            ps.state = ps.ring_parent_state();
                        }
                    }
                    State::InExterior => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"exterior" {
                            ps.state = State::InPatch;
                            ps.ring_role = None;
                        }
                    }
                    State::InInterior => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"interior" {
                            ps.state = State::InPatch;
                            ps.ring_role = None;
                        }
                    }
                    State::InPatch => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"PolygonPatch" {
                            ps.state = State::InFeature;
                        }
                    }
                    State::InFeature => {
                        if ps.depth == ps.feature_depth
                            && is_adv_ns(ns_bytes)
                            && alkis_kind_from_local(local_bytes).is_some()
                        {
                            if let (Some(kind), Some(exterior)) =
                                (ps.feature_kind, ps.exterior.take())
                                && exterior.len() >= 3
                            {
                                let interiors = std::mem::take(&mut ps.interiors);
                                let id = std::mem::take(&mut ps.feature_id);
                                visitor(SurfaceFeature {
                                    id,
                                    kind,
                                    exterior,
                                    interiors,
                                })?;
                            } else {
                                ps.interiors.clear();
                                ps.feature_id.clear();
                            }
                            ps.feature_kind = None;
                            ps.state = State::Root;
                        }
                    }
                    _ => {}
                }

                ps.depth = ps.depth.saturating_sub(1);
            }

            Ok((_, Event::Eof)) => break,
            Err(e) => {
                return Err(anyhow::anyhow!("XML parse error in {}: {e}", path.display()));
            }
            _ => {}
        }
    }

    Ok(())
}

/// Collect all .xml files in a directory, sorted by name.
pub fn collect_alkis_files(dir: &Path) -> Result<Vec<PathBuf>> {
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .with_context(|| format!("reading ALKIS directory: {}", dir.display()))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| {
            p.is_file()
                && p.extension()
                    .map(|ext| ext.eq_ignore_ascii_case("xml"))
                    .unwrap_or(false)
        })
        .collect();
    files.sort();
    Ok(files)
}

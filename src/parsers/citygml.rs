use std::{
    collections::HashMap,
    fs::File,
    io::BufReader,
    path::Path,
};

use anyhow::{Context, Result};
use glam::DVec3;
use quick_xml::{
    events::Event,
    name::ResolveResult,
    NsReader,
};

use crate::{
    bbox::BBox,
    model::building::{Building, Polygon, Surface, SurfaceType},
};

// CityGML building namespace URIs (versions 1.0 and 2.0).
const BLDG_NS_1: &[u8] = b"http://www.opengis.net/citygml/building/1.0";
const BLDG_NS_2: &[u8] = b"http://www.opengis.net/citygml/building/2.0";
const GML_NS: &[u8] = b"http://www.opengis.net/gml";
const GML_NS_32: &[u8] = b"http://www.opengis.net/gml/3.2";

fn is_bldg_ns(ns: &[u8]) -> bool {
    ns == BLDG_NS_1 || ns == BLDG_NS_2
}

fn is_gml_ns(ns: &[u8]) -> bool {
    ns == GML_NS || ns == GML_NS_32
}

fn surface_type_from_local(local: &[u8]) -> Option<SurfaceType> {
    match local {
        b"RoofSurface" => Some(SurfaceType::Roof),
        b"WallSurface" => Some(SurfaceType::Wall),
        b"GroundSurface" => Some(SurfaceType::Ground),
        b"ClosureSurface" => Some(SurfaceType::Closure),
        _ => None,
    }
}

/// Parse all coordinates from a `posList` or `pos` text string into DVec3 triplets.
/// CityGML repeats the first vertex at the end of a ring — we drop that closing vertex.
fn parse_pos_list(text: &str) -> Vec<DVec3> {
    let nums: Vec<f64> = text
        .split_ascii_whitespace()
        .filter_map(|s| s.parse().ok())
        .collect();

    let mut verts: Vec<DVec3> = nums
        .chunks_exact(3)
        .map(|c| DVec3::new(c[0], c[1], c[2]))
        .collect();

    // Drop the closing duplicate vertex if present.
    if verts.len() >= 2 {
        let first = verts[0];
        let last = *verts.last().unwrap();
        if (first - last).length_squared() < 1e-10 {
            verts.pop();
        }
    }

    verts
}

/// State machine states for the streaming parser.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Root,
    InBuilding,
    InBoundedBy,
    InPolygon,
    InExterior,
    InLinearRing,
    InPosList,
    InPos,
    // We skip interior rings (holes); CityGML building surfaces are typically convex.
    InInterior,
}

/// One open Building or BuildingPart scope. Multiple may be active at once
/// (a BuildingPart nested inside a Building). Surfaces parsed while a scope is
/// the innermost are added to that scope's surface list. When the scope closes,
/// it is emitted as its own `Building` to the visitor — so each BuildingPart
/// becomes its own watertight solid in the output, avoiding internal-wall
/// duplication between sibling parts.
struct ScopeFrame {
    id: String,
    surfaces: Vec<Surface>,
    depth: usize,
}

struct ParserState {
    state: State,
    depth: usize,
    surface_depth: usize,
    polygon_depth: usize,

    /// Stack of open Building / BuildingPart scopes (innermost last).
    scopes: Vec<ScopeFrame>,

    // Currently accumulating surface
    current_surface_type: SurfaceType,
    current_polygons: Vec<Polygon>,

    // Currently accumulating polygon ring vertices
    current_verts: Vec<DVec3>,

    // Shared geometry pool for xlink:href resolution (gml:id → polygon)
    geom_pool: HashMap<String, Vec<DVec3>>,
}

impl ParserState {
    fn new() -> Self {
        Self {
            state: State::Root,
            depth: 0,
            surface_depth: 0,
            polygon_depth: 0,
            scopes: Vec::new(),
            current_surface_type: SurfaceType::Unknown,
            current_polygons: Vec::new(),
            current_verts: Vec::new(),
            geom_pool: HashMap::new(),
        }
    }

    fn finish_ring(&mut self) {
        let verts = std::mem::take(&mut self.current_verts);
        if verts.len() >= 3 {
            self.current_polygons.push(Polygon(verts));
        }
    }

    fn finish_surface(&mut self) {
        let polygons = std::mem::take(&mut self.current_polygons);
        if polygons.is_empty() {
            return;
        }
        if let Some(top) = self.scopes.last_mut() {
            top.surfaces.push(Surface {
                surface_type: self.current_surface_type,
                polygons,
            });
        }
    }

    /// Pop the top scope and convert it into a Building.
    fn pop_scope(&mut self) -> Option<Building> {
        let frame = self.scopes.pop()?;
        Some(Building {
            id: frame.id,
            surfaces: frame.surfaces,
        })
    }
}

/// Stream all buildings from a CityGML file, invoking `visitor` for each complete building.
/// Only one building is held in memory at a time.
pub fn stream_buildings(
    path: &Path,
    bbox: Option<BBox>,
    mut visitor: impl FnMut(Building) -> Result<()>,
) -> Result<()> {
    let file = File::open(path)
        .with_context(|| format!("opening CityGML file: {}", path.display()))?;
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

                // Detect Building / BuildingPart at any state outside surface parsing.
                // A new scope is pushed and we (re-)enter the InBuilding state.
                let is_scope_start = is_bldg_ns(ns_bytes)
                    && (local_bytes == b"Building" || local_bytes == b"BuildingPart")
                    && matches!(ps.state, State::Root | State::InBuilding);
                if is_scope_start {
                    let mut id = String::new();
                    for attr in e.attributes().flatten() {
                        if attr.key.local_name().as_ref() == b"id" {
                            id = String::from_utf8_lossy(&attr.value).into_owned();
                        }
                    }
                    ps.scopes.push(ScopeFrame {
                        id,
                        surfaces: Vec::new(),
                        depth: ps.depth,
                    });
                    ps.state = State::InBuilding;
                    continue;
                }

                match ps.state {
                    State::Root => {}
                    State::InBuilding => {
                        if is_bldg_ns(ns_bytes)
                            && let Some(st) = surface_type_from_local(local_bytes)
                        {
                            ps.state = State::InBoundedBy;
                            ps.current_surface_type = st;
                            ps.surface_depth = ps.depth;
                        }
                    }
                    State::InBoundedBy => {
                        if is_bldg_ns(ns_bytes)
                            && let Some(st) = surface_type_from_local(local_bytes)
                        {
                            ps.current_surface_type = st;
                        }
                        if is_gml_ns(ns_bytes) && local_bytes == b"Polygon" {
                            ps.state = State::InPolygon;
                            ps.polygon_depth = ps.depth;
                        }
                    }
                    State::InPolygon => {
                        if is_gml_ns(ns_bytes) {
                            if local_bytes == b"exterior" {
                                ps.state = State::InExterior;
                            } else if local_bytes == b"interior" {
                                ps.state = State::InInterior;
                            }
                        }
                    }
                    State::InExterior => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"LinearRing" {
                            ps.state = State::InLinearRing;
                        }
                    }
                    State::InLinearRing => {
                        if is_gml_ns(ns_bytes) {
                            if local_bytes == b"posList" {
                                ps.state = State::InPosList;
                            } else if local_bytes == b"pos" {
                                ps.state = State::InPos;
                            }
                        }
                    }
                    State::InInterior | State::InPosList | State::InPos => {}
                }
            }

            Ok((ns_result, Event::Empty(e))) => {
                // Handle self-closing tags (e.g., surfaceMember with xlink:href)
                let local = e.local_name();
                let local_bytes = local.as_ref();
                let ns_bytes: &[u8] = match &ns_result {
                    ResolveResult::Bound(ns) => ns.as_ref(),
                    _ => b"",
                };

                if is_gml_ns(ns_bytes) && local_bytes == b"surfaceMember" {
                    // Resolve xlink:href
                    for attr in e.attributes().flatten() {
                        let alocal = attr.key.local_name();
                        if alocal.as_ref() == b"href" {
                            let href = String::from_utf8_lossy(&attr.value).into_owned();
                            let id = href.trim_start_matches('#').to_string();
                            if let Some(verts) = ps.geom_pool.get(&id)
                                && verts.len() >= 3
                            {
                                ps.current_polygons.push(Polygon(verts.clone()));
                            }
                        }
                    }
                }
            }

            Ok((_, Event::Text(e))) => {
                let text = e.unescape().unwrap_or_default();
                match ps.state {
                    State::InPosList | State::InPos => {
                        let new_verts = parse_pos_list(&text);
                        ps.current_verts.extend(new_verts);
                    }
                    _ => {}
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
                    State::InPosList | State::InPos => {
                        if is_gml_ns(ns_bytes)
                            && (local_bytes == b"posList" || local_bytes == b"pos")
                        {
                            ps.state = State::InLinearRing;
                        }
                    }
                    State::InLinearRing => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"LinearRing" {
                            ps.finish_ring();
                            ps.state = State::InExterior;
                        }
                    }
                    State::InExterior => {
                        if is_gml_ns(ns_bytes) && local_bytes == b"exterior" {
                            ps.state = State::InPolygon;
                        }
                    }
                    State::InInterior => {
                        if ps.depth == ps.polygon_depth + 1 {
                            // Leaving interior element
                            ps.state = State::InPolygon;
                        }
                    }
                    State::InPolygon => {
                        if ps.depth == ps.polygon_depth {
                            ps.state = State::InBoundedBy;
                        }
                    }
                    State::InBoundedBy => {
                        if ps.depth == ps.surface_depth {
                            ps.finish_surface();
                            ps.state = State::InBuilding;
                        }
                    }
                    State::InBuilding => {
                        // Close the innermost scope when we reach its element's End.
                        if let Some(top_depth) = ps.scopes.last().map(|s| s.depth)
                            && ps.depth == top_depth
                            && is_bldg_ns(ns_bytes)
                            && (local_bytes == b"Building" || local_bytes == b"BuildingPart")
                        {
                            if let Some(building) = ps.pop_scope() {
                                let in_bbox = bbox.is_none_or(|bb| {
                                    building.bounds().is_some_and(|(bmin, bmax)| {
                                        let tile_bb = BBox {
                                            xmin: bmin.x,
                                            ymin: bmin.y,
                                            xmax: bmax.x,
                                            ymax: bmax.y,
                                        };
                                        bb.intersects(&tile_bb)
                                    })
                                });
                                if in_bbox && !building.surfaces.is_empty() {
                                    visitor(building)?;
                                }
                            }
                            // If outer scope still open, stay in InBuilding;
                            // otherwise return to Root.
                            if ps.scopes.is_empty() {
                                ps.state = State::Root;
                            }
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

/// Collect all .gml and .xml files in a directory, sorted by name.
pub fn collect_gml_files(dir: &Path) -> Result<Vec<std::path::PathBuf>> {
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .with_context(|| format!("reading CityGML directory: {}", dir.display()))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| {
            p.is_file()
                && p.extension()
                    .map(|ext| ext.eq_ignore_ascii_case("gml") || ext.eq_ignore_ascii_case("xml"))
                    .unwrap_or(false)
        })
        .collect();
    files.sort();
    Ok(files)
}

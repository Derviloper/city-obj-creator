## city-obj-creator — codebase summary

A Rust CLI that converts German cadastral 3D data — **CityGML LoD2** building models + **DGM1 GeoTIFF** elevation tiles — into watertight, manifold **Wavefront OBJ** meshes, partitioned per terrain tile.

### Entry point

- [src/main.rs](src/main.rs) — orchestrates the pipeline. Two phases:
  1. **Terrain** ([main.rs:67](src/main.rs#L67)): one `terrain.obj` per DGM1 tile, written into its own subfolder.
  2. **Buildings** ([main.rs:108](src/main.rs#L108)): streams every building, builds a manifold mesh, clips it to each overlapping tile's bbox, and appends to that tile's `buildings.obj`. Sliver pieces below `MIN_PIECE_AREA_M2` (1e-3 m²) are skipped.
- [src/cli.rs](src/cli.rs) — clap CLI with `--citygml`, `--dgm1`, `--output` (default `output/`).

### Module layout

- **[model/](src/model/)** — domain types
  - [tile.rs](src/model/tile.rs): `Tile` (DGM1 metadata), `BBox2` with `intersects` / `contains_bbox`.
  - [building.rs](src/model/building.rs): `Building`, `Surface`, `Polygon`, `SurfaceType` (Roof/Wall/Ground/Closure/Unknown).
  - [terrain.rs](src/model/terrain.rs): `GlobalTerrain` elevation grid (NaN = no data).
- **[parsers/](src/parsers/)**
  - [citygml.rs](src/parsers/citygml.rs): streaming `quick-xml` parser supporting CityGML 1.0 / 2.0 + GML 3 / 3.2 namespaces; drops the closing duplicate vertex; skips interior rings.
  - [dgm1.rs](src/parsers/dgm1.rs): reads single-band float GeoTIFFs (EPSG:25832), honors GDAL nodata tag, prefers `ModelTransformationTag`.
- **[geometry/](src/geometry/)**
  - [clip.rs](src/geometry/clip.rs): four sequential half-space clips with cap re-triangulation so each cut piece stays watertight.
  - [normals.rs](src/geometry/normals.rs): ear-clip triangulator.
  - [vertex_merge.rs](src/geometry/vertex_merge.rs): coincident-vertex deduplication.
  - **[manifold/](src/geometry/manifold/)** — the heart of the project
    - [building.rs](src/geometry/manifold/building.rs): 9-stage cleanup pipeline (snap → cluster-merge → split self-touching rings → dedup → T-junction insertion → drop redundant polys → drop spurious internal floors → close open boundary loops → triangulate + repair + orient).
    - [terrain.rs](src/geometry/manifold/terrain.rs): top surface + vertical skirt down to `Z_BASE = 250.0` + bottom cap, producing one closed solid per tile.
    - [cleanup.rs](src/geometry/manifold/cleanup.rs) (548 LOC): bulk of the LVermGeo data-quirk fixers.
    - [repair.rs](src/geometry/manifold/repair.rs): over-shared-edge repair, BFS winding propagation, signed-volume outward orientation.
    - [check.rs](src/geometry/manifold/check.rs): manifoldness assertion.
- **[export/obj.rs](src/export/obj.rs)** — streaming OBJ writer; vertices written relative to `tile.obj_origin()` so coordinates stay small while a header comment records the UTM origin (CRS ETRS89/UTM32N, EPSG:25832).

### Notable design choices

- **Streaming everywhere**: GML files are parsed building-by-building; OBJ writers stay open per tile and accept incremental object groups.
- **Per-tile output partitioning**: tiles drive the directory structure; a building straddling tile borders is split with watertight caps, not duplicated whole.
- **Heavy data-quality handling**: extensive comments document concrete LVermGeo encoding bugs (duplicate WallSurface/ClosureSurface, lasso-shaped roof rings, internal "floor" GroundSurfaces on bridges, etc.) and the targeted heuristics that fix them.
- **Tolerances are tuned, not arbitrary**: 5 mm primary snap, 5 cm xy / 2 cm z cluster merge — chosen so wall corners (same xy, very different z) survive.
- **Test data**: Data to test against is available in `data/`
- **Important**:
  - All meshes need to be manifold
  - The programm needs to handle large files, so it needs to be memory efficient

use clap::Parser;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(
    name = "city-obj-creator",
    about = "Convert LoD2 CityGML + DGM1 GeoTIFF data to Wavefront OBJ 3D models"
)]
pub struct Cli {
    /// Directory containing CityGML LoD2 .gml / .xml files
    #[arg(long)]
    pub citygml: PathBuf,

    /// Directory containing DGM1 .tif GeoTIFF elevation files. Required:
    /// each tile becomes its own output sub-folder, and tiles also drive
    /// how buildings are partitioned into folders.
    #[arg(long)]
    pub dgm1: PathBuf,

    /// Optional directory containing ALKIS .xml files (AX_Bestandsdatenauszug).
    /// When set, a `road.obj` is emitted per tile from AX_Strassenverkehr features.
    #[arg(long)]
    pub alkis: Option<PathBuf>,

    /// Output directory for .obj files (created if it doesn't exist)
    #[arg(long, short, default_value = "output")]
    pub output: PathBuf,
}

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

    /// Directory containing DGM1 .tif GeoTIFF elevation files
    #[arg(long)]
    pub dgm1: Option<PathBuf>,

    /// Output directory for .obj files (created if it doesn't exist)
    #[arg(long, short, default_value = "output")]
    pub output: PathBuf,

    /// Bounding box filter: xmin,ymin,xmax,ymax in ETRS89/UTM32N (EPSG:25832)
    #[arg(long)]
    pub bbox: Option<String>,

    /// Only output buildings whose gml:id matches one of these (debug aid).
    #[arg(long)]
    pub building_id: Vec<String>,
}

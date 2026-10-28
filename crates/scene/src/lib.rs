//! `scene` — scene description loading (JSON exported by tools/osm_to_scene.py) and mesh generation.
pub mod inflate;
pub mod json;
pub mod mesh;
pub mod model;

pub use mesh::{car_meshes, ground_mesh, scene_meshes, scene_meshes_opts, MeshData};
pub use model::{Building, Facade, Road, Scene};

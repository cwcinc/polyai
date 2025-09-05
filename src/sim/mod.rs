pub mod part_manager;
pub mod track;

use crate::sim::part_manager::{PartData, PartManager};

pub struct SimState {
    pub part_manager: PartManager,
}
impl SimState {
    pub fn new(track_parts_json: &str, car_collision_shape_vertices_json: &str) -> Self {
        let track_parts: Vec<PartData> = serde_json::from_str(track_parts_json).unwrap();
        let car_collision_shape_vertices: Vec<f64> =
            serde_json::from_str(car_collision_shape_vertices_json).unwrap();

        let part_manager = PartManager::new(track_parts);
        Self { part_manager }
    }
}

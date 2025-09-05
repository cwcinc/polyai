use bbox::BoundingBox;
use cxx::UniquePtr;
use nalgebra::{Point3, Vector3};
use serde::Deserialize;

use crate::ammo::*;

#[derive(Deserialize)]
pub struct PartDataDetector {
    #[serde(rename = "type")]
    type_: usize,
    center: [f32; 3],
    size: [f32; 3],
}

#[derive(Deserialize)]
pub struct PartData {
    pub id: usize,
    pub vertices: Vec<f32>,
    pub detector: Option<PartDataDetector>,
    pub start_offset: Option<[f32; 3]>,
}

pub struct PartDetector {
    pub type_: usize,
    pub center: Vector3<f32>,
    pub size: Vector3<f32>,
}

pub struct Part {
    pub id: usize,
    pub bounding_box: BoundingBox<f32>,
    pub shape: UniquePtr<btBvhTriangleMeshShape>,
    pub triangle_mesh: UniquePtr<btTriangleMesh>,
    pub detector: Option<PartDetector>,
    pub start_offset: Option<Vector3<f32>>,
}

pub struct PartManager {
    pub track_parts: Vec<Part>,
}
impl PartManager {
    pub fn new(track_parts: Vec<PartData>) -> Self {
        let mut parts = Vec::new();
        for part in track_parts {
            if part.vertices.len() % 9 != 0 {
                panic!("Physics shape vertices length is not dividable by 9");
            }

            let mut bounding_box = BoundingBox::neg_infinity();
            let mut triangle_mesh = bt_triangle_mesh_new(None, None);
            for i in (0..part.vertices.len()).step_by(9) {
                let v1 = Point3::new(part.vertices[i], part.vertices[i + 1], part.vertices[i + 2]);
                let v2 = Point3::new(
                    part.vertices[i + 3],
                    part.vertices[i + 4],
                    part.vertices[i + 5],
                );
                let v3 = Point3::new(
                    part.vertices[i + 6],
                    part.vertices[i + 7],
                    part.vertices[i + 8],
                );
                let v1_bt = bt_vector3_new1(v1.x, v1.y, v1.z);
                let v2_bt = bt_vector3_new1(v2.x, v2.y, v2.z);
                let v3_bt = bt_vector3_new1(v3.x, v3.y, v3.z);
                bt_triangle_mesh_add_triangle(&mut triangle_mesh, &v1_bt, &v2_bt, &v3_bt, None);
                bounding_box.insert(&v1);
                bounding_box.insert(&v2);
                bounding_box.insert(&v3);
            }
            let mut triangle_mesh_shape =
                bt_bvh_triangle_mesh_shape_new(triangle_mesh.cast_mut(), false, None);
            bt_bvh_triangle_mesh_shape_set_margin(&mut triangle_mesh_shape, 0.01);
            parts.push(Part {
                id: part.id,
                bounding_box,
                shape: triangle_mesh_shape,
                triangle_mesh,
                detector: part.detector.map(|d| PartDetector {
                    type_: d.type_,
                    center: Vector3::from(d.center),
                    size: Vector3::from(d.size),
                }),
                start_offset: part.start_offset.map(Vector3::from),
            });
        }

        Self { track_parts: parts }
    }
}

use autocxx::WithinUniquePtr;
use cxx::UniquePtr;

pub use crate::ammo_sys::*;

pub trait Inherit<T> {}
impl<T> Inherit<T> for T {}

macro_rules! impl_inherit {
    ($child:ty : $parent:ty) => {
        impl<Ancestor> self::Inherit<Ancestor> for $child where $parent: self::Inherit<Ancestor> {}
    };
}

pub trait Cast<T> {
    fn cast(&self) -> &T;
    fn cast_mut(&mut self) -> &mut T;
}
impl<T, U> Cast<U> for T
where
    T: Inherit<U>,
{
    fn cast(&self) -> &U {
        unsafe { &*(self as *const T as *const U) }
    }

    fn cast_mut(&mut self) -> &mut U {
        unsafe { &mut *(self as *mut T as *mut U) }
    }
}

pub fn bt_triangle_mesh_new(
    use_32bit_indices: Option<bool>,
    use_4component_vertices: Option<bool>,
) -> UniquePtr<btTriangleMesh> {
    btTriangleMesh::new(
        use_32bit_indices.unwrap_or(true),
        use_4component_vertices.unwrap_or(true),
    )
    .within_unique_ptr()
}
pub fn bt_triangle_mesh_add_triangle(
    bt_triangle_mesh: &mut UniquePtr<btTriangleMesh>,
    vertex0: &UniquePtr<btVector3>,
    vertex1: &UniquePtr<btVector3>,
    vertex2: &UniquePtr<btVector3>,
    remove_duplicate_vertices: Option<bool>,
) {
    bt_triangle_mesh.pin_mut().addTriangle(
        vertex0.as_ref().unwrap(),
        vertex1.as_ref().unwrap(),
        vertex2.as_ref().unwrap(),
        remove_duplicate_vertices.unwrap_or(false),
    );
}
impl_inherit!(UniquePtr<btTriangleMesh> : UniquePtr<btStridingMeshInterface>);

pub fn bt_vector3_new1(_x: f32, _y: f32, _z: f32) -> UniquePtr<btVector3> {
    btVector3::new1(&_x, &_y, &_z).within_unique_ptr()
}

pub fn bt_bvh_triangle_mesh_shape_new(
    mesh_interface: &mut UniquePtr<btStridingMeshInterface>,
    use_quantized_aabb: bool,
    build_bvh: Option<bool>,
) -> UniquePtr<btBvhTriangleMeshShape> {
    unsafe {
        btBvhTriangleMeshShape::new(
            mesh_interface.as_mut_ptr(),
            use_quantized_aabb,
            build_bvh.unwrap_or(true),
        )
        .within_unique_ptr()
    }
}
pub fn bt_bvh_triangle_mesh_shape_set_margin(
    bt_bvh_triangle_mesh_shape: &mut UniquePtr<btBvhTriangleMeshShape>,
    collision_margin: f32,
) {
    let bt_concave_shape = Cast::<UniquePtr<btConcaveShape>>::cast_mut(bt_bvh_triangle_mesh_shape);
    btConcaveShape::setMargin(bt_concave_shape.pin_mut(), collision_margin);
}
impl_inherit!(UniquePtr<btBvhTriangleMeshShape> : UniquePtr<btConcaveShape>);

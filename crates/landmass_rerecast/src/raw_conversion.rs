use bevy_landmass::{HeightNavigationMesh3d, HeightPolygon, NavigationMesh3d};
use bevy_math::Vec3;
use bevy_rerecast_core::{
  Navmesh as RerecastNavMesh, rerecast::PolygonNavmesh,
};

/// Converts a [`RerecastNavMesh`] into a raw, unvalidated landmass nav mesh.
pub fn convert_rerecast_navmesh_to_landmass_navmesh(
  rerecast_navmesh: &RerecastNavMesh,
) -> bevy_landmass::NavigationMesh3d {
  let orig = rerecast_navmesh.polygon.aabb.min;
  let cs = rerecast_navmesh.polygon.cell_size;
  let ch = rerecast_navmesh.polygon.cell_height;
  let to_local = Vec3::new(cs, ch, cs);
  let nvp = rerecast_navmesh.polygon.max_vertices_per_polygon as usize;

  NavigationMesh3d {
    vertices: rerecast_navmesh
      .polygon
      .vertices
      .iter()
      .map(|v| orig + v.as_vec3() * to_local)
      .collect(),
    polygons: (0..rerecast_navmesh.polygon.polygon_count())
      .map(|i| {
        rerecast_navmesh.polygon.polygons[i * nvp..][..nvp]
          .iter()
          .filter(|i| **i != PolygonNavmesh::NO_INDEX)
          .map(|i| *i as usize)
          .collect::<Vec<_>>()
      })
      .collect(),
    polygon_type_indices: rerecast_navmesh
      .polygon
      .areas
      .iter()
      .map(|a| a.0 as usize)
      .collect(),
    height_mesh: HeightNavigationMesh3d {
      polygons: rerecast_navmesh
        .detail
        .meshes
        .iter()
        .map(|submesh| HeightPolygon {
          base_vertex_index: submesh.base_vertex_index,
          vertex_count: submesh.vertex_count,
          base_triangle_index: submesh.base_triangle_index,
          triangle_count: submesh.triangle_count,
        })
        .collect(),
      triangles: rerecast_navmesh.detail.triangles.clone(),
      vertices: rerecast_navmesh.detail.vertices.clone(),
    }
    .into(),
  }
}

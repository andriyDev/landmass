use bevy_landmass::HeightPolygon;
use bevy_math::{U16Vec3, Vec3};
use bevy_rerecast_core::{
  Navmesh as RerecastNavMesh,
  rerecast::{Aabb3d, AreaType, DetailNavmesh, PolygonNavmesh, SubMesh},
};
use googletest::{expect_that, matchers::elements_are};

use crate::convert_rerecast_navmesh_to_landmass_navmesh;

#[googletest::test]
fn converts_mesh() {
  // The map looks something like this:
  //   up +-+ +-+ down
  //      |  V  |
  //      +--+--+
  //     /       \
  //  +-+         +
  //  |\|         |
  //  +-+---------+
  const N: u16 = PolygonNavmesh::NO_INDEX;
  let rerecast_mesh = RerecastNavMesh {
    polygon: PolygonNavmesh {
      cell_size: 2.0,
      cell_height: 3.0,
      aabb: Aabb3d {
        min: Vec3::new(1.0, 2.0, 3.0),
        max: Vec3::new(100.0, 100.0, 100.0),
      },
      max_vertices_per_polygon: 7,
      vertices: vec![
        U16Vec3::new(0, 5, 0),
        U16Vec3::new(1, 5, 0),
        U16Vec3::new(1, 5, 1),
        U16Vec3::new(0, 5, 1),
        //
        U16Vec3::new(7, 5, 0),
        U16Vec3::new(7, 5, 1),
        U16Vec3::new(6, 5, 2),
        U16Vec3::new(4, 5, 2),
        U16Vec3::new(2, 5, 2),
        // down
        U16Vec3::new(6, 1, 3),
        U16Vec3::new(5, 1, 3),
        // up
        U16Vec3::new(3, 7, 3),
        U16Vec3::new(2, 7, 3),
      ],
      polygons: vec![
        0, 1, 3, N, N, N, N, //
        1, 2, 3, N, N, N, N, //
        1, 4, 5, 6, 7, 8, 2, //
        7, 6, 9, 10, N, N, N, //
        8, 7, 11, 12, N, N, N, //
      ],
      areas: vec![
        AreaType(1),
        AreaType(2),
        AreaType(2),
        AreaType(255),
        AreaType(123),
      ],
      // We compute our own neighbours and regions so we don't use these.
      polygon_neighbors: vec![],
      regions: vec![],
      // We don't care about these.
      flags: vec![],
      border_size: 0,
      max_edge_error: 0.0,
    },
    // This detail mesh doesn't correspond to the actual mesh, but it should
    // still be converted.
    detail: DetailNavmesh {
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 2.0),
        Vec3::new(1.0, 1.0, 2.0),
      ],
      triangles: vec![[0, 1, 2], [2, 3, 0]],
      meshes: vec![SubMesh {
        base_vertex_index: 0,
        vertex_count: 4,
        base_triangle_index: 0,
        triangle_count: 2,
      }],
      triangle_flags: vec![],
    },
    settings: Default::default(),
  };

  let landmass_mesh =
    convert_rerecast_navmesh_to_landmass_navmesh(&rerecast_mesh);
  expect_that!(
    landmass_mesh.vertices,
    elements_are!(
      &Vec3::new(1.0 + 0.0 * 2.0, 2.0 + 5.0 * 3.0, 3.0 + 0.0 * 2.0),
      &Vec3::new(1.0 + 1.0 * 2.0, 2.0 + 5.0 * 3.0, 3.0 + 0.0 * 2.0),
      &Vec3::new(1.0 + 1.0 * 2.0, 2.0 + 5.0 * 3.0, 3.0 + 1.0 * 2.0),
      &Vec3::new(1.0 + 0.0 * 2.0, 2.0 + 5.0 * 3.0, 3.0 + 1.0 * 2.0),
      //
      &Vec3::new(1.0 + 7.0 * 2.0, 2.0 + 5.0 * 3.0, 3.0 + 0.0 * 2.0),
      &Vec3::new(1.0 + 7.0 * 2.0, 2.0 + 5.0 * 3.0, 3.0 + 1.0 * 2.0),
      &Vec3::new(1.0 + 6.0 * 2.0, 2.0 + 5.0 * 3.0, 3.0 + 2.0 * 2.0),
      &Vec3::new(1.0 + 4.0 * 2.0, 2.0 + 5.0 * 3.0, 3.0 + 2.0 * 2.0),
      &Vec3::new(1.0 + 2.0 * 2.0, 2.0 + 5.0 * 3.0, 3.0 + 2.0 * 2.0),
      // down
      &Vec3::new(1.0 + 6.0 * 2.0, 2.0 + 1.0 * 3.0, 3.0 + 3.0 * 2.0),
      &Vec3::new(1.0 + 5.0 * 2.0, 2.0 + 1.0 * 3.0, 3.0 + 3.0 * 2.0),
      // up
      &Vec3::new(1.0 + 3.0 * 2.0, 2.0 + 7.0 * 3.0, 3.0 + 3.0 * 2.0),
      &Vec3::new(1.0 + 2.0 * 2.0, 2.0 + 7.0 * 3.0, 3.0 + 3.0 * 2.0),
    )
  );
  expect_that!(
    landmass_mesh.polygons,
    elements_are!(
      elements_are!(&0, &1, &3),
      elements_are!(&1, &2, &3),
      elements_are!(&1, &4, &5, &6, &7, &8, &2),
      elements_are!(&7, &6, &9, &10),
      elements_are!(&8, &7, &11, &12),
    )
  );
  expect_that!(
    landmass_mesh.polygon_type_indices,
    elements_are!(&1, &2, &2, &255, &123)
  );

  let Some(height_mesh) = landmass_mesh.height_mesh.as_ref() else {
    panic!("The height mesh should have been converted");
  };
  expect_that!(
    height_mesh.vertices,
    elements_are!(
      &Vec3::new(1.0, 1.0, 1.0),
      &Vec3::new(2.0, 1.0, 1.0),
      &Vec3::new(2.0, 1.0, 2.0),
      &Vec3::new(1.0, 1.0, 2.0),
    )
  );
  expect_that!(height_mesh.triangles, elements_are!(&[0, 1, 2], &[2, 3, 0]));
  expect_that!(
    height_mesh.polygons,
    elements_are!(&HeightPolygon {
      base_vertex_index: 0,
      vertex_count: 4,
      base_triangle_index: 0,
      triangle_count: 2,
    })
  );
}

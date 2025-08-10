use std::collections::HashSet;

use glam::Vec3;

use crate::{
  PointSampleDistance3d,
  coords::XYZ,
  nav_mesh::{
    Connectivity, HeightNavigationMesh, HeightPolygon, MeshEdgeRef,
    ValidPolygon,
  },
  util::BoundingBox,
};

use super::{NavigationMesh, ValidationError};

#[test]
fn validation_computes_bounds() {
  let source_mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(2.0, 1.0, 0.0),
      Vec3::new(0.5, 3.0, 0.5),
      Vec3::new(0.75, 4.0, -0.25),
      Vec3::new(0.25, 4.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2], vec![3, 4, 5]],
    polygon_type_indices: vec![0, 0],
    height_mesh: None,
  };

  let valid_mesh =
    source_mesh.clone().validate().expect("Validation succeeds.");
  assert_eq!(
    valid_mesh.mesh_bounds,
    BoundingBox::new_box(Vec3::new(0.0, 0.0, -0.25), Vec3::new(2.0, 4.0, 1.0))
  );
}

#[test]
fn correctly_computes_bounds_for_small_number_of_points() {
  let valid_mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(-1.0, -1.0, -1.0),
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(2.0, 1.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2]],
    polygon_type_indices: vec![0],
    height_mesh: None,
  }
  .validate()
  .expect("Validation succeeds");

  assert_eq!(
    valid_mesh.mesh_bounds,
    BoundingBox::new_box(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(2.0, 1.0, 1.0))
  );
}

#[test]
fn polygons_derived_and_vertices_copied() {
  let source_mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(2.0, 1.0, 0.0),
      Vec3::new(0.5, 3.0, 0.5),
      Vec3::new(0.75, 4.0, -0.25),
      Vec3::new(0.25, 4.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2], vec![3, 4, 5]],
    polygon_type_indices: vec![1337, 123],
    height_mesh: None,
  };

  let expected_polygons = vec![
    ValidPolygon {
      vertices: source_mesh.polygons[0].clone(),
      connectivity: vec![None, None, None],
      region: 0,
      type_index: 1337,
      bounds: BoundingBox::new_box(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0, 1.0, 1.0),
      ),
      center: Vec3::new(3.0, 1.0, 1.0) / 3.0,
    },
    ValidPolygon {
      vertices: source_mesh.polygons[1].clone(),
      connectivity: vec![None, None, None],
      region: 1,
      type_index: 123,
      bounds: BoundingBox::new_box(
        Vec3::new(0.25, 3.0, -0.25),
        Vec3::new(0.75, 4.0, 0.5),
      ),
      center: Vec3::new(1.5, 11.0, 0.25) / 3.0,
    },
  ];

  let valid_mesh =
    source_mesh.clone().validate().expect("Validation succeeds.");
  assert_eq!(valid_mesh.vertices, source_mesh.vertices);
  assert_eq!(valid_mesh.polygons, expected_polygons);
  assert_eq!(valid_mesh.used_type_indices, HashSet::from([1337, 123]));
}

#[test]
fn error_on_wrong_type_indices_length() {
  let source_mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2]],
    polygon_type_indices: vec![0, 0],
    height_mesh: None,
  };

  let error = source_mesh
    .clone()
    .validate()
    .expect_err("Wrong type indices length should be detected.");
  match error {
    ValidationError::TypeIndicesHaveWrongLength(polygons, type_indices) => {
      assert_eq!(polygons, 1);
      assert_eq!(type_indices, 2);
    }
    _ => panic!(
      "Wrong error variant! Expected TypeIndicesHaveWrongLength but got: {:?}",
      error
    ),
  };
}

#[test]
fn error_on_concave_polygon() {
  let source_mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2]],
    polygon_type_indices: vec![0],
    height_mesh: None,
  };

  let error = source_mesh
    .clone()
    .validate()
    .expect_err("Concave polygon should be detected.");
  match error {
    ValidationError::ConcavePolygon(polygon) => assert_eq!(polygon, 0),
    _ => panic!(
      "Wrong error variant! Expected ConcavePolygon but got: {:?}",
      error
    ),
  };
}

#[test]
fn error_on_small_polygon() {
  let source_mesh = NavigationMesh::<XYZ> {
    vertices: vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0)],
    polygons: vec![vec![0, 1]],
    polygon_type_indices: vec![0],
    height_mesh: None,
  };

  let error = source_mesh
    .clone()
    .validate()
    .expect_err("Small polygon should be detected.");
  match error {
    ValidationError::NotEnoughVerticesInPolygon(polygon) => {
      assert_eq!(polygon, 0)
    }
    _ => panic!(
      "Wrong error variant! Expected NotEnoughVerticesInPolygon but got: {:?}",
      error
    ),
  };
}

#[test]
fn error_on_bad_polygon_index() {
  let source_mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 3]],
    polygon_type_indices: vec![0],
    height_mesh: None,
  };

  let error = source_mesh
    .clone()
    .validate()
    .expect_err("Bad polygon index should be detected.");
  match error {
    ValidationError::InvalidVertexIndexInPolygon(polygon) => {
      assert_eq!(polygon, 0)
    }
    _ => panic!(
      "Wrong error variant! Expected InvalidVertexIndexInPolygon but got: {:?}",
      error
    ),
  };
}

#[test]
fn error_on_degenerate_edge() {
  let source_mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 1, 2]],
    polygon_type_indices: vec![0],
    height_mesh: None,
  };

  let error = source_mesh
    .clone()
    .validate()
    .expect_err("Degenerate edge should be detected.");
  match error {
    ValidationError::DegenerateEdgeInPolygon(polygon) => {
      assert_eq!(polygon, 0)
    }
    _ => panic!(
      "Wrong error variant! Expected DegenerateEdgeInPolygon but got: {:?}",
      error
    ),
  };
}

#[test]
fn error_on_doubly_connected_edge() {
  let source_mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(2.0, 1.0, 0.0),
      Vec3::new(2.0, 0.0, 1.0),
      Vec3::new(2.0, 1.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 2], vec![1, 3, 4, 2], vec![1, 5, 6, 2]],
    polygon_type_indices: vec![0, 0, 0],
    height_mesh: None,
  };

  let error = source_mesh
    .clone()
    .validate()
    .expect_err("Doubly connected edge should be detected.");
  match error {
    ValidationError::DoublyConnectedEdge(vertex_1, vertex_2) => {
      assert_eq!((vertex_1, vertex_2), (1, 2));
    }
    _ => panic!(
      "Wrong error variant! Expected DoublyConnectedEdge but got: {:?}",
      error
    ),
  };
}

#[test]
fn derives_connectivity_and_boundary_edges() {
  let source_mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(2.0, 1.0, 0.0),
      Vec3::new(3.0, 0.0, 1.0),
      Vec3::new(3.0, 1.0, 1.0),
      Vec3::new(1.0, 2.0, 1.0),
      Vec3::new(2.0, 2.0, 1.0),
    ],
    polygons: vec![
      vec![0, 1, 2],
      vec![1, 3, 4, 2],
      vec![3, 5, 6, 4],
      vec![2, 4, 8, 7],
    ],
    polygon_type_indices: vec![0, 0, 0, 0],
    height_mesh: None,
  };

  let mut valid_mesh =
    source_mesh.clone().validate().expect("Validation succeeds.");

  // Sort boundary edges to ensure the order is consistent when comparing.
  valid_mesh.boundary_edges.sort_by_key(|boundary_edge| {
    boundary_edge.polygon_index * 100 + boundary_edge.edge_index
  });

  let expected_connectivity: [&[_]; 4] = [
    &[None, Some(Connectivity { polygon_index: 1, reverse_edge: 3 }), None],
    &[
      None,
      Some(Connectivity { polygon_index: 2, reverse_edge: 3 }),
      Some(Connectivity { polygon_index: 3, reverse_edge: 0 }),
      Some(Connectivity { polygon_index: 0, reverse_edge: 1 }),
    ],
    &[
      None,
      None,
      None,
      Some(Connectivity { polygon_index: 1, reverse_edge: 1 }),
    ],
    &[
      Some(Connectivity { polygon_index: 1, reverse_edge: 2 }),
      None,
      None,
      None,
    ],
  ];
  assert_eq!(
    valid_mesh
      .polygons
      .iter()
      .map(|polygon| polygon.connectivity.clone())
      .collect::<Vec<_>>(),
    expected_connectivity
  );
  assert_eq!(
    valid_mesh.boundary_edges,
    [
      MeshEdgeRef { polygon_index: 0, edge_index: 0 },
      MeshEdgeRef { polygon_index: 0, edge_index: 2 },
      MeshEdgeRef { polygon_index: 1, edge_index: 0 },
      MeshEdgeRef { polygon_index: 2, edge_index: 0 },
      MeshEdgeRef { polygon_index: 2, edge_index: 1 },
      MeshEdgeRef { polygon_index: 2, edge_index: 2 },
      MeshEdgeRef { polygon_index: 3, edge_index: 1 },
      MeshEdgeRef { polygon_index: 3, edge_index: 2 },
      MeshEdgeRef { polygon_index: 3, edge_index: 3 },
    ]
  );
}

#[test]
fn finds_regions() {
  let mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 2.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
      Vec3::new(1.0, 3.0, 0.0),
      Vec3::new(0.0, 3.0, 0.0),
      //
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(3.0, 0.0, 0.0),
      Vec3::new(3.0, 1.0, 0.0),
      Vec3::new(2.0, 1.0, 0.0),
      Vec3::new(3.0, 2.0, 0.0),
      Vec3::new(2.0, 2.0, 0.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3],
      vec![3, 2, 4, 5],
      vec![5, 4, 6, 7],
      vec![8, 9, 10, 11],
      vec![11, 10, 12, 13],
    ],
    polygon_type_indices: vec![0, 0, 0, 0, 0],
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  assert_eq!(
    mesh.polygons.iter().map(|polygon| polygon.region).collect::<Vec<_>>(),
    [0, 0, 0, 1, 1],
  );
}

#[test]
fn sample_point_returns_none_for_far_point() {
  let mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 1.0, 0.0),
      Vec3::new(4.0, 2.0, 0.0),
      Vec3::new(2.0, 3.0, 0.0),
      Vec3::new(1.0, 3.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 5.0, 0.0),
      Vec3::new(2.0, 5.0, 0.0),
      Vec3::new(2.0, 4.0, 0.0),
      Vec3::new(3.0, 5.0, 1.0),
      Vec3::new(3.0, 4.0, 1.0),
      Vec3::new(3.0, 4.0, -2.0),
      Vec3::new(3.0, 3.0, -2.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
    polygon_type_indices: vec![0, 0, 0, 0],
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(-3.0, 0.0, 0.0),
      &PointSampleDistance3d {
        horizontal_distance: 0.1,
        distance_below: 0.1,
        distance_above: 0.1,
        vertical_preference_ratio: 1.0,
      },
    ),
    None
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(6.0, 0.0, 0.0),
      &PointSampleDistance3d {
        horizontal_distance: 0.1,
        distance_below: 0.1,
        distance_above: 0.1,
        vertical_preference_ratio: 1.0,
      },
    ),
    None
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(0.0, 0.0, -3.0),
      &PointSampleDistance3d {
        horizontal_distance: 0.1,
        distance_below: 0.1,
        distance_above: 0.1,
        vertical_preference_ratio: 1.0,
      },
    ),
    None
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(0.0, 0.0, 2.0),
      &PointSampleDistance3d {
        horizontal_distance: 0.1,
        distance_below: 0.1,
        distance_above: 0.1,
        vertical_preference_ratio: 1.0,
      },
    ),
    None
  );
}

#[test]
fn sample_point_in_nodes() {
  let mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 1.0, 0.0),
      Vec3::new(4.0, 2.0, 0.0),
      Vec3::new(2.0, 3.0, 0.0),
      Vec3::new(1.0, 3.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 5.0, 0.0),
      Vec3::new(2.0, 5.0, 0.0),
      Vec3::new(2.0, 4.0, 0.0),
      Vec3::new(3.0, 5.0, 1.0),
      Vec3::new(3.0, 4.0, 1.0),
      Vec3::new(3.0, 4.0, -2.0),
      Vec3::new(3.0, 3.0, -2.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
    polygon_type_indices: vec![0, 0, 0, 0],
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  // Flat nodes

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(1.5, 1.5, 0.95),
      &PointSampleDistance3d {
        horizontal_distance: 1.0,
        distance_below: 1.0,
        distance_above: 1.0,
        vertical_preference_ratio: 1.0,
      },
    ),
    Some((Vec3::new(1.5, 1.5, 0.0), 0))
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(1.5, 4.0, -0.95),
      &PointSampleDistance3d {
        horizontal_distance: 1.0,
        distance_below: 1.0,
        distance_above: 1.0,
        vertical_preference_ratio: 1.0,
      },
    ),
    Some((Vec3::new(1.5, 4.0, 0.0), 1))
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(1.5, 4.0, -0.95),
      &PointSampleDistance3d {
        horizontal_distance: 1.0,
        distance_below: 1.0,
        distance_above: 1.0,
        vertical_preference_ratio: 1.0,
      },
    ),
    Some((Vec3::new(1.5, 4.0, 0.0), 1))
  );

  // Angled nodes

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(2.5, 3.5, -0.55),
      &PointSampleDistance3d {
        horizontal_distance: 5.0,
        distance_below: 5.0,
        distance_above: 5.0,
        vertical_preference_ratio: 1.0,
      },
    ),
    Some((Vec3::new(2.5, 3.5, -1.0), 3))
  );
  assert_eq!(
    mesh
      .sample_point(
        /* point= */ Vec3::new(2.5, 4.5, 0.1),
        &PointSampleDistance3d {
          horizontal_distance: 5.0,
          distance_below: 5.0,
          distance_above: 5.0,
          vertical_preference_ratio: 1.0,
        },
      )
      .map(|(point, node)| ((point * 1000.0).round() / 1000.0, node)),
    Some((Vec3::new(2.5, 4.5, 0.5), 2))
  );
}

#[test]
fn sample_point_near_node() {
  let mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 1.0, 0.0),
      Vec3::new(4.0, 2.0, 0.0),
      Vec3::new(2.0, 3.0, 0.0),
      Vec3::new(1.0, 3.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 5.0, 0.0),
      Vec3::new(2.0, 5.0, 0.0),
      Vec3::new(2.0, 4.0, 0.0),
      Vec3::new(3.0, 5.0, 1.0),
      Vec3::new(3.0, 4.0, 1.0),
      Vec3::new(3.0, 4.0, -2.0),
      Vec3::new(3.0, 3.0, -2.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
    polygon_type_indices: vec![0, 0, 0, 0],
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  // Flat nodes

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(-0.5, 1.5, 0.25),
      &PointSampleDistance3d {
        horizontal_distance: 1.0,
        distance_below: 1.0,
        distance_above: 1.0,
        vertical_preference_ratio: 1.0,
      },
    ),
    Some((Vec3::new(0.0, 1.5, 0.0), 0))
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(0.5, 5.5, -0.25),
      &PointSampleDistance3d {
        horizontal_distance: 1.0,
        distance_below: 1.0,
        distance_above: 1.0,
        vertical_preference_ratio: 1.0,
      },
    ),
    Some((Vec3::new(1.0, 5.0, 0.0), 1))
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(4.5, 1.5, -0.25),
      &PointSampleDistance3d {
        horizontal_distance: 1.0,
        distance_below: 1.0,
        distance_above: 1.0,
        vertical_preference_ratio: 1.0,
      },
    ),
    Some((Vec3::new(4.0, 1.5, 0.0), 0))
  );

  // Angled nodes

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(2.5, 5.5, 0.5),
      &PointSampleDistance3d {
        horizontal_distance: 5.0,
        distance_below: 5.0,
        distance_above: 5.0,
        vertical_preference_ratio: 1.0,
      },
    ),
    Some((Vec3::new(2.5, 5.0, 0.5), 2))
  );
}

#[test]
fn valid_polygon_gets_edge_indices() {
  let polygon = ValidPolygon {
    bounds: BoundingBox::Empty,
    vertices: vec![1, 3, 9, 2, 7],
    region: 0,
    type_index: 0,
    connectivity: vec![],
    center: Vec3::ZERO,
  };

  assert_eq!(polygon.get_edge_indices(0), (1, 3));
  assert_eq!(polygon.get_edge_indices(2), (9, 2));
  assert_eq!(polygon.get_edge_indices(4), (7, 1));
}

#[test]
fn sample_ignores_closer_horizontal() {
  let mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 0.0, -1.9),
      Vec3::new(3.0, 0.0, -1.9),
      Vec3::new(3.0, 1.0, -1.9),
      Vec3::new(1.0, 1.0, -1.9),
    ],
    polygons: vec![vec![0, 1, 2, 3], vec![4, 5, 6, 7]],
    polygon_type_indices: vec![0; 2],
    height_mesh: None,
  }
  .validate()
  .expect("mesh is valid");

  // The first polygon is physically closer, but it is not within the horizontal
  // distance, so it should be filtered out. The second polygon is much further
  // physically, but it is still within the distance_below.
  assert_eq!(
    mesh.sample_point(
      Vec3::new(1.5, 0.5, 0.0),
      &PointSampleDistance3d {
        horizontal_distance: 0.25,
        distance_above: 5.0,
        distance_below: 5.0,
        vertical_preference_ratio: 1.0,
      }
    ),
    Some((Vec3::new(1.5, 0.5, -1.9), 1))
  );
}

#[test]
fn sample_favoring_vertical() {
  let mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 0.0, -1.9),
      Vec3::new(3.0, 0.0, -1.9),
      Vec3::new(3.0, 1.0, -1.9),
      Vec3::new(1.0, 1.0, -1.9),
    ],
    polygons: vec![vec![0, 1, 2, 3], vec![4, 5, 6, 7]],
    polygon_type_indices: vec![0; 2],
    height_mesh: None,
  }
  .validate()
  .expect("mesh is valid");

  // Both polygons are in range, but the one below our query point is preferred,
  // since our vertical preference is 2.0.
  assert_eq!(
    mesh.sample_point(
      Vec3::new(2.0, 0.5, 0.0),
      &PointSampleDistance3d {
        horizontal_distance: 5.0,
        distance_above: 5.0,
        distance_below: 5.0,
        vertical_preference_ratio: 2.0,
      }
    ),
    Some((Vec3::new(2.0, 0.5, -1.9), 1))
  );
}

#[test]
fn sample_filters_vertical_points_differently() {
  let mesh = NavigationMesh::<XYZ> {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0; 1],
    height_mesh: None,
  }
  .validate()
  .expect("mesh is valid.");

  let point_sample_distance = PointSampleDistance3d {
    horizontal_distance: 100.0,
    distance_above: 1.0,
    distance_below: 2.0,
    vertical_preference_ratio: 1.0,
  };
  assert_eq!(
    mesh.sample_point(Vec3::new(0.5, 0.5, -0.5), &point_sample_distance),
    Some((Vec3::new(0.5, 0.5, 0.0), 0))
  );
  assert_eq!(
    mesh.sample_point(Vec3::new(0.5, 0.5, -1.5), &point_sample_distance),
    None
  );
  assert_eq!(
    mesh.sample_point(Vec3::new(0.5, 0.5, 0.5), &point_sample_distance),
    Some((Vec3::new(0.5, 0.5, 0.0), 0))
  );
  assert_eq!(
    mesh.sample_point(Vec3::new(0.5, 0.5, 1.5), &point_sample_distance),
    Some((Vec3::new(0.5, 0.5, 0.0), 0))
  );
  assert_eq!(
    mesh.sample_point(Vec3::new(0.5, 0.5, 2.5), &point_sample_distance),
    None
  );
}

/// Create a height mesh from some common pieces.
///
/// `vertices` is the pool of vertices that the polygons use. `polygons` are a
/// collection of height polygons, which are themselves represented by polygons
/// that are then triangulated. This allows us to match the number of regular
/// polygons (using the outer most Vec), then create as many polygons as we want
/// for that single regular polygon (the middle Vec).
fn create_height_mesh(
  vertices: Vec<Vec3>,
  polygons: Vec<Vec<Vec<usize>>>,
) -> HeightNavigationMesh<XYZ> {
  let mut height_mesh = HeightNavigationMesh {
    vertices: vec![],
    polygons: vec![],
    triangles: vec![],
  };
  for polygon in polygons.iter() {
    let base_vertex_index = height_mesh.vertices.len();
    let base_triangle_index = height_mesh.triangles.len();

    for subpolygon in polygon {
      let base_index = height_mesh.vertices.len() - base_vertex_index;
      height_mesh.vertices.push(vertices[subpolygon[0]]);
      height_mesh.vertices.push(vertices[subpolygon[1]]);

      for i in 2..subpolygon.len() {
        height_mesh.vertices.push(vertices[subpolygon[i]]);

        height_mesh.triangles.push([
          base_index as _,
          (base_index + i - 1) as _,
          (base_index + i) as _,
        ]);
      }
    }

    height_mesh.polygons.push(HeightPolygon {
      base_vertex_index: base_vertex_index as u32,
      vertex_count: (height_mesh.vertices.len() - base_vertex_index) as u32,
      base_triangle_index: base_triangle_index as u32,
      triangle_count: (height_mesh.triangles.len() - base_triangle_index)
        as u32,
    });
  }

  height_mesh
}

#[test]
fn sample_point_uses_height_mesh_if_available() {
  let mesh = NavigationMesh::<XYZ> {
    // The regular mesh considers the entire mesh as one big polygon. This
    // unfortunately means we don't represent the height very well.
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 3.0, 1.0),
      Vec3::new(0.0, 3.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0; 1],
    height_mesh: Some(create_height_mesh(
      // The height mesh tells us that the actual surface deviates heavily from
      // the regular mesh. Namely, it looks like:
      //
      //   _
      // _/
      vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(0.0, 2.0, 1.0),
        Vec3::new(1.0, 2.0, 1.0),
        Vec3::new(0.0, 3.0, 1.0),
        Vec3::new(1.0, 3.0, 1.0),
      ],
      // Polygons
      vec![vec![vec![0, 1, 3, 2], vec![2, 3, 5, 4], vec![4, 5, 7, 6]]],
    )),
  }
  .validate()
  .expect("mesh is valid.");

  let point_sample_distance = PointSampleDistance3d {
    horizontal_distance: 100.0,
    distance_above: 0.1,
    distance_below: 0.1,
    vertical_preference_ratio: 1.0,
  };

  // These points are on the height mesh, but not on the nav mesh (according to
  // the distance thresholds above). So if we weren't using the detail mesh,
  // these would not find a point.
  assert_eq!(
    mesh.sample_point(Vec3::new(0.5, 1.0, 0.0), &point_sample_distance),
    Some((Vec3::new(0.5, 1.0, 0.0), 0))
  );
  assert_eq!(
    mesh.sample_point(Vec3::new(0.5, 2.0, 1.0), &point_sample_distance),
    Some((Vec3::new(0.5, 2.0, 1.0), 0))
  );
}

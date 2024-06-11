use glam::Vec3;

use crate::{
  nav_mesh::{Connectivity, MeshEdgeRef, ValidPolygon},
  util::BoundingBox,
};

use super::{NavigationMesh, ValidationError};

#[test]
fn validation_computes_bounds_if_none() {
  let mut source_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(2.0, 0.0, 1.0),
      Vec3::new(0.5, 0.5, 3.0),
      Vec3::new(0.75, -0.25, 4.0),
      Vec3::new(0.25, 0.0, 4.0),
    ],
    polygons: vec![vec![0, 1, 2], vec![3, 4, 5]],
  };

  let valid_mesh =
    source_mesh.clone().validate().expect("Validation succeeds.");
  assert_eq!(
    valid_mesh.mesh_bounds,
    BoundingBox::new_box(Vec3::new(0.0, -0.25, 0.0), Vec3::new(2.0, 1.0, 4.0))
  );

  let fake_mesh_bounds =
    BoundingBox::new_box(Vec3::new(-5.0, -5.0, -5.0), Vec3::new(5.0, 5.0, 5.0));
  source_mesh.mesh_bounds = Some(fake_mesh_bounds);

  let valid_mesh =
    source_mesh.clone().validate().expect("Validation succeeds.");
  assert_eq!(valid_mesh.mesh_bounds, fake_mesh_bounds);
}

#[test]
fn correctly_computes_bounds_for_small_number_of_points() {
  let valid_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(-1.0, -1.0, -1.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(2.0, 0.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 2]],
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
  let source_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(2.0, 0.0, 1.0),
      Vec3::new(0.5, 0.5, 3.0),
      Vec3::new(0.75, -0.25, 4.0),
      Vec3::new(0.25, 0.0, 4.0),
    ],
    polygons: vec![vec![0, 1, 2], vec![3, 4, 5]],
  };

  let expected_polygons = vec![
    ValidPolygon {
      vertices: source_mesh.polygons[0].clone(),
      bounds: BoundingBox::new_box(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0, 1.0, 1.0),
      ),
      center: Vec3::new(3.0, 1.0, 1.0) / 3.0,
    },
    ValidPolygon {
      vertices: source_mesh.polygons[1].clone(),
      bounds: BoundingBox::new_box(
        Vec3::new(0.25, -0.25, 3.0),
        Vec3::new(0.75, 0.5, 4.0),
      ),
      center: Vec3::new(1.5, 0.25, 11.0) / 3.0,
    },
  ];

  let valid_mesh =
    source_mesh.clone().validate().expect("Validation succeeds.");
  assert_eq!(valid_mesh.vertices, source_mesh.vertices);
  assert_eq!(valid_mesh.polygons, expected_polygons);
}

#[test]
fn error_on_concave_polygon() {
  let source_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(1.0, 0.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2]],
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
  let source_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 1.0)],
    polygons: vec![vec![0, 1]],
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
  let source_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 3]],
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
  let source_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 1, 2]],
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
  let source_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 1.0),
      Vec3::new(2.0, 1.0, 0.0),
      Vec3::new(2.0, 1.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 2], vec![1, 3, 4, 2], vec![1, 5, 6, 2]],
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
  let source_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 1.0),
      Vec3::new(3.0, 1.0, 0.0),
      Vec3::new(3.0, 1.0, 1.0),
      Vec3::new(1.0, 1.0, 2.0),
      Vec3::new(2.0, 1.0, 2.0),
    ],
    polygons: vec![
      vec![0, 1, 2],
      vec![1, 3, 4, 2],
      vec![3, 5, 6, 4],
      vec![2, 4, 8, 7],
    ],
  };

  let mut valid_mesh =
    source_mesh.clone().validate().expect("Validation succeeds.");

  // Sort connectivity and boundary edges to ensure the order is consistent
  // when comparing.
  for connectivity in valid_mesh.connectivity.iter_mut() {
    connectivity.sort_by_key(|connectivity| {
      connectivity.polygon_index * 100 + connectivity.edge_index
    });
  }
  valid_mesh.boundary_edges.sort_by_key(|boundary_edge| {
    boundary_edge.polygon_index * 100 + boundary_edge.edge_index
  });

  let expected_connectivity: [&[_]; 4] = [
    &[Connectivity { edge_index: 1, polygon_index: 1 }],
    &[
      Connectivity { edge_index: 3, polygon_index: 0 },
      Connectivity { edge_index: 1, polygon_index: 2 },
      Connectivity { edge_index: 2, polygon_index: 3 },
    ],
    &[Connectivity { edge_index: 3, polygon_index: 1 }],
    &[Connectivity { edge_index: 0, polygon_index: 1 }],
  ];
  assert_eq!(valid_mesh.connectivity, expected_connectivity);
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
fn sample_point_returns_none_for_far_point() {
  let mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 0.0, 1.0),
      Vec3::new(4.0, 0.0, 2.0),
      Vec3::new(2.0, 0.0, 3.0),
      Vec3::new(1.0, 0.0, 3.0),
      Vec3::new(0.0, 0.0, 2.0),
      Vec3::new(0.0, 0.0, 1.0),
      Vec3::new(1.0, 0.0, 5.0),
      Vec3::new(2.0, 0.0, 5.0),
      Vec3::new(2.0, 0.0, 4.0),
      Vec3::new(3.0, 1.0, 5.0),
      Vec3::new(3.0, 1.0, 4.0),
      Vec3::new(3.0, -2.0, 4.0),
      Vec3::new(3.0, -2.0, 3.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
  }
  .validate()
  .expect("Mesh is valid.");

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(-3.0, 0.0, 0.0),
      /* distance_to_node= */ 0.1
    ),
    None
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(6.0, 0.0, 0.0),
      /* distance_to_node= */ 0.1
    ),
    None
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(0.0, -3.0, 0.0),
      /* distance_to_node= */ 0.1
    ),
    None
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(0.0, 2.0, 0.0),
      /* distance_to_node= */ 0.1
    ),
    None
  );
}

#[test]
fn sample_point_in_nodes() {
  let mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 0.0, 1.0),
      Vec3::new(4.0, 0.0, 2.0),
      Vec3::new(2.0, 0.0, 3.0),
      Vec3::new(1.0, 0.0, 3.0),
      Vec3::new(0.0, 0.0, 2.0),
      Vec3::new(0.0, 0.0, 1.0),
      Vec3::new(1.0, 0.0, 5.0),
      Vec3::new(2.0, 0.0, 5.0),
      Vec3::new(2.0, 0.0, 4.0),
      Vec3::new(3.0, 1.0, 5.0),
      Vec3::new(3.0, 1.0, 4.0),
      Vec3::new(3.0, -2.0, 4.0),
      Vec3::new(3.0, -2.0, 3.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
  }
  .validate()
  .expect("Mesh is valid.");

  // Flat nodes

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(1.5, 0.95, 1.5),
      /* distance_to_node= */ 1.0,
    ),
    Some((Vec3::new(1.5, 0.0, 1.5), 0))
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(1.5, -0.95, 4.0),
      /* distance_to_node= */ 1.0,
    ),
    Some((Vec3::new(1.5, 0.0, 4.0), 1))
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(1.5, -0.95, 4.0),
      /* distance_to_node= */ 1.0,
    ),
    Some((Vec3::new(1.5, 0.0, 4.0), 1))
  );

  // Angled nodes

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(2.5, -0.55, 3.5),
      /* distance_to_node = */ 5.0
    ),
    Some((Vec3::new(2.5, -1.0, 3.5), 3))
  );
  assert_eq!(
    mesh
      .sample_point(
        /* point= */ Vec3::new(2.5, 0.1, 4.5),
        /* distance_to_node = */ 5.0
      )
      .map(|(point, node)| ((point * 1000.0).round() / 1000.0, node)),
    Some((Vec3::new(2.5, 0.5, 4.5), 2))
  );
}

#[test]
fn sample_point_near_node() {
  let mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 0.0, 1.0),
      Vec3::new(4.0, 0.0, 2.0),
      Vec3::new(2.0, 0.0, 3.0),
      Vec3::new(1.0, 0.0, 3.0),
      Vec3::new(0.0, 0.0, 2.0),
      Vec3::new(0.0, 0.0, 1.0),
      Vec3::new(1.0, 0.0, 5.0),
      Vec3::new(2.0, 0.0, 5.0),
      Vec3::new(2.0, 0.0, 4.0),
      Vec3::new(3.0, 1.0, 5.0),
      Vec3::new(3.0, 1.0, 4.0),
      Vec3::new(3.0, -2.0, 4.0),
      Vec3::new(3.0, -2.0, 3.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
  }
  .validate()
  .expect("Mesh is valid.");

  // Flat nodes

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(-0.5, 0.25, 1.5),
      /* distance_to_node= */ 1.0,
    ),
    Some((Vec3::new(0.0, 0.0, 1.5), 0))
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(0.5, -0.25, 5.5),
      /* distance_to_node= */ 1.0,
    ),
    Some((Vec3::new(1.0, 0.0, 5.0), 1))
  );

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(4.5, -0.25, 1.5),
      /* distance_to_node= */ 1.0,
    ),
    Some((Vec3::new(4.0, 0.0, 1.5), 0))
  );

  // Angled nodes

  assert_eq!(
    mesh.sample_point(
      /* point= */ Vec3::new(2.5, 0.5, 5.5),
      /* distance_to_node = */ 5.0
    ),
    Some((Vec3::new(2.5, 0.5, 5.0), 2))
  );
}

#[test]
fn valid_polygon_gets_edge_indices() {
  let polygon = ValidPolygon {
    bounds: BoundingBox::Empty,
    vertices: vec![1, 3, 9, 2, 7],
    center: Vec3::ZERO,
  };

  assert_eq!(polygon.get_edge_indices(0), (1, 3));
  assert_eq!(polygon.get_edge_indices(2), (9, 2));
  assert_eq!(polygon.get_edge_indices(4), (7, 1));
}

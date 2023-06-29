use std::collections::HashMap;

use glam::{swizzles::Vec3Swizzles, Vec3};

use crate::BoundingBox;

// A navigation mesh.
#[derive(Clone)]
pub struct NavigationMesh {
  // The bounds of the mesh data itself. This should be a tight bounding box
  // around the vertices of the navigation mesh. This may be None to
  // automatically compute this from the vertices.
  pub mesh_bounds: Option<BoundingBox>,
  // The vertices that make up the polygons. The Y component is considered up.
  pub vertices: Vec<Vec3>,
  // The polygons of the mesh. Polygons are indices to the `vertices` that make
  // up the polygon. Polygons must be convex, and oriented counterclockwise.
  // Polygons are assumed to be not self-intersecting.
  pub polygons: Vec<Vec<usize>>,
}

// An error when validating a navigation mesh.
#[derive(Debug)]
pub enum ValidationError {
  // A polygon is concave (or has edges in clockwise order). Stores the index
  // of the polygon.
  ConcavePolygon(usize),
  // A polygon was not big enough (less than 3 vertices). Stores the index of
  // the polygon.
  NotEnoughVerticesInPolygon(usize),
  // A polygon indexed an invalid vertex. Stores the index of the polygon.
  InvalidVertexIndexInPolygon(usize),
  // A polygon contains a degenerate edge (an edge using the same vertex for
  // both endpoints). Stores the index of the polygon.
  DegenerateEdgeInPolygon(usize),
  // An edge is used by more than two polygons. Stores the indices of the two
  // vertices that make up the edge.
  DoublyConnectedEdge(usize, usize),
}

impl NavigationMesh {
  pub fn validate(mut self) -> Result<ValidNavigationMesh, ValidationError> {
    if self.mesh_bounds.is_none() {
      if self.vertices.is_empty() {
        self.mesh_bounds = Some(BoundingBox::Empty);
      }
      self.mesh_bounds =
        Some(self.vertices.iter().fold(BoundingBox::Empty, |acc, &vertex| {
          acc.expand_to_point(vertex)
        }));
    }

    enum ConnectivityState {
      Disconnected,
      Boundary {
        polygon: usize,
        edge: usize,
      },
      Connected {
        polygon_1: usize,
        edge_1: usize,
        polygon_2: usize,
        edge_2: usize,
      },
    }
    let mut connectivity_set = HashMap::new();

    for (polygon_index, polygon) in self.polygons.iter().enumerate() {
      if polygon.len() < 3 {
        return Err(ValidationError::NotEnoughVerticesInPolygon(polygon_index));
      }

      for vertex_index in polygon {
        if *vertex_index >= self.vertices.len() {
          return Err(ValidationError::InvalidVertexIndexInPolygon(
            polygon_index,
          ));
        }
      }

      for i in 0..polygon.len() {
        let left_vertex =
          polygon[if i == 0 { polygon.len() - 1 } else { i - 1 }];
        let center_vertex = polygon[i];
        let right_vertex =
          polygon[if i == polygon.len() - 1 { 0 } else { i + 1 }];

        // Check if the edge is degenerate.

        let edge = if center_vertex < right_vertex {
          (center_vertex, right_vertex)
        } else {
          (right_vertex, center_vertex)
        };
        if edge.0 == edge.1 {
          return Err(ValidationError::DegenerateEdgeInPolygon(polygon_index));
        }

        // Derive connectivity for the edge.

        let state = connectivity_set
          .entry(edge)
          .or_insert(ConnectivityState::Disconnected);
        match state {
          ConnectivityState::Disconnected => {
            *state =
              ConnectivityState::Boundary { polygon: polygon_index, edge: i };
          }
          &mut ConnectivityState::Boundary {
            polygon: polygon_1,
            edge: edge_1,
            ..
          } => {
            *state = ConnectivityState::Connected {
              polygon_1,
              edge_1,
              polygon_2: polygon_index,
              edge_2: i,
            };
          }
          ConnectivityState::Connected { .. } => {
            return Err(ValidationError::DoublyConnectedEdge(edge.0, edge.1));
          }
        }

        // Check if the vertex is concave.

        let left_vertex = self.vertices[left_vertex].xz();
        let center_vertex = self.vertices[center_vertex].xz();
        let right_vertex = self.vertices[right_vertex].xz();

        let left_edge = left_vertex - center_vertex;
        let right_edge = right_vertex - center_vertex;

        // If right_edge is to the left of the left_edge, then the polygon is
        // concave. This is the equation for a 2D cross product.
        if right_edge.x * left_edge.y - right_edge.y * left_edge.x < 0.0 {
          return Err(ValidationError::ConcavePolygon(polygon_index));
        }
      }
    }

    let mut connectivity = vec![Vec::new(); self.polygons.len()];
    let mut boundary_edges = Vec::new();
    for connectivity_state in connectivity_set.values() {
      match connectivity_state {
        ConnectivityState::Disconnected => panic!("Value is never stored"),
        &ConnectivityState::Boundary { polygon, edge } => {
          boundary_edges
            .push(MeshEdgeRef { edge_index: edge, polygon_index: polygon });
        }
        &ConnectivityState::Connected {
          polygon_1,
          edge_1,
          polygon_2,
          edge_2,
        } => {
          connectivity[polygon_1].push(Connectivity {
            edge_index: edge_1,
            polygon_index: polygon_2,
          });
          connectivity[polygon_2].push(Connectivity {
            edge_index: edge_2,
            polygon_index: polygon_1,
          });
        }
      }
    }

    Ok(ValidNavigationMesh {
      mesh_bounds: self.mesh_bounds.unwrap(),
      polygons: self
        .polygons
        .drain(..)
        .map(|polygon_vertices| ValidPolygon {
          bounds: polygon_vertices
            .iter()
            .fold(BoundingBox::Empty, |bounds, vertex| {
              bounds.expand_to_point(self.vertices[*vertex])
            }),
          center: polygon_vertices
            .iter()
            .map(|i| self.vertices[*i])
            .sum::<Vec3>()
            / polygon_vertices.len() as f32,
          vertices: polygon_vertices,
        })
        .collect(),
      vertices: self.vertices,
      connectivity,
      boundary_edges,
    })
  }
}

// A navigation mesh which has been validated and derived data has been
// computed.
#[derive(Debug, Clone)]
pub struct ValidNavigationMesh {
  // The bounds of the mesh data itself. This is a tight bounding box around
  // the vertices of the navigation mesh.
  pub(crate) mesh_bounds: BoundingBox,
  // The vertices that make up the polygons.
  pub(crate) vertices: Vec<Vec3>,
  // The polygons of the mesh.
  pub(crate) polygons: Vec<ValidPolygon>,
  // The connectivity for each polygon. Each polygon has a Vec of the pairs
  // of (edge index, node index).
  pub(crate) connectivity: Vec<Vec<Connectivity>>,
  // The boundary edges in the navigation mesh. Edges are stored as pairs of
  // vertices in a counter-clockwise direction. That is, moving along an edge
  // (e.0, e.1) from e.0 to e.1 will move counter-clockwise along the boundary.
  // The order of edges is undefined.
  pub(crate) boundary_edges: Vec<MeshEdgeRef>,
}

// A valid polygon. This means the polygon is convex and indexes the `vertices`
// Vec of the corresponding ValidNavigationMesh.
#[derive(PartialEq, Debug, Clone)]
pub(crate) struct ValidPolygon {
  // The vertices are indexes to the `vertices` Vec of the corresponding
  // ValidNavigationMesh.
  pub(crate) vertices: Vec<usize>,
  // The bounding box of `vertices`.
  pub(crate) bounds: BoundingBox,
  pub(crate) center: Vec3,
}

#[derive(PartialEq, Eq, Debug, Clone)]
pub struct Connectivity {
  // The index of the edge within the polygon.
  pub edge_index: usize,
  // The index of the polygon that this edge leads to.
  pub polygon_index: usize,
}

// A reference to an edge on a navigation mesh.
#[derive(PartialEq, Eq, Debug, Clone, Hash)]
pub struct MeshEdgeRef {
  // The index of the polygon that this edge belongs to.
  pub polygon_index: usize,
  // The index of the edge within the polygon.
  pub edge_index: usize,
}

impl ValidNavigationMesh {
  pub fn get_bounds(&self) -> BoundingBox {
    self.mesh_bounds
  }

  // Gets the points that make up the specified edge.
  pub fn get_edge_points(&self, edge_ref: MeshEdgeRef) -> (Vec3, Vec3) {
    let polygon = &self.polygons[edge_ref.polygon_index];
    let left_vertex_index = polygon.vertices[edge_ref.edge_index];

    let right_vertex_index =
      if edge_ref.edge_index == polygon.vertices.len() - 1 {
        0
      } else {
        edge_ref.edge_index + 1
      };
    let right_vertex_index = polygon.vertices[right_vertex_index];

    (self.vertices[left_vertex_index], self.vertices[right_vertex_index])
  }

  pub fn find_linkable_edges(
    &self,
    region_bounds: BoundingBox,
    linkable_distance_to_region_edge: f32,
  ) -> [Vec<MeshEdgeRef>; 6] {
    const ALIGNED_TO_PLANE_DIRECTION_DEVIATION_EPSILON: f32 = 1e-5;

    fn find_plane_index(
      region_bounds: &BoundingBox,
      edge_vertices: &(Vec3, Vec3),
      linkable_distance_to_region_edge: f32,
    ) -> Option<usize> {
      let region_bounds = region_bounds.as_box();
      let edge_direction_abs =
        (edge_vertices.1 - edge_vertices.0).normalize_or_zero().abs();
      assert_ne!(edge_direction_abs, Vec3::ZERO);

      if edge_direction_abs.x < ALIGNED_TO_PLANE_DIRECTION_DEVIATION_EPSILON {
        if edge_vertices.0.x - region_bounds.0.x
          < linkable_distance_to_region_edge
        {
          return Some(0);
        } else if region_bounds.1.x - edge_vertices.0.x
          < linkable_distance_to_region_edge
        {
          return Some(1);
        }
      }

      if edge_direction_abs.y < ALIGNED_TO_PLANE_DIRECTION_DEVIATION_EPSILON {
        if edge_vertices.0.y - region_bounds.0.y
          < linkable_distance_to_region_edge
        {
          return Some(2);
        } else if region_bounds.1.y - edge_vertices.0.y
          < linkable_distance_to_region_edge
        {
          return Some(3);
        }
      }

      if edge_direction_abs.z < ALIGNED_TO_PLANE_DIRECTION_DEVIATION_EPSILON {
        if edge_vertices.0.z - region_bounds.0.z
          < linkable_distance_to_region_edge
        {
          return Some(4);
        } else if region_bounds.1.z - edge_vertices.0.z
          < linkable_distance_to_region_edge
        {
          return Some(5);
        }
      }

      None
    }

    let mut linkable_edges_by_plane =
      [Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    for boundary_edge in self.boundary_edges.iter() {
      let edge_vertices = self.get_edge_points(boundary_edge.clone());

      if let Some(index) = find_plane_index(
        &region_bounds,
        &edge_vertices,
        linkable_distance_to_region_edge,
      ) {
        linkable_edges_by_plane[index].push(boundary_edge.clone());
      }
    }

    linkable_edges_by_plane
  }

  pub(crate) fn sample_point(
    &self,
    point: Vec3,
    distance_to_node: f32,
  ) -> Option<(Vec3, usize)> {
    let sample_box = BoundingBox::new_box(point, point)
      .expand_by_size(Vec3::ONE * distance_to_node);

    fn project_to_triangle(triangle: (Vec3, Vec3, Vec3), point: Vec3) -> Vec3 {
      let triangle_deltas = (
        triangle.1 - triangle.0,
        triangle.2 - triangle.1,
        triangle.0 - triangle.2,
      );
      let triangle_deltas_flat = (
        triangle_deltas.0.xz(),
        triangle_deltas.1.xz(),
        triangle_deltas.2.xz(),
      );

      if triangle_deltas_flat.0.perp_dot(point.xz() - triangle.0.xz()) < 0.0 {
        let s = triangle_deltas_flat.0.dot(point.xz() - triangle.0.xz())
          / triangle_deltas_flat.0.length_squared();
        return triangle_deltas.0 * s.clamp(0.0, 1.0) + triangle.0;
      }
      if triangle_deltas_flat.1.perp_dot(point.xz() - triangle.1.xz()) < 0.0 {
        let s = triangle_deltas_flat.1.dot(point.xz() - triangle.1.xz())
          / triangle_deltas_flat.1.length_squared();
        return triangle_deltas.1 * s.clamp(0.0, 1.0) + triangle.1;
      }
      if triangle_deltas_flat.2.perp_dot(point.xz() - triangle.2.xz()) < 0.0 {
        let s = triangle_deltas_flat.2.dot(point.xz() - triangle.2.xz())
          / triangle_deltas_flat.2.length_squared();
        return triangle_deltas.2 * s.clamp(0.0, 1.0) + triangle.2;
      }

      let normal = -triangle_deltas.0.cross(triangle_deltas.2).normalize();
      let height = normal.dot(point - triangle.0) / normal.y;
      Vec3::new(point.x, point.y - height, point.z)
    }

    let mut best_node = None;

    for (polygon_index, polygon) in self.polygons.iter().enumerate() {
      if !sample_box.intersects_bounds(&polygon.bounds) {
        continue;
      }
      for i in 2..polygon.vertices.len() {
        let triangle =
          (polygon.vertices[0], polygon.vertices[i - 1], polygon.vertices[i]);
        let triangle = (
          self.vertices[triangle.0],
          self.vertices[triangle.1],
          self.vertices[triangle.2],
        );
        let projected_point = project_to_triangle(triangle, point);

        let distance_to_triangle = point.distance_squared(projected_point);
        if distance_to_triangle < distance_to_node * distance_to_node {
          let replace = match best_node {
            None => true,
            Some((_, _, previous_best_distance))
              if distance_to_triangle < previous_best_distance =>
            {
              true
            }
            _ => false,
          };
          if replace {
            best_node =
              Some((polygon_index, projected_point, distance_to_triangle));
          }
        }
      }
    }

    best_node.map(|(polygon_index, projected_point, _)| {
      (projected_point, polygon_index)
    })
  }
}

impl ValidPolygon {
  pub(crate) fn get_edge_indices(&self, edge: usize) -> (usize, usize) {
    (
      self.vertices[edge],
      self.vertices[if edge == self.vertices.len() - 1 { 0 } else { edge + 1 }],
    )
  }
}

#[cfg(test)]
mod tests {
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
      BoundingBox::new_box(
        Vec3::new(0.0, -0.25, 0.0),
        Vec3::new(2.0, 1.0, 4.0)
      )
    );

    let fake_mesh_bounds = BoundingBox::new_box(
      Vec3::new(-5.0, -5.0, -5.0),
      Vec3::new(5.0, 5.0, 5.0),
    );
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
      BoundingBox::new_box(
        Vec3::new(-1.0, -1.0, -1.0),
        Vec3::new(2.0, 1.0, 1.0)
      )
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
      ValidationError::NotEnoughVerticesInPolygon(polygon) => assert_eq!(polygon, 0),
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
      ValidationError::InvalidVertexIndexInPolygon(polygon) => assert_eq!(polygon, 0),
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
  fn finds_linkable_edges() {
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

    let mut linkable_edges = mesh.find_linkable_edges(mesh.mesh_bounds, 1e-10);

    // Sort the edges so it matches the expected result.
    linkable_edges.iter_mut().for_each(|edges| {
      edges.sort_by_key(|edge| edge.polygon_index * 100 + edge.edge_index)
    });

    assert_eq!(
      linkable_edges,
      [
        &[MeshEdgeRef { polygon_index: 0, edge_index: 6 }] as &[_],
        &[MeshEdgeRef { polygon_index: 0, edge_index: 2 }] as &[_],
        &[MeshEdgeRef { polygon_index: 3, edge_index: 2 }] as &[_],
        &[MeshEdgeRef { polygon_index: 2, edge_index: 2 }] as &[_],
        &[MeshEdgeRef { polygon_index: 0, edge_index: 0 }] as &[_],
        &[
          MeshEdgeRef { polygon_index: 1, edge_index: 3 },
          MeshEdgeRef { polygon_index: 2, edge_index: 3 },
        ] as &[_],
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
}

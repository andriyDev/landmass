use std::{
  cmp::Ordering,
  collections::{HashMap, HashSet},
  marker::PhantomData,
};

use disjoint::DisjointSet;
use glam::{swizzles::Vec3Swizzles, Vec3};
use thiserror::Error;

use crate::{coords::CoordinateSystem, util::BoundingBox};

/// A navigation mesh.
pub struct NavigationMesh<CS: CoordinateSystem> {
  /// The vertices that make up the polygons.
  pub vertices: Vec<CS::Coordinate>,
  /// The polygons of the mesh. Polygons are indices to the `vertices` that
  /// make up the polygon. Polygons must be convex, and oriented
  /// counterclockwise (using the right hand rule). Polygons are assumed to be
  /// not self-intersecting.
  pub polygons: Vec<Vec<usize>>,
  /// The type index of each polygon. This type index is translated into a real
  /// [`crate::NodeType`] when assigned to an [`crate::Archipelago`]. Must be
  /// the same length as [`Self::polygons`].
  pub polygon_type_indices: Vec<usize>,
}

impl<CS: CoordinateSystem> Clone for NavigationMesh<CS>
where
  CS::Coordinate: Clone,
{
  fn clone(&self) -> Self {
    Self {
      vertices: self.vertices.clone(),
      polygons: self.polygons.clone(),
      polygon_type_indices: self.polygon_type_indices.clone(),
    }
  }
}

/// An error when validating a navigation mesh.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Error)]
pub enum ValidationError {
  /// Stores the number of polygons and the number of type indices.
  #[error(
    "The polygon type indices do not have the same length as the polygons. There are {0} polygons, but {1} type indices."
  )]
  TypeIndicesHaveWrongLength(usize, usize),
  /// Stores the index of the polygon.
  #[error(
    "The polygon at index {0} is concave or has edges in clockwise order."
  )]
  ConcavePolygon(usize),
  /// Stores the index of the polygon.
  #[error("The polygon at index {0} does not have at least 3 vertices.")]
  NotEnoughVerticesInPolygon(usize),
  /// Stores the index of the polygon.
  #[error("The polygon at index {0} references an out-of-bounds vertex.")]
  InvalidVertexIndexInPolygon(usize),
  /// Stores the index of the polygon.
  #[error("The polygon at index {0} contains a degenerate edge (an edge with zero length).")]
  DegenerateEdgeInPolygon(usize),
  /// Stores the indices of the two vertices that make up the edge.
  #[error(
    "The edge made from vertices {0} and {1} is used by more than two polygons."
  )]
  DoublyConnectedEdge(usize, usize),
}

impl<CS: CoordinateSystem> NavigationMesh<CS> {
  /// Ensures required invariants of the navigation mesh, and computes
  /// additional derived properties to produce and optimized and validated
  /// navigation mesh. Returns an error if the navigation mesh is invalid in
  /// some way.
  pub fn validate(
    mut self,
  ) -> Result<ValidNavigationMesh<CS>, ValidationError> {
    if self.polygons.len() != self.polygon_type_indices.len() {
      return Err(ValidationError::TypeIndicesHaveWrongLength(
        self.polygons.len(),
        self.polygon_type_indices.len(),
      ));
    }

    let vertices =
      self.vertices.iter().map(CS::to_landmass).collect::<Vec<_>>();

    let mesh_bounds = vertices
      .iter()
      .fold(BoundingBox::Empty, |acc, &vertex| acc.expand_to_point(vertex));

    let mut region_sets = DisjointSet::with_len(self.polygons.len());

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
        if *vertex_index >= vertices.len() {
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
            region_sets.join(polygon_1, polygon_index);
          }
          ConnectivityState::Connected { .. } => {
            return Err(ValidationError::DoublyConnectedEdge(edge.0, edge.1));
          }
        }

        // Check if the vertex is concave.

        let left_vertex = vertices[left_vertex].xy();
        let center_vertex = vertices[center_vertex].xy();
        let right_vertex = vertices[right_vertex].xy();

        let left_edge = left_vertex - center_vertex;
        let right_edge = right_vertex - center_vertex;

        match right_edge.perp_dot(left_edge).partial_cmp(&0.0) {
          // The right edge is to the right of the left edge.
          Some(Ordering::Greater) => {}
          // The right edge is parallel to the left edge, but they point in
          // opposite directions.
          Some(Ordering::Equal) if right_edge.dot(left_edge) < 0.0 => {}
          // right_edge is to the left of the left_edge (or they are parallel
          // and point in the same direciton), so the polygon is
          // concave.
          _ => return Err(ValidationError::ConcavePolygon(polygon_index)),
        }
      }
    }

    let mut region_to_normalized_region = HashMap::new();
    let mut used_type_indices = HashSet::new();

    let mut polygons = self
      .polygons
      .drain(..)
      .enumerate()
      .map(|(polygon_index, polygon_vertices)| ValidPolygon {
        bounds: polygon_vertices
          .iter()
          .fold(BoundingBox::Empty, |bounds, vertex| {
            bounds.expand_to_point(vertices[*vertex])
          }),
        center: polygon_vertices.iter().map(|i| vertices[*i]).sum::<Vec3>()
          / polygon_vertices.len() as f32,
        connectivity: vec![None; polygon_vertices.len()],
        vertices: polygon_vertices,
        region: {
          let region = region_sets.root_of(polygon_index);
          // Get around the borrow checker by deciding on the new normalized
          // region beforehand.
          let new_normalized_region = region_to_normalized_region.len();
          // Either lookup the existing normalized region or insert the next
          // unique index.
          *region_to_normalized_region
            .entry(region)
            .or_insert_with(|| new_normalized_region)
        },
        type_index: self.polygon_type_indices[polygon_index],
      })
      .inspect(|polygon| {
        used_type_indices.insert(polygon.type_index);
      })
      .collect::<Vec<_>>();

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
          let edge = polygons[polygon_1].get_edge_indices(edge_1);
          let edge_center = (vertices[edge.0] + vertices[edge.1]) / 2.0;
          let travel_distances = (
            polygons[polygon_1].center.distance(edge_center),
            polygons[polygon_2].center.distance(edge_center),
          );
          polygons[polygon_1].connectivity[edge_1] =
            Some(Connectivity { polygon_index: polygon_2, travel_distances });
          polygons[polygon_2].connectivity[edge_2] = Some(Connectivity {
            polygon_index: polygon_1,
            travel_distances: (travel_distances.1, travel_distances.0),
          });
        }
      }
    }

    Ok(ValidNavigationMesh {
      mesh_bounds,
      polygons,
      vertices,
      boundary_edges,
      used_type_indices,
      marker: Default::default(),
    })
  }
}

/// A navigation mesh which has been validated and derived data has been
/// computed.
pub struct ValidNavigationMesh<CS: CoordinateSystem> {
  /// The bounds of the mesh data itself. This is a tight bounding box around
  /// the vertices of the navigation mesh.
  pub(crate) mesh_bounds: BoundingBox,
  /// The vertices that make up the polygons.
  pub(crate) vertices: Vec<Vec3>,
  /// The polygons of the mesh.
  pub(crate) polygons: Vec<ValidPolygon>,
  /// The boundary edges in the navigation mesh. Edges are stored as pairs of
  /// vertices in a counter-clockwise direction. That is, moving along an edge
  /// (e.0, e.1) from e.0 to e.1 will move counter-clockwise along the
  /// boundary. The order of edges is undefined.
  pub(crate) boundary_edges: Vec<MeshEdgeRef>,
  /// The type indices used by this navigation mesh. This is a convenience for
  /// just iterating through every polygon and checking its type index. Note
  /// these don't correspond to [`crate::NodeType`]s yet. This occurs once
  /// assigned to an island.
  pub(crate) used_type_indices: HashSet<usize>,
  /// Marker for the CoordinateSystem.
  pub(crate) marker: PhantomData<CS>,
}

// Manual Debug impl to avoid Debug bound on CoordinateSystem.
impl<CS: CoordinateSystem> Clone for ValidNavigationMesh<CS> {
  fn clone(&self) -> Self {
    Self {
      mesh_bounds: self.mesh_bounds,
      vertices: self.vertices.clone(),
      polygons: self.polygons.clone(),
      boundary_edges: self.boundary_edges.clone(),
      used_type_indices: self.used_type_indices.clone(),
      marker: self.marker,
    }
  }
}

// Manual Debug impl to avoid Debug bound on CoordinateSystem.
impl<CS: CoordinateSystem> std::fmt::Debug for ValidNavigationMesh<CS> {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.debug_struct("ValidNavigationMesh")
      .field("mesh_bounds", &self.mesh_bounds)
      .field("vertices", &self.vertices)
      .field("polygons", &self.polygons)
      .field("boundary_edges", &self.boundary_edges)
      .field("used_type_indices", &self.used_type_indices)
      .field("marker", &self.marker)
      .finish()
  }
}

/// A valid polygon. This means the polygon is convex and indexes the `vertices`
/// Vec of the corresponding ValidNavigationMesh.
#[derive(PartialEq, Debug, Clone)]
pub(crate) struct ValidPolygon {
  /// The vertices are indexes to the `vertices` Vec of the corresponding
  /// ValidNavigationMesh.
  pub(crate) vertices: Vec<usize>,
  /// The connectivity of each edge in the polygon. This is the same length as
  /// the number of edges (which is equivalent to `self.vertices.len()`).
  /// Entries that are `None` correspond to the boundary of the navigation
  /// mesh, while `Some` entries are connected to another node.
  pub(crate) connectivity: Vec<Option<Connectivity>>,
  /// The "region" that this polygon belongs to. Each region is disjoint from
  /// every other. A "direct" path only exists if the region matches between
  /// two nodes. An "indirect" path exists if regions are joined together
  /// through boundary links.
  pub(crate) region: usize,
  /// The "type" of this node. This is translated into a [`crate::NodeType`]
  /// once it is part of an island.
  pub(crate) type_index: usize,
  /// The bounding box of `vertices`.
  pub(crate) bounds: BoundingBox,
  /// The center of the polygon.
  pub(crate) center: Vec3,
}

#[derive(PartialEq, Debug, Clone)]
pub(crate) struct Connectivity {
  /// The index of the polygon that this edge leads to.
  pub(crate) polygon_index: usize,
  /// The distances of travelling across this connection. The first is the
  /// distance travelled across the starting node, and the second is the
  /// distance travelled across the destination node. These must be multiplied
  /// by the actual node costs.
  pub(crate) travel_distances: (f32, f32),
}

/// A reference to an edge on a navigation mesh.
#[derive(PartialEq, Eq, Debug, Clone, Hash, Default)]
pub(crate) struct MeshEdgeRef {
  /// The index of the polygon that this edge belongs to.
  pub(crate) polygon_index: usize,
  /// The index of the edge within the polygon.
  pub(crate) edge_index: usize,
}

impl<CS: CoordinateSystem> ValidNavigationMesh<CS> {
  /// Returns the bounds of the navigation mesh.
  pub(crate) fn get_bounds(&self) -> BoundingBox {
    self.mesh_bounds
  }

  // Gets the points that make up the specified edge.
  pub(crate) fn get_edge_points(&self, edge_ref: MeshEdgeRef) -> (Vec3, Vec3) {
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

  /// Finds the node nearest to (and within `distance_to_node` of) `point`.
  /// Returns the point on the nav mesh nearest to `point` and the index of the
  /// polygon.
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
        triangle_deltas.0.xy(),
        triangle_deltas.1.xy(),
        triangle_deltas.2.xy(),
      );

      if triangle_deltas_flat.0.perp_dot(point.xy() - triangle.0.xy()) < 0.0 {
        let s = triangle_deltas_flat.0.dot(point.xy() - triangle.0.xy())
          / triangle_deltas_flat.0.length_squared();
        return triangle_deltas.0 * s.clamp(0.0, 1.0) + triangle.0;
      }
      if triangle_deltas_flat.1.perp_dot(point.xy() - triangle.1.xy()) < 0.0 {
        let s = triangle_deltas_flat.1.dot(point.xy() - triangle.1.xy())
          / triangle_deltas_flat.1.length_squared();
        return triangle_deltas.1 * s.clamp(0.0, 1.0) + triangle.1;
      }
      if triangle_deltas_flat.2.perp_dot(point.xy() - triangle.2.xy()) < 0.0 {
        let s = triangle_deltas_flat.2.dot(point.xy() - triangle.2.xy())
          / triangle_deltas_flat.2.length_squared();
        return triangle_deltas.2 * s.clamp(0.0, 1.0) + triangle.2;
      }

      let normal = -triangle_deltas.0.cross(triangle_deltas.2).normalize();
      let height = normal.dot(point - triangle.0) / normal.z;
      Vec3::new(point.x, point.y, point.z - height)
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
  /// Determines the vertices corresponding to `edge`.
  pub(crate) fn get_edge_indices(&self, edge: usize) -> (usize, usize) {
    (
      self.vertices[edge],
      self.vertices[if edge == self.vertices.len() - 1 { 0 } else { edge + 1 }],
    )
  }
}

#[cfg(test)]
#[path = "nav_mesh_test.rs"]
mod test;

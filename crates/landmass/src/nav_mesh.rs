use std::{
  cmp::Ordering,
  collections::{HashMap, HashSet},
  marker::PhantomData,
};

use disjoint::DisjointSet;
use glam::{Vec3, swizzles::Vec3Swizzles};
use thiserror::Error;

use crate::{
  coords::{CoordinateSystem, PointSampleDistance},
  util::BoundingBox,
};

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
  /// A height mesh to accurately represent the height of the surface. See
  /// [`HeightNavigationMesh`] for more details. If [`None`], uses the regular
  /// polygons as the height of the surface.
  pub height_mesh: Option<HeightNavigationMesh<CS>>,
}

/// An (optional) part of the navigation mesh dedicated to "refining" the height
/// of a point.
///
/// [`NavigationMesh`] is used for the actual pathfinding. However, in order to
/// get there we need to know where to start (and end) the search. Or in other
/// words, we need to know which polygons the search should start and end at.
///
/// Navgation meshes may have very coarse/simplified polygons for improving
/// search efficiency. However this can come at the cost of accurately
/// representing the height of the geometry. This means an agent "on the ground"
/// may actually be above or below the nav mesh. This "height mesh" is
/// explicitly for accurately representing the height so that agents on the
/// ground will be recognized as being on the correct node. In other words, we
/// use the height mesh to figure out where the agent is, then use the regular
/// nav mesh to do the actual pathfinding.
pub struct HeightNavigationMesh<CS: CoordinateSystem> {
  /// The list of height polygons that correspond to the original polygons.
  ///
  /// The length of this [`Vec`] must match the number of
  /// [`NavigationMesh::polygons`].
  pub polygons: Vec<HeightPolygon>,
  /// The list of vertices that make up the height mesh.
  ///
  /// These are a pool of vertices that can be used to create the triangles.
  pub vertices: Vec<CS::Coordinate>,
  /// The list of triangles that make up the height mesh.
  ///
  /// The indices that make up the triangle are relative to
  /// [`HeightPolygon::base_vertex_index`]. The indices must be oriented
  /// counter-clockwise.
  pub triangles: Vec<[u8; 3]>,
}

/// A polygon specifically for "refining" the height of a point.
///
/// While regular polygons are used for finding paths, this polygon is used to
/// help determine which node a given point is on.
#[derive(Clone, Debug)]
pub struct HeightPolygon {
  /// The index of the first vertex in [`HeightNavigationMesh::vertices`] used
  /// by this polygon's triangles. The indices that make up a triangle are
  /// relative to this index.
  pub base_vertex_index: u32,
  /// The number of vertices that contribute to the triangles. Note only the
  /// vertices in `base_vertex_index..(base_vertex_index + vertex_count)`
  /// should be used by these triangles.
  pub vertex_count: u32,
  /// The index of the first triangle in [`HeightNavigationMesh::triangles`]
  /// that makes up this polygon.
  pub base_triangle_index: u32,
  /// The number of triangles that make up this polygon. This will always be
  /// the number after the [`Self::base_triangle_index`].
  pub triangle_count: u32,
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
      height_mesh: self.height_mesh.clone(),
    }
  }
}

impl<CS: CoordinateSystem> Clone for HeightNavigationMesh<CS> {
  fn clone(&self) -> Self {
    Self {
      polygons: self.polygons.clone(),
      vertices: self.vertices.clone(),
      triangles: self.triangles.clone(),
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
  #[error(
    "The polygon at index {0} contains a degenerate edge (an edge with zero length)."
  )]
  DegenerateEdgeInPolygon(usize),
  /// Stores the indices of the two vertices that make up the edge.
  #[error(
    "The edge made from vertices {0} and {1} is used by more than two polygons."
  )]
  DoublyConnectedEdge(usize, usize),
  #[error(transparent)]
  HeightMeshError(#[from] ValidateHeightMeshError),
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Error)]
pub enum ValidateHeightMeshError {
  /// Stores the number of polygons in the regular mesh and the number in the
  /// height mesh.
  #[error(
    "The height mesh contains a different number of polygons ({1}) than the regular mesh, which has {0}"
  )]
  IncorrectNumberOfPolygons(usize, usize),
  /// Stores the index of the invalid polygon.
  #[error(
    "The polygon at index {0} in the height mesh contains out of range indices"
  )]
  InvalidPolygonIndices(usize),
  #[error(
    "The triangle at index {triangle} in the height mesh contains an out of range vertex index {vertex}"
  )]
  InvalidIndexInTriangle { triangle: u32, vertex: usize },
  #[error(
    "The triangle at index {0} in the height mesh is clockwise instead of counter-clockwise"
  )]
  ClockwiseTriangle(usize),
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

    let height_mesh = match self.height_mesh {
      None => None,
      Some(height_mesh) => Some(height_mesh.validate(self.polygons.len())?),
    };

    let vertices =
      self.vertices.iter().map(CS::to_landmass).collect::<Vec<_>>();

    // Use the height mesh for more accurate bounds when available, but
    // otherwise, fall back to the regular mesh.
    let mesh_bounds = if let Some(height_mesh) = height_mesh.as_ref() {
      &height_mesh.vertices
    } else {
      &vertices
    }
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
      .map(|(polygon_index, polygon_vertices)| {
        // If we're using a height mesh, we want the bounds of the polygon to
        // use the height mesh's vertices, so that sampling doesn't miss the
        // height polygon due to bad bounds.
        let bounds = if let Some(height_mesh) = height_mesh.as_ref() {
          let polygon = &height_mesh.polygons[polygon_index];
          let range = polygon.base_vertex_index
            ..(polygon.base_vertex_index + polygon.vertex_count);
          range.fold(BoundingBox::Empty, |bounds, vertex| {
            bounds.expand_to_point(height_mesh.vertices[vertex as usize])
          })
        } else {
          polygon_vertices.iter().fold(BoundingBox::Empty, |bounds, vertex| {
            bounds.expand_to_point(vertices[*vertex])
          })
        };
        ValidPolygon {
          bounds,
          // We still use the non-height polygon for the center since we just
          // need a rough approximation.
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
        }
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
          polygons[polygon_1].connectivity[edge_1] = Some(Connectivity {
            polygon_index: polygon_2,
            reverse_edge: edge_2,
          });
          polygons[polygon_2].connectivity[edge_2] = Some(Connectivity {
            polygon_index: polygon_1,
            reverse_edge: edge_1,
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
      height_mesh,
      marker: Default::default(),
    })
  }
}

impl<CS: CoordinateSystem> HeightNavigationMesh<CS> {
  fn validate(
    self,
    expected_polygons: usize,
  ) -> Result<ValidHeightNavigationMesh, ValidateHeightMeshError> {
    if self.polygons.len() != expected_polygons {
      return Err(ValidateHeightMeshError::IncorrectNumberOfPolygons(
        expected_polygons,
        self.polygons.len(),
      ));
    }

    let vertices: Vec<Vec3> =
      self.vertices.into_iter().map(|v| CS::to_landmass(&v)).collect();

    for (polygon_index, polygon) in self.polygons.iter().enumerate() {
      let last_triangle = polygon.base_triangle_index + polygon.triangle_count;
      if last_triangle as usize > self.triangles.len() {
        return Err(ValidateHeightMeshError::InvalidPolygonIndices(
          polygon_index,
        ));
      }

      for triangle_index in polygon.base_triangle_index..last_triangle {
        let triangle = self.triangles[triangle_index as usize];

        let check_index = |index| {
          let index = index as usize;
          let real_index = index + polygon.base_vertex_index as usize;
          if real_index >= vertices.len()
            || index >= polygon.vertex_count as usize
          {
            Err(ValidateHeightMeshError::InvalidIndexInTriangle {
              triangle: triangle_index,
              vertex: real_index,
            })
          } else {
            Ok(real_index)
          }
        };
        let [a, b, c] = triangle;
        let a = check_index(a)?;
        let b = check_index(b)?;
        let c = check_index(c)?;

        let a = vertices[a].xy();
        let b = vertices[b].xy();
        let c = vertices[c].xy();

        if (b - a).perp_dot(c - a) < 0.0 {
          return Err(ValidateHeightMeshError::ClockwiseTriangle(
            polygon_index,
          ));
        }
      }
    }

    Ok(ValidHeightNavigationMesh {
      polygons: self.polygons,
      vertices,
      triangles: self.triangles,
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
  /// The height mesh used to "refine" point positions. See
  /// [`HeightNavigationMesh`] for more details.
  pub(crate) height_mesh: Option<ValidHeightNavigationMesh>,
  /// Marker for the CoordinateSystem.
  pub(crate) marker: PhantomData<CS>,
}

/// A version of [`HeightNavigationMesh`] after it has been validated and
/// converted to the standard coordinate system.
#[derive(Clone, Debug)]
pub(crate) struct ValidHeightNavigationMesh {
  /// The list of height polygons that correspond to the original polygons.
  ///
  /// The length of this [`Vec`] must match the number of
  /// [`ValidNavigationMesh::polygons`].
  pub(crate) polygons: Vec<HeightPolygon>,
  /// The list of vertices that make up the height mesh.
  ///
  /// These are a pool of vertices that can be used to create the triangles.
  pub(crate) vertices: Vec<Vec3>,
  /// The list of triangles that make up the height mesh.
  ///
  /// The indices that make up the triangle are relative to
  /// [`HeightPolygon::base_vertex_index`].
  pub(crate) triangles: Vec<[u8; 3]>,
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
      height_mesh: self.height_mesh.clone(),
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
      .field("height_mesh", &self.height_mesh)
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
  /// The index of the edge that would take us back to the original node.
  pub(crate) reverse_edge: usize,
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
    let (left_vertex_index, right_vertex_index) =
      polygon.get_edge_indices(edge_ref.edge_index);

    (self.vertices[left_vertex_index], self.vertices[right_vertex_index])
  }

  /// Finds the node nearest to (and within `distance_to_node` of) `point`.
  /// Returns the point on the nav mesh nearest to `point` and the index of the
  /// polygon.
  pub(crate) fn sample_point(
    &self,
    point: Vec3,
    point_sample_distance: &CS::SampleDistance,
  ) -> Option<(Vec3, usize)> {
    let sample_box = BoundingBox::new_box(
      point
        + Vec3::new(
          -point_sample_distance.horizontal_distance(),
          -point_sample_distance.horizontal_distance(),
          -point_sample_distance.distance_below(),
        ),
      point
        + Vec3::new(
          point_sample_distance.horizontal_distance(),
          point_sample_distance.horizontal_distance(),
          point_sample_distance.distance_above(),
        ),
    );

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

      // Whether we are using the normal mesh or the height mesh, we want the
      // triangles to be handled the same. So factor out the test and turn it
      // into a closure we can call in both cases.
      let mut test_triangle = |triangle: (Vec3, Vec3, Vec3)| {
        let projected_point = project_to_triangle(triangle, point);

        let distance_to_triangle_horizontal =
          point.xy().distance(projected_point.xy());
        let distance_to_triangle_vertical = projected_point.z - point.z;
        if distance_to_triangle_horizontal
          < point_sample_distance.horizontal_distance()
          && (-point_sample_distance.distance_below()
            ..point_sample_distance.distance_above())
            .contains(&distance_to_triangle_vertical)
        {
          let distance_to_triangle = distance_to_triangle_horizontal
            * point_sample_distance.vertical_preference_ratio()
            + distance_to_triangle_vertical.abs();
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
      };

      if let Some(height_mesh) = self.height_mesh.as_ref() {
        let height_polygon = &height_mesh.polygons[polygon_index];
        let vertex_base = height_polygon.base_vertex_index as usize;
        for i in height_polygon.base_triangle_index
          ..(height_polygon.base_triangle_index + height_polygon.triangle_count)
        {
          let [a, b, c] = &height_mesh.triangles[i as usize];

          let triangle = (
            height_mesh.vertices[vertex_base + *a as usize],
            height_mesh.vertices[vertex_base + *b as usize],
            height_mesh.vertices[vertex_base + *c as usize],
          );
          test_triangle(triangle);
        }
      } else {
        for i in 2..polygon.vertices.len() {
          let triangle =
            (polygon.vertices[0], polygon.vertices[i - 1], polygon.vertices[i]);
          let triangle = (
            self.vertices[triangle.0],
            self.vertices[triangle.1],
            self.vertices[triangle.2],
          );
          test_triangle(triangle);
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

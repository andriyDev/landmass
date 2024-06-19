use std::{
  collections::{HashMap, HashSet},
  mem::swap,
};

use geo::{BooleanOps, Coord, LineString, LinesIter, MultiPolygon, Polygon};
use glam::{Vec2, Vec3, Vec3Swizzles};
use kdtree::{distance::squared_euclidean, KdTree};
use rand::{thread_rng, Rng};

use crate::{
  geometry::edge_intersection,
  island::{Island, IslandId, IslandNavigationData},
  nav_mesh::MeshEdgeRef,
  util::BoundingBoxHierarchy,
  BoundingBox,
};

/// The navigation data of a whole [`crate::Archipelago`]. This only includes
/// "static" features.
pub struct NavigationData {
  /// The islands in the [`crate::Archipelago`].
  pub islands: HashMap<IslandId, Island>,
  /// The links to other islands by [`crate::NodeRef`]
  pub boundary_links: HashMap<NodeRef, HashMap<BoundaryLinkId, BoundaryLink>>,
  /// The nodes that have been modified.
  pub modified_nodes: HashMap<NodeRef, ModifiedNode>,
  /// The islands that have been deleted since the last update.
  pub deleted_islands: HashSet<IslandId>,
}

/// A reference to a node in the navigation data.
#[derive(PartialEq, Eq, Debug, Clone, Copy, Hash, PartialOrd, Ord)]
pub struct NodeRef {
  /// The island of the node.
  pub island_id: IslandId,
  /// The index of the node in the island.
  pub polygon_index: usize,
}

/// The ID of a boundary link.
pub type BoundaryLinkId = u32;

/// A single link between two nodes on the boundary of an island.
#[derive(PartialEq, Debug, Clone)]
pub struct BoundaryLink {
  /// The node that taking this link leads to.
  pub destination_node: NodeRef,
  /// The portal that this link occupies on the boundary of the source node.
  /// This is essentially the intersection of the linked islands' linkable
  /// edges.
  pub portal: (Vec3, Vec3),
}

/// A node that has been modified (e.g., by being connected with a boundary link
/// to another island).
#[derive(PartialEq, Debug, Clone)]
pub struct ModifiedNode {
  /// The new (2D) edges that make up the boundary of this node. These are
  /// indices in the nav mesh this corresponds to. Indices larger than the nav
  /// mesh vertices refer to [`ModifiedNode::new_vertices`]. Note the boundary
  /// winds in the same direction as nav mesh polygons (CCW).
  pub new_boundary: Vec<(usize, usize)>,
  /// The "new" vertices (in world space) that are needed for the modified
  /// node. These are not vertices in the original nav mesh and should only
  /// be used by a single boundary edge.
  pub new_vertices: Vec<Vec2>,
}

impl NavigationData {
  /// Creates new navigation data.
  pub fn new() -> Self {
    Self {
      islands: HashMap::new(),
      boundary_links: HashMap::new(),
      modified_nodes: HashMap::new(),
      deleted_islands: HashSet::new(),
    }
  }

  /// Finds the node nearest to (and within `distance_to_node` of) `point`.
  /// Returns the point on the nav data nearest to `point` and the reference to
  /// the corresponding node.
  pub fn sample_point(
    &self,
    point: Vec3,
    distance_to_node: f32,
  ) -> Option<(Vec3, NodeRef)> {
    let mut best_point = None;
    for (island_id, island) in self.islands.iter() {
      let nav_data = match &island.nav_data {
        None => continue,
        Some(data) => data,
      };

      let relative_point = nav_data.transform.apply_inverse(point);
      if !nav_data
        .nav_mesh
        .mesh_bounds
        .expand_by_size(Vec3::ONE * distance_to_node)
        .contains_point(relative_point)
      {
        continue;
      }

      let (sampled_point, sampled_node) = match nav_data
        .nav_mesh
        .sample_point(relative_point, distance_to_node)
      {
        Some(sampled) => sampled,
        None => continue,
      };

      let distance = relative_point.distance_squared(sampled_point);
      match best_point {
        Some((best_distance, _)) if distance >= best_distance => continue,
        _ => {}
      }

      best_point = Some((
        distance,
        (
          nav_data.transform.apply(sampled_point),
          NodeRef { island_id: *island_id, polygon_index: sampled_node },
        ),
      ));
    }
    best_point.map(|(_, b)| b)
  }

  fn update_islands(
    &mut self,
    edge_link_distance: f32,
  ) -> (HashSet<BoundaryLinkId>, HashSet<IslandId>, HashSet<NodeRef>) {
    let mut dirty_islands = HashSet::new();
    for (&island_id, island) in self.islands.iter_mut() {
      if island.dirty {
        island.dirty = false;
        dirty_islands.insert(island_id);
      }
    }

    let changed_islands = self
      .deleted_islands
      .union(&dirty_islands)
      .copied()
      .collect::<HashSet<_>>();

    let mut dropped_links = HashSet::new();
    let mut modified_node_refs_to_update = HashSet::new();
    if !self.deleted_islands.is_empty() || !dirty_islands.is_empty() {
      self.boundary_links.retain(|node_ref, links| {
        if changed_islands.contains(&node_ref.island_id) {
          modified_node_refs_to_update.insert(*node_ref);
          dropped_links.extend(links.keys().copied());
          return false;
        }

        let links_before = links.len();

        links.retain(|&link_id, link| {
          let retain =
            !changed_islands.contains(&link.destination_node.island_id);
          if !retain {
            dropped_links.insert(link_id);
          }
          retain
        });

        if links_before != links.len() {
          // If a node has a different set of boundary links, we need to
          // recompute that node.
          modified_node_refs_to_update.insert(*node_ref);
        }

        !links.is_empty()
      });
    }

    self.deleted_islands.clear();

    if dirty_islands.is_empty() {
      // No new or changed islands, so no need to check for new links.
      return (dropped_links, changed_islands, modified_node_refs_to_update);
    }

    let mut island_bounds = self
      .islands
      .iter()
      .filter_map(|(key, value)| {
        value
          .nav_data
          .as_ref()
          .map(|nav_data| (nav_data.transformed_bounds, *key))
      })
      .collect::<Vec<_>>();
    // There are no islands with nav data, so no islands to link and prevents a
    // panic.
    if island_bounds.is_empty() {
      return (dropped_links, changed_islands, modified_node_refs_to_update);
    }
    let island_bbh = BoundingBoxHierarchy::new(&mut island_bounds);

    for &dirty_island_id in dirty_islands.iter() {
      let dirty_island = self.islands.get(&dirty_island_id).unwrap();
      let dirty_nav_data = match &dirty_island.nav_data {
        None => continue,
        Some(n) => n,
      };
      if dirty_nav_data.nav_mesh.boundary_edges.is_empty() {
        continue;
      }
      let query = dirty_nav_data
        .transformed_bounds
        .expand_by_size(Vec3::ONE * edge_link_distance);
      let candidate_islands = island_bbh.query_box(query);
      // If the only candidate island is the dirty island itself, skip the rest.
      if candidate_islands.len() <= 1 {
        continue;
      }
      let dirty_island_edge_bbh = island_edges_bbh(dirty_nav_data);
      for &candidate_island_id in candidate_islands {
        if candidate_island_id == dirty_island_id {
          continue;
        }
        // `link_edges_between_islands` links forwards and backwards. This means
        // for a pair of dirty islands, we must only process them once, so only
        // execute `link_edges_between_islands` for one ordering of the island
        // ids and not the other.
        if dirty_islands.contains(&candidate_island_id)
          && candidate_island_id < dirty_island_id
        {
          continue;
        }
        let candidate_island = self.islands.get(&candidate_island_id).unwrap();
        link_edges_between_islands(
          (dirty_island_id, dirty_island),
          (candidate_island_id, candidate_island),
          &dirty_island_edge_bbh,
          edge_link_distance,
          &mut self.boundary_links,
          &mut modified_node_refs_to_update,
          &mut thread_rng(),
        );
      }
    }

    (dropped_links, changed_islands, modified_node_refs_to_update)
  }

  fn update_modified_node(
    &mut self,
    node_ref: NodeRef,
    edge_link_distance: f32,
  ) {
    // Any node from an island that doesn't exist (deleted), or one without nav
    // data (the nav mesh was removed), should be removed.
    let Some(island_nav_data) = self
      .islands
      .get(&node_ref.island_id)
      .and_then(|island| island.nav_data.as_ref())
    else {
      self.modified_nodes.remove(&node_ref);
      return;
    };
    // Any nodes without boundary links don't need to be modified.
    let Some(boundary_links) = self.boundary_links.get(&node_ref) else {
      self.modified_nodes.remove(&node_ref);
      return;
    };

    let polygon = &island_nav_data.nav_mesh.polygons[node_ref.polygon_index];
    let connectivity =
      &island_nav_data.nav_mesh.connectivity[node_ref.polygon_index];

    let connected_edges = connectivity
      .iter()
      .map(|connectivity| connectivity.edge_index)
      .collect::<HashSet<usize>>();

    fn vec2_to_coord(v: Vec2) -> Coord<f32> {
      Coord::from((v.x, v.y))
    }

    fn coord_to_vec2(c: Coord<f32>) -> Vec2 {
      Vec2::new(c.x, c.y)
    }

    fn push_vertex(
      vertex: usize,
      nav_data: &IslandNavigationData,
      line_string: &mut Vec<Coord<f32>>,
    ) {
      let vertex = nav_data.transform.apply(nav_data.nav_mesh.vertices[vertex]);
      line_string.push(vec2_to_coord(vertex.xz()));
    }

    let mut multi_line_string = vec![];
    let mut current_line_string = vec![];

    for (i, vertex) in polygon.vertices.iter().copied().enumerate() {
      if connected_edges.contains(&i) {
        if !current_line_string.is_empty() {
          let mut line_string = vec![];
          swap(&mut current_line_string, &mut line_string);

          // Add the right vertex of the previous edge (the current vertex).
          push_vertex(vertex, island_nav_data, &mut line_string);

          multi_line_string.push(LineString(line_string));
        }
        continue;
      }

      // Add the left vertex. The right vertex will be added when we finish the
      // line string.
      push_vertex(vertex, island_nav_data, &mut current_line_string);
    }

    if !current_line_string.is_empty() {
      // The last "closing" edge must not be connected, so add the first vertex
      // to finish that edge and push it into the `multi_line_string`.
      push_vertex(
        polygon.vertices[0],
        island_nav_data,
        &mut current_line_string,
      );

      multi_line_string.push(LineString(current_line_string));
    }

    let boundary_edges = geo::MultiLineString(multi_line_string);

    fn boundary_link_to_clip_polygon(
      link: &BoundaryLink,
      edge_link_distance: f32,
    ) -> MultiPolygon<f32> {
      let flat_portal = (link.portal.0.xz(), link.portal.1.xz());
      let portal_forward =
        (flat_portal.1 - flat_portal.0).normalize().perp() * edge_link_distance;
      MultiPolygon::new(vec![Polygon::new(
        LineString(vec![
          vec2_to_coord(flat_portal.0 + portal_forward),
          vec2_to_coord(flat_portal.0 - portal_forward),
          vec2_to_coord(flat_portal.1 - portal_forward),
          vec2_to_coord(flat_portal.1 + portal_forward),
        ]),
        vec![],
      )])
    }

    let mut clip_polygons = boundary_links
      .values()
      .map(|link| boundary_link_to_clip_polygon(link, edge_link_distance));

    let mut link_clip = clip_polygons.next().unwrap();
    for clip_polygon in clip_polygons {
      link_clip = link_clip.union(&clip_polygon);
    }

    let clipped_boundary_edges =
      link_clip.clip(&boundary_edges, /* invert= */ true);

    let mut original_vertices = KdTree::new(/* dimensions= */ 2);
    for &index in polygon.vertices.iter() {
      let vertex = island_nav_data
        .transform
        .apply(island_nav_data.nav_mesh.vertices[index]);

      original_vertices
        .add([vertex.x, vertex.z], index)
        .expect("Vertex is valid");
    }

    let modified_node =
      self.modified_nodes.entry(node_ref).or_insert_with(|| ModifiedNode {
        new_boundary: Vec::new(),
        new_vertices: Vec::new(),
      });

    for line_string in clipped_boundary_edges.iter() {
      for edge in line_string.lines_iter() {
        let start_index = original_vertices
          .nearest(
            &[edge.start.x, edge.start.y],
            /* num= */ 1,
            &squared_euclidean,
          )
          .unwrap()
          .first()
          .filter(|&(distance, _)| *distance < 0.01)
          .map(|&(_, &index)| index);

        let end_index = original_vertices
          .nearest(
            &[edge.end.x, edge.end.y],
            /* num= */ 1,
            &squared_euclidean,
          )
          .unwrap()
          .first()
          .filter(|&(distance, _)| *distance < 0.01)
          .map(|&(_, &index)| index);

        if let (Some(start_index), Some(end_index)) = (start_index, end_index) {
          // We don't want degenerate edges, so ignore edges with equal indices.
          if start_index == end_index {
            continue;
          }
        }

        let mut start_index = start_index.unwrap_or_else(|| {
          modified_node.new_vertices.push(coord_to_vec2(edge.start));
          island_nav_data.nav_mesh.vertices.len()
            + modified_node.new_vertices.len()
            - 1
        });

        let mut end_index = end_index.unwrap_or_else(|| {
          modified_node.new_vertices.push(coord_to_vec2(edge.end));
          island_nav_data.nav_mesh.vertices.len()
            + modified_node.new_vertices.len()
            - 1
        });

        let polygon_center =
          island_nav_data.transform.apply(polygon.center).xz();

        // Ensure the winding order of the modified node boundary matches the
        // polygon edges.
        if (coord_to_vec2(edge.start) - polygon_center)
          .perp_dot(coord_to_vec2(edge.end) - polygon_center)
          < 0.0
        {
          swap(&mut start_index, &mut end_index);
        }

        modified_node.new_boundary.push((start_index, end_index));
      }
    }
  }

  pub fn update(
    &mut self,
    edge_link_distance: f32,
  ) -> (HashSet<BoundaryLinkId>, HashSet<IslandId>) {
    let (dropped_links, changed_islands, modified_node_refs_to_update) =
      self.update_islands(edge_link_distance);
    for node_ref in modified_node_refs_to_update {
      self.update_modified_node(node_ref, edge_link_distance);
    }
    (dropped_links, changed_islands)
  }
}

fn edge_ref_to_world_edge(
  edge: MeshEdgeRef,
  nav_data: &IslandNavigationData,
) -> (Vec3, Vec3) {
  let (a, b) = nav_data.nav_mesh.get_edge_points(edge);
  (nav_data.transform.apply(a), nav_data.transform.apply(b))
}

fn edge_to_bbox(edge: (Vec3, Vec3)) -> BoundingBox {
  BoundingBox::new_box(edge.0, edge.0).expand_to_point(edge.1)
}

fn island_edges_bbh(
  island_nav_data: &IslandNavigationData,
) -> BoundingBoxHierarchy<MeshEdgeRef> {
  let mut edge_bounds = island_nav_data
    .nav_mesh
    .boundary_edges
    .iter()
    .map(|edge| {
      (
        edge_to_bbox(edge_ref_to_world_edge(edge.clone(), island_nav_data)),
        edge.clone(),
      )
    })
    .collect::<Vec<_>>();
  BoundingBoxHierarchy::new(&mut edge_bounds)
}

trait CreateBoundaryId {
  fn create(&mut self) -> BoundaryLinkId;
}

impl<T: Rng> CreateBoundaryId for T {
  fn create(&mut self) -> BoundaryLinkId {
    self.gen()
  }
}

fn link_edges_between_islands(
  island_1: (IslandId, &Island),
  island_2: (IslandId, &Island),
  island_1_edge_bbh: &BoundingBoxHierarchy<MeshEdgeRef>,
  edge_link_distance: f32,
  boundary_links: &mut HashMap<NodeRef, HashMap<BoundaryLinkId, BoundaryLink>>,
  modified_node_refs_to_update: &mut HashSet<NodeRef>,
  id_creator: &mut impl CreateBoundaryId,
) {
  let island_1_nav_data = island_1.1.nav_data.as_ref().unwrap();
  let island_2_nav_data = island_2.1.nav_data.as_ref().unwrap();

  let edge_link_distance_squared = edge_link_distance * edge_link_distance;

  for island_2_edge_ref in island_2_nav_data.nav_mesh.boundary_edges.iter() {
    let island_2_edge =
      edge_ref_to_world_edge(island_2_edge_ref.clone(), &island_2_nav_data);
    let island_2_edge_bbox = edge_to_bbox(island_2_edge)
      .expand_by_size(Vec3::ONE * edge_link_distance);

    for island_1_edge_ref in island_1_edge_bbh.query_box(island_2_edge_bbox) {
      let island_1_edge =
        edge_ref_to_world_edge(island_1_edge_ref.clone(), &island_1_nav_data);
      if let Some(portal) = edge_intersection(
        island_1_edge,
        island_2_edge,
        edge_link_distance_squared,
      ) {
        if portal.0.distance_squared(portal.1) < edge_link_distance_squared {
          continue;
        }

        let node_1 = NodeRef {
          island_id: island_1.0,
          polygon_index: island_1_edge_ref.polygon_index,
        };
        let node_2 = NodeRef {
          island_id: island_2.0,
          polygon_index: island_2_edge_ref.polygon_index,
        };

        let id = id_creator.create();

        boundary_links.entry(node_1.clone()).or_default().insert(
          id,
          BoundaryLink { destination_node: node_2.clone(), portal: portal },
        );
        boundary_links.entry(node_2).or_default().insert(
          id,
          BoundaryLink {
            destination_node: node_1,
            portal: (portal.1, portal.0),
          },
        );
        modified_node_refs_to_update.insert(node_1);
        modified_node_refs_to_update.insert(node_2);
      }
    }
  }
}

#[cfg(test)]
#[path = "nav_data_test.rs"]
mod test;

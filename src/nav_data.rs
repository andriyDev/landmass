use std::{
  collections::{HashMap, HashSet},
  mem::swap,
};

use geo::{BooleanOps, Coord, LineString, LinesIter, MultiPolygon, Polygon};
use glam::{Vec2, Vec3, Vec3Swizzles};

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
  pub boundary_links: HashMap<NodeRef, Vec<BoundaryLink>>,
  pub modified_nodes: HashMap<NodeRef, ModifiedNode>,
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

#[derive(PartialEq, Debug, Clone)]
pub struct BoundaryLink {
  // The node that taking this link leads to.
  pub destination_node: NodeRef,
  // The portal that this link occupies on the boundary of the source node.
  // This is essentially the intersection of the linked islands' linkable
  // edges.
  pub portal: (Vec3, Vec3),
}

#[derive(PartialEq, Debug, Clone)]
pub struct ModifiedNode {
  // The new (2D) edges that make up the boundary of this node. These are used
  // for local collision avoidance, which already assumes "flatness".
  pub new_boundary: Vec<(Vec2, Vec2)>,
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
  ) -> (HashSet<IslandId>, HashSet<NodeRef>) {
    let mut dirty_islands = HashSet::new();
    for (&island_id, island) in self.islands.iter_mut() {
      if island.dirty {
        island.dirty = false;
        dirty_islands.insert(island_id);
      }
    }

    let mut modified_node_refs_to_update = HashSet::new();
    if !self.deleted_islands.is_empty() || !dirty_islands.is_empty() {
      let changed_islands =
        self.deleted_islands.union(&dirty_islands).collect::<HashSet<_>>();
      self.boundary_links.retain(|node_ref, links| {
        if changed_islands.contains(&node_ref.island_id) {
          modified_node_refs_to_update.insert(*node_ref);
          return false;
        }

        let links_before = links.len();

        links.retain(|link| {
          !changed_islands.contains(&link.destination_node.island_id)
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
      return (dirty_islands, modified_node_refs_to_update);
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
      return (dirty_islands, modified_node_refs_to_update);
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
        );
      }
    }

    (dirty_islands, modified_node_refs_to_update)
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

      // Add the left vertex. The right vertex will be added when we finish the line string.
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

    let link_clip = boundary_links.iter().skip(1).fold(
      boundary_link_to_clip_polygon(&boundary_links[0], edge_link_distance),
      |sum_clip, link| {
        let new_clip = boundary_link_to_clip_polygon(link, edge_link_distance);
        sum_clip.union(&new_clip)
      },
    );

    let clipped_boundary_edges =
      link_clip.clip(&boundary_edges, /*invert=*/ true);

    let modified_node = self
      .modified_nodes
      .entry(node_ref)
      .or_insert_with(|| ModifiedNode { new_boundary: Vec::new() });

    for line_string in clipped_boundary_edges.iter() {
      for edge in line_string.lines_iter() {
        modified_node
          .new_boundary
          .push((coord_to_vec2(edge.start), coord_to_vec2(edge.end)));
      }
    }
  }

  pub fn update(&mut self, edge_link_distance: f32) -> HashSet<u32> {
    let (dirty_islands, modified_node_refs_to_update) =
      self.update_islands(edge_link_distance);
    for node_ref in modified_node_refs_to_update {
      self.update_modified_node(node_ref, edge_link_distance);
    }
    dirty_islands
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

fn link_edges_between_islands(
  island_1: (IslandId, &Island),
  island_2: (IslandId, &Island),
  island_1_edge_bbh: &BoundingBoxHierarchy<MeshEdgeRef>,
  edge_link_distance: f32,
  boundary_links: &mut HashMap<NodeRef, Vec<BoundaryLink>>,
  modified_node_refs_to_update: &mut HashSet<NodeRef>,
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

        boundary_links.entry(node_1.clone()).or_default().push(BoundaryLink {
          destination_node: node_2.clone(),
          portal: portal,
        });
        boundary_links.entry(node_2).or_default().push(BoundaryLink {
          destination_node: node_1,
          portal: (portal.1, portal.0),
        });
        modified_node_refs_to_update.insert(node_1);
        modified_node_refs_to_update.insert(node_2);
      }
    }
  }
}

#[cfg(test)]
mod tests {
  use std::{
    cmp::Ordering,
    collections::{HashMap, HashSet},
    f32::consts::PI,
    sync::Arc,
  };

  use glam::{Vec2, Vec3};

  use crate::{
    island::Island,
    nav_data::{BoundaryLink, NodeRef},
    nav_mesh::NavigationMesh,
    Transform,
  };

  use super::{
    island_edges_bbh, link_edges_between_islands, ModifiedNode, NavigationData,
  };

  #[test]
  fn samples_points() {
    let nav_mesh = NavigationMesh {
      mesh_bounds: None,
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 1.0),
        Vec3::new(3.0, 1.0, 1.0),
        Vec3::new(4.0, 1.0, 1.0),
        Vec3::new(4.0, 1.0, 2.0),
        Vec3::new(3.0, 1.0, 2.0),
        Vec3::new(2.0, 1.0, 2.0),
        Vec3::new(1.0, 1.0, 2.0),
      ],
      polygons: vec![vec![0, 1, 6, 7], vec![1, 2, 5, 6], vec![2, 3, 4, 5]],
    }
    .validate()
    .expect("is valid");
    let nav_mesh = Arc::new(nav_mesh);

    let mut islands = HashMap::new();

    let mut island_1 = Island::new();
    island_1.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
    );

    let mut island_2 = Island::new();
    island_2.set_nav_mesh(
      Transform { translation: Vec3::new(5.0, 0.1, 0.0), rotation: PI * -0.5 },
      Arc::clone(&nav_mesh),
    );

    islands.insert(1, island_1);
    islands.insert(2, island_2);

    let mut nav_data = NavigationData::new();
    nav_data.islands = islands;
    // Just above island 1 node.
    assert_eq!(
      nav_data.sample_point(
        Vec3::new(1.5, 1.09, 1.5),
        /* distance_to_node= */ 0.1,
      ),
      Some((
        Vec3::new(1.5, 1.0, 1.5),
        NodeRef { island_id: 1, polygon_index: 0 }
      )),
    );
    // Just outside island 1 node.
    assert_eq!(
      nav_data.sample_point(
        Vec3::new(0.95, 0.95, 0.95),
        /* distance_to_node= */ 0.1,
      ),
      Some((
        Vec3::new(1.0, 1.0, 1.0),
        NodeRef { island_id: 1, polygon_index: 0 }
      )),
    );
    // At overlap, but closer to island 1.
    assert_eq!(
      nav_data.sample_point(
        Vec3::new(3.5, 1.04, 1.5),
        /* distance_to_node= */ 0.1,
      ),
      Some((
        Vec3::new(3.5, 1.0, 1.5),
        NodeRef { island_id: 1, polygon_index: 2 }
      )),
    );
    // At overlap, but closer to island 2.
    assert_eq!(
      nav_data
        .sample_point(
          Vec3::new(3.5, 1.06, 1.5),
          /* distance_to_node= */ 0.1,
        )
        .map(|(p, n)| ((p * 1e6).round() / 1e6, n)),
      Some((
        Vec3::new(3.5, 1.1, 1.5),
        NodeRef { island_id: 2, polygon_index: 0 }
      )),
    );
  }

  fn clone_sort_round_links(
    boundary_links: &HashMap<NodeRef, Vec<BoundaryLink>>,
    round_amount: f32,
  ) -> Vec<(NodeRef, Vec<BoundaryLink>)> {
    fn node_ref_to_num(node_ref: &NodeRef) -> u32 {
      node_ref.island_id as u32 * 100 + node_ref.polygon_index as u32
    }

    let mut links = boundary_links
      .iter()
      .map(|(key, value)| {
        (*key, {
          let mut v = value
            .iter()
            .map(|link| BoundaryLink {
              destination_node: link.destination_node,
              portal: (
                (link.portal.0 / round_amount).round() * round_amount,
                (link.portal.1 / round_amount).round() * round_amount,
              ),
            })
            .collect::<Vec<_>>();
          v.sort_by_key(|link| node_ref_to_num(&link.destination_node));
          v
        })
      })
      .collect::<Vec<_>>();
    links.sort_by_key(|(a, _)| node_ref_to_num(a));
    links
  }

  #[test]
  fn link_edges_between_islands_links_touching_islands() {
    let nav_mesh_1 = Arc::new(
      NavigationMesh {
        mesh_bounds: None,
        vertices: vec![
          Vec3::new(1.0, 1.0, 0.0),
          Vec3::new(1.0, 1.0, 1.0),
          Vec3::new(-1.0, 1.0, 1.0),
          Vec3::new(-1.0, 1.0, 0.0),
          Vec3::new(-1.0, 1.0, -1.0),
          Vec3::new(1.0, 1.0, -1.0),
          //
          Vec3::new(2.0, 1.0, 0.0),
          Vec3::new(2.0, 1.0, 2.0),
          Vec3::new(-2.0, 1.0, 2.0),
          Vec3::new(-2.0, 1.0, 0.0),
          Vec3::new(-2.0, 1.0, -2.0),
          Vec3::new(2.0, 1.0, -2.0),
        ],
        polygons: vec![
          vec![0, 6, 7, 1],
          vec![1, 7, 8, 2],
          vec![2, 8, 9, 3],
          vec![10, 4, 3, 9],
          vec![10, 11, 5, 4],
          vec![6, 0, 5, 11],
        ],
      }
      .validate()
      .expect("is valid."),
    );

    let nav_mesh_2 = Arc::new(
      NavigationMesh {
        mesh_bounds: None,
        vertices: vec![
          Vec3::new(-1.0, 1.0, -0.5),
          Vec3::new(-0.5, 1.0, -0.5),
          Vec3::new(0.5, 1.0, -0.5),
          Vec3::new(1.0, 1.0, -0.5),
          Vec3::new(-1.0, 1.0, 0.5),
          Vec3::new(-0.5, 1.0, 0.5),
          Vec3::new(0.5, 1.0, 0.5),
          Vec3::new(1.0, 1.0, 0.5),
          Vec3::new(-0.5, 1.0, 1.0),
          Vec3::new(0.5, 1.0, 1.0),
          Vec3::new(-0.5, 1.0, -1.0),
          Vec3::new(0.5, 1.0, -1.0),
        ],
        polygons: vec![
          vec![5, 4, 0, 1],
          vec![1, 2, 6, 5],
          vec![3, 7, 6, 2],
          vec![5, 6, 9, 8],
          vec![10, 11, 2, 1],
        ],
      }
      .validate()
      .expect("is valid."),
    );

    let island_1_id = 1;
    let island_2_id = 2;

    let mut island_1 = Island::new();
    let mut island_2 = Island::new();

    let transform =
      Transform { translation: Vec3::new(1.0, 2.0, 3.0), rotation: PI * 0.25 };
    island_1.set_nav_mesh(transform, Arc::clone(&nav_mesh_1));
    island_2.set_nav_mesh(transform, Arc::clone(&nav_mesh_2));

    let island_1_edge_bbh =
      island_edges_bbh(island_1.nav_data.as_ref().unwrap());
    let island_2_edge_bbh =
      island_edges_bbh(island_2.nav_data.as_ref().unwrap());

    let mut boundary_links = HashMap::new();
    let mut modified_node_refs_to_update = HashSet::new();

    link_edges_between_islands(
      (island_1_id, &island_1),
      (island_2_id, &island_2),
      &island_1_edge_bbh,
      /* edge_link_distance= */ 1e-5,
      &mut boundary_links,
      &mut modified_node_refs_to_update,
    );

    fn transform_and_round_portal(
      transform: Transform,
      a: Vec3,
      b: Vec3,
    ) -> (Vec3, Vec3) {
      (
        (transform.apply(a) * 1e6).round() * 1e-6,
        (transform.apply(b) * 1e6).round() * 1e-6,
      )
    }

    let expected_links = [
      (
        NodeRef { island_id: island_1_id, polygon_index: 0 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_2_id,
            polygon_index: 2,
          },
          portal: transform_and_round_portal(
            transform,
            Vec3::new(1.0, 1.0, 0.5),
            Vec3::new(1.0, 1.0, 0.0),
          ),
        }],
      ),
      (
        NodeRef { island_id: island_1_id, polygon_index: 1 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_2_id,
            polygon_index: 3,
          },
          portal: transform_and_round_portal(
            transform,
            Vec3::new(-0.5, 1.0, 1.0),
            Vec3::new(0.5, 1.0, 1.0),
          ),
        }],
      ),
      (
        NodeRef { island_id: island_1_id, polygon_index: 2 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_2_id,
            polygon_index: 0,
          },
          portal: transform_and_round_portal(
            transform,
            Vec3::new(-1.0, 1.0, 0.0),
            Vec3::new(-1.0, 1.0, 0.5),
          ),
        }],
      ),
      (
        NodeRef { island_id: island_1_id, polygon_index: 3 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_2_id,
            polygon_index: 0,
          },
          portal: transform_and_round_portal(
            transform,
            Vec3::new(-1.0, 1.0, -0.5),
            Vec3::new(-1.0, 1.0, 0.0),
          ),
        }],
      ),
      (
        NodeRef { island_id: island_1_id, polygon_index: 4 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_2_id,
            polygon_index: 4,
          },
          portal: transform_and_round_portal(
            transform,
            Vec3::new(0.5, 1.0, -1.0),
            Vec3::new(-0.5, 1.0, -1.0),
          ),
        }],
      ),
      (
        NodeRef { island_id: island_1_id, polygon_index: 5 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_2_id,
            polygon_index: 2,
          },
          portal: transform_and_round_portal(
            transform,
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(1.0, 1.0, -0.5),
          ),
        }],
      ),
      // Reverse links
      (
        NodeRef { island_id: island_2_id, polygon_index: 0 },
        vec![
          BoundaryLink {
            destination_node: NodeRef {
              island_id: island_1_id,
              polygon_index: 2,
            },
            portal: transform_and_round_portal(
              transform,
              Vec3::new(-1.0, 1.0, 0.5),
              Vec3::new(-1.0, 1.0, 0.0),
            ),
          },
          BoundaryLink {
            destination_node: NodeRef {
              island_id: island_1_id,
              polygon_index: 3,
            },
            portal: transform_and_round_portal(
              transform,
              Vec3::new(-1.0, 1.0, 0.0),
              Vec3::new(-1.0, 1.0, -0.5),
            ),
          },
        ],
      ),
      (
        NodeRef { island_id: island_2_id, polygon_index: 2 },
        vec![
          BoundaryLink {
            destination_node: NodeRef {
              island_id: island_1_id,
              polygon_index: 0,
            },
            portal: transform_and_round_portal(
              transform,
              Vec3::new(1.0, 1.0, 0.0),
              Vec3::new(1.0, 1.0, 0.5),
            ),
          },
          BoundaryLink {
            destination_node: NodeRef {
              island_id: island_1_id,
              polygon_index: 5,
            },
            portal: transform_and_round_portal(
              transform,
              Vec3::new(1.0, 1.0, -0.5),
              Vec3::new(1.0, 1.0, 0.0),
            ),
          },
        ],
      ),
      (
        NodeRef { island_id: island_2_id, polygon_index: 3 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 1,
          },
          portal: transform_and_round_portal(
            transform,
            Vec3::new(0.5, 1.0, 1.0),
            Vec3::new(-0.5, 1.0, 1.0),
          ),
        }],
      ),
      (
        NodeRef { island_id: island_2_id, polygon_index: 4 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 4,
          },
          portal: transform_and_round_portal(
            transform,
            Vec3::new(-0.5, 1.0, -1.0),
            Vec3::new(0.5, 1.0, -1.0),
          ),
        }],
      ),
    ];
    assert_eq!(clone_sort_round_links(&boundary_links, 1e-6), &expected_links);

    let mut modified_node_refs_to_update_sorted =
      modified_node_refs_to_update.iter().copied().collect::<Vec<_>>();
    modified_node_refs_to_update_sorted.sort();
    assert_eq!(
      modified_node_refs_to_update_sorted,
      [
        NodeRef { island_id: island_1_id, polygon_index: 0 },
        NodeRef { island_id: island_1_id, polygon_index: 1 },
        NodeRef { island_id: island_1_id, polygon_index: 2 },
        NodeRef { island_id: island_1_id, polygon_index: 3 },
        NodeRef { island_id: island_1_id, polygon_index: 4 },
        NodeRef { island_id: island_1_id, polygon_index: 5 },
        //
        NodeRef { island_id: island_2_id, polygon_index: 0 },
        NodeRef { island_id: island_2_id, polygon_index: 2 },
        NodeRef { island_id: island_2_id, polygon_index: 3 },
        NodeRef { island_id: island_2_id, polygon_index: 4 },
      ]
    );

    boundary_links = HashMap::new();
    modified_node_refs_to_update = HashSet::new();

    link_edges_between_islands(
      (island_2_id, &island_2),
      (island_1_id, &island_1),
      &island_2_edge_bbh,
      /* edge_link_distance= */ 1e-5,
      &mut boundary_links,
      &mut modified_node_refs_to_update,
    );
    assert_eq!(clone_sort_round_links(&boundary_links, 1e-6), &expected_links);
  }

  #[test]
  fn update_links_islands_and_unlinks_on_delete() {
    let nav_mesh = Arc::new(
      NavigationMesh {
        mesh_bounds: None,
        vertices: vec![
          Vec3::new(1.0, 1.0, 0.0),
          Vec3::new(2.0, 1.0, 0.0),
          Vec3::new(2.0, 1.0, 2.0),
          Vec3::new(0.0, 1.0, 2.0),
          Vec3::new(0.0, 1.0, 1.0),
          Vec3::new(1.0, 1.0, 1.0),
        ],
        polygons: vec![vec![0, 1, 2, 5], vec![4, 5, 2, 3]],
      }
      .validate()
      .expect("is valid."),
    );

    let island_1_id = 1;
    let island_2_id = 2;
    let island_3_id = 3;
    let island_4_id = 4;
    let island_5_id = 5;

    let mut island_1 = Island::new();
    let mut island_2 = Island::new();
    let mut island_3 = Island::new();
    let mut island_4 = Island::new();
    let mut island_5 = Island::new();

    island_1.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
    );
    island_2.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: PI * -0.5 },
      Arc::clone(&nav_mesh),
    );
    island_3.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: PI },
      Arc::clone(&nav_mesh),
    );
    island_4.set_nav_mesh(
      Transform { translation: Vec3::new(3.0, 0.0, 0.0), rotation: PI },
      Arc::clone(&nav_mesh),
    );
    island_5.set_nav_mesh(
      Transform { translation: Vec3::new(2.0, 0.0, 3.0), rotation: PI * 0.5 },
      Arc::clone(&nav_mesh),
    );

    let mut nav_data = NavigationData::new();
    nav_data.islands.insert(island_1_id, island_1);
    nav_data.islands.insert(island_2_id, island_2);
    nav_data.islands.insert(island_3_id, island_3);
    nav_data.islands.insert(island_4_id, island_4);
    nav_data.islands.insert(island_5_id, island_5);

    nav_data.update(/* edge_link_distance= */ 0.01);

    assert_eq!(
      clone_sort_round_links(&nav_data.boundary_links, 1e-6),
      [
        (
          NodeRef { island_id: island_1_id, polygon_index: 0 },
          vec![
            BoundaryLink {
              destination_node: NodeRef {
                island_id: island_4_id,
                polygon_index: 0,
              },
              portal: (Vec3::new(1.0, 1.0, 0.0), Vec3::new(2.0, 1.0, 0.0)),
            },
            BoundaryLink {
              destination_node: NodeRef {
                island_id: island_5_id,
                polygon_index: 0,
              },
              portal: (Vec3::new(2.0, 1.0, 1.0), Vec3::new(2.0, 1.0, 2.0)),
            },
          ],
        ),
        (
          NodeRef { island_id: island_1_id, polygon_index: 1 },
          vec![BoundaryLink {
            destination_node: NodeRef {
              island_id: island_2_id,
              polygon_index: 0,
            },
            portal: (Vec3::new(0.0, 1.0, 2.0), Vec3::new(0.0, 1.0, 1.0)),
          }],
        ),
        (
          NodeRef { island_id: island_2_id, polygon_index: 0 },
          vec![BoundaryLink {
            destination_node: NodeRef {
              island_id: island_1_id,
              polygon_index: 1,
            },
            portal: (Vec3::new(0.0, 1.0, 1.0), Vec3::new(0.0, 1.0, 2.0)),
          }],
        ),
        (
          NodeRef { island_id: island_2_id, polygon_index: 1 },
          vec![BoundaryLink {
            destination_node: NodeRef {
              island_id: island_3_id,
              polygon_index: 0,
            },
            portal: (Vec3::new(-2.0, 1.0, 0.0), Vec3::new(-1.0, 1.0, 0.0)),
          }],
        ),
        (
          NodeRef { island_id: island_3_id, polygon_index: 0 },
          vec![BoundaryLink {
            destination_node: NodeRef {
              island_id: island_2_id,
              polygon_index: 1,
            },
            portal: (Vec3::new(-1.0, 1.0, 0.0), Vec3::new(-2.0, 1.0, 0.0)),
          }],
        ),
        (
          NodeRef { island_id: island_4_id, polygon_index: 0 },
          vec![BoundaryLink {
            destination_node: NodeRef {
              island_id: island_1_id,
              polygon_index: 0,
            },
            portal: (Vec3::new(2.0, 1.0, 0.0), Vec3::new(1.0, 1.0, 0.0)),
          }],
        ),
        (
          NodeRef { island_id: island_5_id, polygon_index: 0 },
          vec![BoundaryLink {
            destination_node: NodeRef {
              island_id: island_1_id,
              polygon_index: 0,
            },
            portal: (Vec3::new(2.0, 1.0, 2.0), Vec3::new(2.0, 1.0, 1.0)),
          }],
        ),
      ],
    );

    // Delete island_2 and island_4.
    nav_data.islands.remove(&island_2_id);
    nav_data.islands.remove(&island_4_id);
    // Move island_5 to replace island_2.
    nav_data
      .islands
      .get_mut(&island_5_id)
      .expect("island_5 still exists")
      .set_nav_mesh(
        Transform { translation: Vec3::ZERO, rotation: PI * -0.5 },
        Arc::clone(&nav_mesh),
      );
    // Record the deletions so the update can react to them.
    nav_data.deleted_islands.insert(island_2_id);
    nav_data.deleted_islands.insert(island_4_id);

    nav_data.update(/* edge_link_distance= */ 0.01);

    assert_eq!(
      clone_sort_round_links(&nav_data.boundary_links, 1e-6),
      [
        (
          NodeRef { island_id: island_1_id, polygon_index: 1 },
          vec![BoundaryLink {
            destination_node: NodeRef {
              island_id: island_5_id,
              polygon_index: 0,
            },
            portal: (Vec3::new(0.0, 1.0, 2.0), Vec3::new(0.0, 1.0, 1.0)),
          }],
        ),
        (
          NodeRef { island_id: island_3_id, polygon_index: 0 },
          vec![BoundaryLink {
            destination_node: NodeRef {
              island_id: island_5_id,
              polygon_index: 1,
            },
            portal: (Vec3::new(-1.0, 1.0, 0.0), Vec3::new(-2.0, 1.0, 0.0)),
          }],
        ),
        (
          NodeRef { island_id: island_5_id, polygon_index: 0 },
          vec![BoundaryLink {
            destination_node: NodeRef {
              island_id: island_1_id,
              polygon_index: 1,
            },
            portal: (Vec3::new(0.0, 1.0, 1.0), Vec3::new(0.0, 1.0, 2.0)),
          }],
        ),
        (
          NodeRef { island_id: island_5_id, polygon_index: 1 },
          vec![BoundaryLink {
            destination_node: NodeRef {
              island_id: island_3_id,
              polygon_index: 0,
            },
            portal: (Vec3::new(-2.0, 1.0, 0.0), Vec3::new(-1.0, 1.0, 0.0)),
          }],
        ),
      ],
    );
  }

  fn clone_sort_round_modified_nodes(
    modified_nodes: &HashMap<NodeRef, ModifiedNode>,
    round_amount: f32,
  ) -> Vec<(NodeRef, ModifiedNode)> {
    fn order_vec2(a: Vec2, b: Vec2) -> Ordering {
      match a.x.partial_cmp(&b.x).unwrap() {
        Ordering::Equal => {}
        other => return other,
      }
      a.y.partial_cmp(&b.y).unwrap()
    }

    let mut nodes = modified_nodes
      .iter()
      .map(|(key, value)| {
        (*key, {
          let mut new_value = ModifiedNode {
            new_boundary: value
              .new_boundary
              .iter()
              .map(|(left, right)| {
                let rounded_edge = (
                  (*left / round_amount).round() * round_amount,
                  (*right / round_amount).round() * round_amount,
                );

                match order_vec2(rounded_edge.0, rounded_edge.1) {
                  Ordering::Equal | Ordering::Less => rounded_edge,
                  Ordering::Greater => (rounded_edge.1, rounded_edge.0),
                }
              })
              .filter(|(left, right)| left != right)
              .collect(),
          };

          new_value.new_boundary.sort_by(
            |a: &(Vec2, Vec2), b: &(Vec2, Vec2)| match order_vec2(a.0, b.0) {
              Ordering::Equal => order_vec2(a.1, b.1),
              other => other,
            },
          );

          new_value
        })
      })
      .collect::<Vec<_>>();
    nodes.sort_by_key(|(node_ref, _)| {
      node_ref.island_id as u32 * 100 + node_ref.polygon_index as u32
    });
    nodes
  }

  #[test]
  fn modifies_node_boundaries_for_linked_islands() {
    let nav_mesh = Arc::new(
      NavigationMesh {
        mesh_bounds: None,
        vertices: vec![
          Vec3::new(1.0, 1.0, 1.0),
          Vec3::new(2.0, 1.0, 1.0),
          Vec3::new(2.0, 1.0, 2.0),
          Vec3::new(1.0, 1.0, 2.0),
          Vec3::new(2.0, 1.0, 3.0),
          Vec3::new(1.0, 1.0, 3.0),
        ],
        polygons: vec![vec![0, 1, 2, 3], vec![3, 2, 4, 5]],
      }
      .validate()
      .expect("is valid."),
    );

    let island_1_id = 1;
    let island_2_id = 2;
    let island_3_id = 3;

    let mut island_1 = Island::new();
    let mut island_2 = Island::new();
    let mut island_3 = Island::new();

    island_1.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
    );
    island_2.set_nav_mesh(
      Transform { translation: Vec3::new(1.0, 0.0, -1.0), rotation: 0.0 },
      Arc::clone(&nav_mesh),
    );
    island_3.set_nav_mesh(
      Transform { translation: Vec3::new(2.0, 0.0, 3.0), rotation: PI * 0.5 },
      Arc::clone(&nav_mesh),
    );

    let mut nav_data = NavigationData::new();
    nav_data.islands.insert(island_1_id, island_1);
    nav_data.islands.insert(island_2_id, island_2);
    nav_data.islands.insert(island_3_id, island_3);

    nav_data.update(/*edge_link_distance=*/ 1e-6);

    let expected_modified_nodes = [
      (
        NodeRef { island_id: island_1_id, polygon_index: 0 },
        ModifiedNode {
          new_boundary: vec![
            (Vec2::new(1.0, 1.0), Vec2::new(1.0, 2.0)),
            (Vec2::new(1.0, 1.0), Vec2::new(2.0, 1.0)),
          ],
        },
      ),
      (
        NodeRef { island_id: island_2_id, polygon_index: 1 },
        ModifiedNode {
          new_boundary: vec![(Vec2::new(2.0, 2.0), Vec2::new(3.0, 2.0))],
        },
      ),
      (
        NodeRef { island_id: island_3_id, polygon_index: 0 },
        ModifiedNode {
          new_boundary: vec![
            (Vec2::new(3.0, 1.0), Vec2::new(4.0, 1.0)),
            (Vec2::new(3.0, 2.0), Vec2::new(4.0, 2.0)),
          ],
        },
      ),
    ];

    assert_eq!(
      clone_sort_round_modified_nodes(&nav_data.modified_nodes, 1e-4),
      &expected_modified_nodes
    );
  }

  #[test]
  fn stale_modified_nodes_are_removed() {
    let nav_mesh = Arc::new(
      NavigationMesh {
        mesh_bounds: None,
        vertices: vec![
          Vec3::new(1.0, 1.0, 1.0),
          Vec3::new(2.0, 1.0, 1.0),
          Vec3::new(2.0, 1.0, 2.0),
          Vec3::new(1.0, 1.0, 2.0),
          Vec3::new(2.0, 1.0, 3.0),
          Vec3::new(1.0, 1.0, 3.0),
        ],
        polygons: vec![vec![0, 1, 2, 3], vec![3, 2, 4, 5]],
      }
      .validate()
      .expect("is valid."),
    );

    let island_1_id = 1;
    let island_2_id = 2;

    let mut island_1 = Island::new();
    let mut island_2 = Island::new();

    island_1.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
    );
    island_2.set_nav_mesh(
      Transform { translation: Vec3::new(1.0, 0.0, -1.0), rotation: 0.0 },
      Arc::clone(&nav_mesh),
    );

    let mut nav_data = NavigationData::new();
    nav_data.islands.insert(island_1_id, island_1);
    nav_data.islands.insert(island_2_id, island_2);

    nav_data.update(/*edge_link_distance=*/ 1e-6);

    assert_eq!(nav_data.modified_nodes.len(), 2);

    nav_data.islands.remove(&island_2_id);
    nav_data.deleted_islands.insert(island_2_id);

    nav_data.update(/*edge_link_distance=*/ 1e-6);

    assert_eq!(nav_data.modified_nodes.len(), 0);
  }
}

use std::collections::{HashMap, HashSet};

use glam::Vec3;

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
  pub deleted_islands: HashSet<IslandId>,
}

/// A reference to a node in the navigation data.
#[derive(PartialEq, Eq, Debug, Clone, Copy, Hash)]
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

impl NavigationData {
  /// Creates new navigation data.
  pub fn new() -> Self {
    Self {
      islands: HashMap::new(),
      boundary_links: HashMap::new(),
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

  pub fn update(&mut self, edge_link_distance: f32) {
    let mut dirty_islands = HashSet::new();
    for (&island_id, island) in self.islands.iter_mut() {
      island.dirty = false;
      dirty_islands.insert(island_id);
    }

    if !self.deleted_islands.is_empty() || !dirty_islands.is_empty() {
      let changed_islands =
        self.deleted_islands.union(&dirty_islands).collect::<HashSet<_>>();
      self.boundary_links.retain(|node_ref, links| {
        if changed_islands.contains(&node_ref.island_id) {
          return false;
        }

        links.retain(|link| {
          !changed_islands.contains(&link.destination_node.island_id)
        });

        !links.is_empty()
      });
    }

    self.deleted_islands.clear();

    if dirty_islands.is_empty() {
      // No new or changed islands, so no need to check for new links.
      return;
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
      return;
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
        );
      }
    }
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
      }
    }
  }
}

#[cfg(test)]
mod tests {
  use std::{collections::HashMap, f32::consts::PI, sync::Arc};

  use glam::Vec3;

  use crate::{
    island::Island,
    nav_data::{BoundaryLink, NodeRef},
    nav_mesh::NavigationMesh,
    Transform,
  };

  use super::{island_edges_bbh, link_edges_between_islands, NavigationData};

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
                (link.portal.0 * 1e6).round() * 1e-6,
                (link.portal.1 * 1e6).round() * 1e-6,
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

    link_edges_between_islands(
      (island_1_id, &island_1),
      (island_2_id, &island_2),
      &island_1_edge_bbh,
      /* edge_link_distance= */ 1e-5,
      &mut boundary_links,
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
    assert_eq!(clone_sort_round_links(&boundary_links), &expected_links);

    boundary_links = HashMap::new();

    link_edges_between_islands(
      (island_2_id, &island_2),
      (island_1_id, &island_1),
      &island_2_edge_bbh,
      /* edge_link_distance= */ 1e-5,
      &mut boundary_links,
    );
    assert_eq!(clone_sort_round_links(&boundary_links), &expected_links);
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
      clone_sort_round_links(&nav_data.boundary_links),
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
      clone_sort_round_links(&nav_data.boundary_links),
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
}

use std::{
  cmp::Ordering,
  collections::{HashMap, HashSet},
  f32::consts::PI,
  sync::Arc,
};

use glam::{Vec2, Vec3};
use rand::thread_rng;

use crate::{
  island::Island,
  nav_data::{BoundaryLink, NodeRef},
  nav_mesh::NavigationMesh,
  Transform,
};

use super::{
  island_edges_bbh, link_edges_between_islands, BoundaryLinkId, ModifiedNode,
  NavigationData,
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
      .sample_point(Vec3::new(3.5, 1.06, 1.5), /* distance_to_node= */ 0.1,)
      .map(|(p, n)| ((p * 1e6).round() / 1e6, n)),
    Some((
      Vec3::new(3.5, 1.1, 1.5),
      NodeRef { island_id: 2, polygon_index: 0 }
    )),
  );
}

fn clone_sort_round_links(
  boundary_links: &HashMap<NodeRef, HashMap<BoundaryLinkId, BoundaryLink>>,
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
          .values()
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

  let island_1_edge_bbh = island_edges_bbh(island_1.nav_data.as_ref().unwrap());
  let island_2_edge_bbh = island_edges_bbh(island_2.nav_data.as_ref().unwrap());

  let mut boundary_links = HashMap::new();
  let mut modified_node_refs_to_update = HashSet::new();

  link_edges_between_islands(
    (island_1_id, &island_1),
    (island_2_id, &island_2),
    &island_1_edge_bbh,
    /* edge_link_distance= */ 1e-5,
    &mut boundary_links,
    &mut modified_node_refs_to_update,
    &mut thread_rng(),
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
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 2 },
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
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 3 },
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
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 0 },
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
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 0 },
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
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 4 },
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
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 2 },
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
        destination_node: NodeRef { island_id: island_1_id, polygon_index: 1 },
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
        destination_node: NodeRef { island_id: island_1_id, polygon_index: 4 },
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
    &mut thread_rng(),
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

        new_value.new_boundary.sort_by(|a: &(Vec2, Vec2), b: &(Vec2, Vec2)| {
          match order_vec2(a.0, b.0) {
            Ordering::Equal => order_vec2(a.1, b.1),
            other => other,
          }
        });

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

  nav_data.update(/* edge_link_distance= */ 1e-6);

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

  nav_data.update(/* edge_link_distance= */ 1e-6);

  assert_eq!(nav_data.modified_nodes.len(), 2);

  nav_data.islands.remove(&island_2_id);
  nav_data.deleted_islands.insert(island_2_id);

  nav_data.update(/* edge_link_distance= */ 1e-6);

  assert_eq!(nav_data.modified_nodes.len(), 0);
}
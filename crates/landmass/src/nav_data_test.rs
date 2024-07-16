use std::{
  cmp::Ordering,
  collections::{HashMap, HashSet},
  f32::consts::PI,
  sync::Arc,
};

use glam::{Vec2, Vec3};
use slotmap::{HopSlotMap, SlotMap};

use crate::{
  coords::{XY, XYZ},
  island::Island,
  nav_data::{BoundaryLink, NodeRef},
  nav_mesh::NavigationMesh,
  Archipelago, IslandId, Transform,
};

use super::{
  island_edges_bbh, link_edges_between_islands, BoundaryLinkId, ModifiedNode,
  NavigationData,
};

#[test]
fn samples_points() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 1.0, 1.0),
      Vec3::new(2.0, 1.0, 1.0),
      Vec3::new(3.0, 1.0, 1.0),
      Vec3::new(4.0, 1.0, 1.0),
      Vec3::new(4.0, 2.0, 1.0),
      Vec3::new(3.0, 2.0, 1.0),
      Vec3::new(2.0, 2.0, 1.0),
      Vec3::new(1.0, 2.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 6, 7], vec![1, 2, 5, 6], vec![2, 3, 4, 5]],
    polygon_type_indices: vec![0, 0, 0],
  }
  .validate()
  .expect("is valid");
  let nav_mesh = Arc::new(nav_mesh);

  let mut nav_data = NavigationData::<XYZ>::new();
  let island_id_1 = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();
  let island_id_2 = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::new(5.0, 0.0, 0.1), rotation: PI * 0.5 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();

  // Just above island 1 node.
  assert_eq!(
    nav_data.sample_point(
      Vec3::new(1.5, 1.5, 1.09),
      /* distance_to_node= */ 0.1,
    ),
    Some((
      Vec3::new(1.5, 1.5, 1.0),
      NodeRef { island_id: island_id_1, polygon_index: 0 }
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
      NodeRef { island_id: island_id_1, polygon_index: 0 }
    )),
  );
  // At overlap, but closer to island 1.
  assert_eq!(
    nav_data.sample_point(
      Vec3::new(3.5, 1.5, 1.04),
      /* distance_to_node= */ 0.1,
    ),
    Some((
      Vec3::new(3.5, 1.5, 1.0),
      NodeRef { island_id: island_id_1, polygon_index: 2 }
    )),
  );
  // At overlap, but closer to island 2.
  assert_eq!(
    nav_data
      .sample_point(Vec3::new(3.5, 1.5, 1.06), /* distance_to_node= */ 0.1,)
      .map(|(p, n)| ((p * 1e6).round() / 1e6, n)),
    Some((
      Vec3::new(3.5, 1.5, 1.1),
      NodeRef { island_id: island_id_2, polygon_index: 0 }
    )),
  );
}

fn node_ref_to_num(node_ref: &NodeRef, island_order: &[IslandId]) -> u32 {
  let island_index =
    island_order.iter().position(|id| node_ref.island_id == *id).unwrap();
  island_index as u32 * 100 + node_ref.polygon_index as u32
}

fn clone_sort_round_links(
  boundary_links: &SlotMap<BoundaryLinkId, BoundaryLink>,
  node_to_boundary_link_ids: &HashMap<NodeRef, HashSet<BoundaryLinkId>>,
  island_order: &[IslandId],
  round_amount: f32,
) -> Vec<(NodeRef, Vec<BoundaryLink>)> {
  let mut links = node_to_boundary_link_ids
    .iter()
    .map(|(key, value)| {
      (*key, {
        let mut v = value
          .iter()
          .map(|link_id| boundary_links.get(*link_id).unwrap())
          .map(|link| BoundaryLink {
            destination_node: link.destination_node,
            portal: (
              (link.portal.0 / round_amount).round() * round_amount,
              (link.portal.1 / round_amount).round() * round_amount,
            ),
            cost: (link.cost / round_amount).round() * round_amount,
          })
          .collect::<Vec<_>>();
        v.sort_by_key(|link| {
          node_ref_to_num(&link.destination_node, &island_order)
        });
        v
      })
    })
    .collect::<Vec<_>>();
  links.sort_by_key(|(a, _)| node_ref_to_num(a, &island_order));
  links
}

fn chain_length_rounded(chain: &[Vec2], round_amount: f32) -> f32 {
  let mut sum = 0.0;
  for i in 0..(chain.len() - 1) {
    sum += chain[i].distance(chain[i + 1]);
  }
  (sum / round_amount).round() * round_amount
}

#[test]
fn link_edges_between_islands_links_touching_islands() {
  let nav_mesh_1 = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(1.0, 0.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(-1.0, 1.0, 1.0),
        Vec3::new(-1.0, 0.0, 1.0),
        Vec3::new(-1.0, -1.0, 1.0),
        Vec3::new(1.0, -1.0, 1.0),
        //
        Vec3::new(2.0, 0.0, 1.0),
        Vec3::new(2.0, 2.0, 1.0),
        Vec3::new(-2.0, 2.0, 1.0),
        Vec3::new(-2.0, 0.0, 1.0),
        Vec3::new(-2.0, -2.0, 1.0),
        Vec3::new(2.0, -2.0, 1.0),
      ],
      polygons: vec![
        vec![0, 6, 7, 1],
        vec![1, 7, 8, 2],
        vec![2, 8, 9, 3],
        vec![10, 4, 3, 9],
        vec![10, 11, 5, 4],
        vec![6, 0, 5, 11],
      ],
      polygon_type_indices: vec![0, 0, 0, 0, 0, 0],
    }
    .validate()
    .expect("is valid."),
  );

  let nav_mesh_2 = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(-1.0, -0.5, 1.0),
        Vec3::new(-0.5, -0.5, 1.0),
        Vec3::new(0.5, -0.5, 1.0),
        Vec3::new(1.0, -0.5, 1.0),
        Vec3::new(-1.0, 0.5, 1.0),
        Vec3::new(-0.5, 0.5, 1.0),
        Vec3::new(0.5, 0.5, 1.0),
        Vec3::new(1.0, 0.5, 1.0),
        Vec3::new(-0.5, 1.0, 1.0),
        Vec3::new(0.5, 1.0, 1.0),
        Vec3::new(-0.5, -1.0, 1.0),
        Vec3::new(0.5, -1.0, 1.0),
      ],
      polygons: vec![
        vec![5, 4, 0, 1],
        vec![1, 2, 6, 5],
        vec![3, 7, 6, 2],
        vec![5, 6, 9, 8],
        vec![10, 11, 2, 1],
      ],
      polygon_type_indices: vec![0, 0, 0, 0, 0],
    }
    .validate()
    .expect("is valid."),
  );

  // Create an unused slotmap just to get `IslandId`s.
  let mut slotmap = HopSlotMap::<IslandId, _>::with_key();
  let island_1_id = slotmap.insert(0);
  let island_2_id = slotmap.insert(0);

  let mut island_1 = Island::new();
  let mut island_2 = Island::new();

  let transform =
    Transform { translation: Vec3::new(1.0, 2.0, 3.0), rotation: PI * -0.25 };
  island_1.set_nav_mesh(
    transform.clone(),
    Arc::clone(&nav_mesh_1),
    HashMap::new(),
  );
  island_2.set_nav_mesh(
    transform.clone(),
    Arc::clone(&nav_mesh_2),
    HashMap::new(),
  );

  let island_1_edge_bbh = island_edges_bbh(island_1.nav_data.as_ref().unwrap());
  let island_2_edge_bbh = island_edges_bbh(island_2.nav_data.as_ref().unwrap());

  let mut boundary_links = SlotMap::with_key();
  let mut node_to_boundary_link_ids = HashMap::new();
  let mut modified_node_refs_to_update = HashSet::new();

  link_edges_between_islands(
    (island_1_id, &island_1),
    (island_2_id, &island_2),
    &island_1_edge_bbh,
    /* edge_link_distance= */ 1e-5,
    &mut boundary_links,
    &mut node_to_boundary_link_ids,
    &mut modified_node_refs_to_update,
  );

  fn transform_and_round_portal(
    transform: &Transform<XYZ>,
    a: Vec3,
    b: Vec3,
  ) -> (Vec3, Vec3) {
    (
      (transform.apply(a) * 1e6).round() * 1e-6,
      (transform.apply(b) * 1e6).round() * 1e-6,
    )
  }

  // The cost of a link between island_2's horizontal nodes, and island_1's
  // left/right nodes. All these are symmetric, so we can just pick a candidate.
  let diagonal_link_cost = chain_length_rounded(
    &[Vec2::new(0.75, 0.0), Vec2::new(1.0, 0.25), Vec2::new(1.5, 0.75)],
    1e-6,
  );
  // The same logic applies for the vertical connections.
  let vertical_link_cost =
    chain_length_rounded(&[Vec2::new(0.0, 1.5), Vec2::new(0.0, 0.75)], 1e-6);

  let expected_links = [
    (
      NodeRef { island_id: island_1_id, polygon_index: 0 },
      vec![BoundaryLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 2 },
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(1.0, 0.5, 1.0),
          Vec3::new(1.0, 0.0, 1.0),
        ),
        cost: diagonal_link_cost,
      }],
    ),
    (
      NodeRef { island_id: island_1_id, polygon_index: 1 },
      vec![BoundaryLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 3 },
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(-0.5, 1.0, 1.0),
          Vec3::new(0.5, 1.0, 1.0),
        ),
        cost: vertical_link_cost,
      }],
    ),
    (
      NodeRef { island_id: island_1_id, polygon_index: 2 },
      vec![BoundaryLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 0 },
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(-1.0, 0.0, 1.0),
          Vec3::new(-1.0, 0.5, 1.0),
        ),
        cost: diagonal_link_cost,
      }],
    ),
    (
      NodeRef { island_id: island_1_id, polygon_index: 3 },
      vec![BoundaryLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 0 },
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(-1.0, -0.5, 1.0),
          Vec3::new(-1.0, 0.0, 1.0),
        ),
        cost: diagonal_link_cost,
      }],
    ),
    (
      NodeRef { island_id: island_1_id, polygon_index: 4 },
      vec![BoundaryLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 4 },
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(0.5, -1.0, 1.0),
          Vec3::new(-0.5, -1.0, 1.0),
        ),
        cost: vertical_link_cost,
      }],
    ),
    (
      NodeRef { island_id: island_1_id, polygon_index: 5 },
      vec![BoundaryLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 2 },
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(1.0, 0.0, 1.0),
          Vec3::new(1.0, -0.5, 1.0),
        ),
        cost: diagonal_link_cost,
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
            &transform,
            Vec3::new(-1.0, 0.5, 1.0),
            Vec3::new(-1.0, 0.0, 1.0),
          ),
          cost: diagonal_link_cost,
        },
        BoundaryLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 3,
          },
          portal: transform_and_round_portal(
            &transform,
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(-1.0, -0.5, 1.0),
          ),
          cost: diagonal_link_cost,
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
            &transform,
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 0.5, 1.0),
          ),
          cost: diagonal_link_cost,
        },
        BoundaryLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 5,
          },
          portal: transform_and_round_portal(
            &transform,
            Vec3::new(1.0, -0.5, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
          ),
          cost: diagonal_link_cost,
        },
      ],
    ),
    (
      NodeRef { island_id: island_2_id, polygon_index: 3 },
      vec![BoundaryLink {
        destination_node: NodeRef { island_id: island_1_id, polygon_index: 1 },
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(0.5, 1.0, 1.0),
          Vec3::new(-0.5, 1.0, 1.0),
        ),
        cost: vertical_link_cost,
      }],
    ),
    (
      NodeRef { island_id: island_2_id, polygon_index: 4 },
      vec![BoundaryLink {
        destination_node: NodeRef { island_id: island_1_id, polygon_index: 4 },
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(-0.5, -1.0, 1.0),
          Vec3::new(0.5, -1.0, 1.0),
        ),
        cost: vertical_link_cost,
      }],
    ),
  ];
  assert_eq!(
    clone_sort_round_links(
      &boundary_links,
      &node_to_boundary_link_ids,
      &[island_1_id, island_2_id],
      1e-6
    ),
    &expected_links
  );

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

  boundary_links = SlotMap::with_key();
  node_to_boundary_link_ids = HashMap::new();
  modified_node_refs_to_update = HashSet::new();

  link_edges_between_islands(
    (island_2_id, &island_2),
    (island_1_id, &island_1),
    &island_2_edge_bbh,
    /* edge_link_distance= */ 1e-5,
    &mut boundary_links,
    &mut node_to_boundary_link_ids,
    &mut modified_node_refs_to_update,
  );
  assert_eq!(
    clone_sort_round_links(
      &boundary_links,
      &node_to_boundary_link_ids,
      &[island_1_id, island_2_id],
      1e-6
    ),
    &expected_links
  );
}

#[test]
fn update_links_islands_and_unlinks_on_delete() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(1.0, 0.0, 1.0),
        Vec3::new(2.0, 0.0, 1.0),
        Vec3::new(2.0, 2.0, 1.0),
        Vec3::new(0.0, 2.0, 1.0),
        Vec3::new(0.0, 1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 5], vec![4, 5, 2, 3]],
      polygon_type_indices: vec![0, 0],
    }
    .validate()
    .expect("is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_1_id = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();
  let island_2_id = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: PI * 0.5 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();
  let island_3_id = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: PI },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();
  let island_4_id = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::new(3.0, 0.0, 0.0), rotation: PI },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();
  let island_5_id = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::new(2.0, 3.0, 0.0), rotation: PI * -0.5 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();

  nav_data.update(/* edge_link_distance= */ 0.01);

  let lined_up_cost =
    chain_length_rounded(&[Vec2::new(1.5, 0.75), Vec2::new(1.5, -0.75)], 1e-6);
  let offset_cost = chain_length_rounded(
    &[Vec2::new(1.5, 0.75), Vec2::new(2.0, 1.5), Vec2::new(2.75, 1.5)],
    1e-6,
  );

  assert_eq!(
    clone_sort_round_links(
      &nav_data.boundary_links,
      &nav_data.node_to_boundary_link_ids,
      &[island_1_id, island_2_id, island_3_id, island_4_id, island_5_id],
      1e-6
    ),
    [
      (
        NodeRef { island_id: island_1_id, polygon_index: 0 },
        vec![
          BoundaryLink {
            destination_node: NodeRef {
              island_id: island_4_id,
              polygon_index: 0,
            },
            portal: (Vec3::new(1.0, 0.0, 1.0), Vec3::new(2.0, 0.0, 1.0)),
            cost: lined_up_cost,
          },
          BoundaryLink {
            destination_node: NodeRef {
              island_id: island_5_id,
              polygon_index: 0,
            },
            portal: (Vec3::new(2.0, 1.0, 1.0), Vec3::new(2.0, 2.0, 1.0)),
            cost: offset_cost,
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
          portal: (Vec3::new(0.0, 2.0, 1.0), Vec3::new(0.0, 1.0, 1.0)),
          cost: lined_up_cost,
        }],
      ),
      (
        NodeRef { island_id: island_2_id, polygon_index: 0 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 1,
          },
          portal: (Vec3::new(0.0, 1.0, 1.0), Vec3::new(0.0, 2.0, 1.0)),
          cost: lined_up_cost,
        }],
      ),
      (
        NodeRef { island_id: island_2_id, polygon_index: 1 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_3_id,
            polygon_index: 0,
          },
          portal: (Vec3::new(-2.0, 0.0, 1.0), Vec3::new(-1.0, 0.0, 1.0)),
          cost: lined_up_cost,
        }],
      ),
      (
        NodeRef { island_id: island_3_id, polygon_index: 0 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_2_id,
            polygon_index: 1,
          },
          portal: (Vec3::new(-1.0, 0.0, 1.0), Vec3::new(-2.0, 0.0, 1.0)),
          cost: lined_up_cost,
        }],
      ),
      (
        NodeRef { island_id: island_4_id, polygon_index: 0 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 0,
          },
          portal: (Vec3::new(2.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 1.0)),
          cost: lined_up_cost,
        }],
      ),
      (
        NodeRef { island_id: island_5_id, polygon_index: 0 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 0,
          },
          portal: (Vec3::new(2.0, 2.0, 1.0), Vec3::new(2.0, 1.0, 1.0)),
          cost: offset_cost,
        }],
      ),
    ],
  );

  // Delete island_2 and island_4.
  nav_data.remove_island(island_2_id);
  nav_data.remove_island(island_4_id);
  // Move island_5 to replace island_2.
  nav_data
    .islands
    .get_mut(island_5_id)
    .expect("island_5 still exists")
    .set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: PI * 0.5 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    );

  nav_data.update(/* edge_link_distance= */ 0.01);

  assert_eq!(
    clone_sort_round_links(
      &nav_data.boundary_links,
      &nav_data.node_to_boundary_link_ids,
      &[island_1_id, island_2_id, island_3_id, island_4_id, island_5_id],
      1e-6
    ),
    [
      (
        NodeRef { island_id: island_1_id, polygon_index: 1 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_5_id,
            polygon_index: 0,
          },
          portal: (Vec3::new(0.0, 2.0, 1.0), Vec3::new(0.0, 1.0, 1.0)),
          cost: lined_up_cost,
        }],
      ),
      (
        NodeRef { island_id: island_3_id, polygon_index: 0 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_5_id,
            polygon_index: 1,
          },
          portal: (Vec3::new(-1.0, 0.0, 1.0), Vec3::new(-2.0, 0.0, 1.0)),
          cost: lined_up_cost,
        }],
      ),
      (
        NodeRef { island_id: island_5_id, polygon_index: 0 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 1,
          },
          portal: (Vec3::new(0.0, 1.0, 1.0), Vec3::new(0.0, 2.0, 1.0)),
          cost: lined_up_cost,
        }],
      ),
      (
        NodeRef { island_id: island_5_id, polygon_index: 1 },
        vec![BoundaryLink {
          destination_node: NodeRef {
            island_id: island_3_id,
            polygon_index: 0,
          },
          portal: (Vec3::new(-2.0, 0.0, 1.0), Vec3::new(-1.0, 0.0, 1.0)),
          cost: lined_up_cost,
        }],
      ),
    ],
  );
}

fn clone_sort_round_modified_nodes(
  modified_nodes: &HashMap<NodeRef, ModifiedNode>,
  island_id_to_vertices: &HashMap<IslandId, usize>,
  island_order: &[IslandId],
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
        let nav_mesh_vertices =
          *island_id_to_vertices.get(&key.island_id).unwrap();

        let new_vertices_rounded = value
          .new_vertices
          .iter()
          .map(|&point| (point / round_amount).round() * round_amount)
          .collect::<Vec<_>>();

        let mut new_vertices_sort_indices =
          (0..new_vertices_rounded.len()).collect::<Vec<_>>();
        new_vertices_sort_indices.sort_by(|&a, &b| {
          order_vec2(new_vertices_rounded[a], new_vertices_rounded[b])
        });

        let new_vertices_sorted = new_vertices_sort_indices
          .iter()
          .copied()
          .map(|index| new_vertices_rounded[index])
          .collect::<Vec<_>>();

        let mut new_boundary_sorted = value
          .new_boundary
          .iter()
          .map(|&(mut left, mut right)| {
            if left >= nav_mesh_vertices {
              left = new_vertices_sort_indices[left - nav_mesh_vertices]
                + nav_mesh_vertices;
            }
            if right >= nav_mesh_vertices {
              right = new_vertices_sort_indices[right - nav_mesh_vertices]
                + nav_mesh_vertices;
            }
            (left, right)
          })
          .collect::<Vec<_>>();

        new_boundary_sorted.sort();

        ModifiedNode {
          new_boundary: new_boundary_sorted,
          new_vertices: new_vertices_sorted,
        }
      })
    })
    .collect::<Vec<_>>();
  nodes.sort_by_key(|(node_ref, _)| node_ref_to_num(node_ref, island_order));
  nodes
}

#[test]
fn modifies_node_boundaries_for_linked_islands() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 1.0),
        Vec3::new(2.0, 2.0, 1.0),
        Vec3::new(1.0, 2.0, 1.0),
        Vec3::new(2.0, 3.0, 1.0),
        Vec3::new(1.0, 3.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![3, 2, 4, 5]],
      polygon_type_indices: vec![0, 0],
    }
    .validate()
    .expect("is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_1_id = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();
  let island_2_id = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::new(1.0, -1.0, 0.0), rotation: 0.0 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();
  let island_3_id = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::new(2.0, 3.5, 0.0), rotation: PI * -0.5 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();

  nav_data.update(/* edge_link_distance= */ 1e-6);

  let expected_modified_nodes = [
    (
      NodeRef { island_id: island_1_id, polygon_index: 0 },
      ModifiedNode { new_boundary: vec![(0, 1), (3, 0)], new_vertices: vec![] },
    ),
    (
      NodeRef { island_id: island_2_id, polygon_index: 1 },
      ModifiedNode {
        new_boundary: vec![(2, 6), (4, 5)],
        new_vertices: vec![Vec2::new(3.0, 1.5)],
      },
    ),
    (
      NodeRef { island_id: island_3_id, polygon_index: 0 },
      ModifiedNode {
        new_boundary: vec![(0, 6), (1, 2), (3, 0)],
        new_vertices: vec![Vec2::new(3.0, 2.0)],
      },
    ),
  ];

  assert_eq!(
    clone_sort_round_modified_nodes(
      &nav_data.modified_nodes,
      &HashMap::from([(island_1_id, 6), (island_2_id, 6), (island_3_id, 6)]),
      &[island_1_id, island_2_id, island_3_id],
      1e-4
    ),
    &expected_modified_nodes
  );
}

#[test]
fn stale_modified_nodes_are_removed() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 1.0),
        Vec3::new(2.0, 2.0, 1.0),
        Vec3::new(1.0, 2.0, 1.0),
        Vec3::new(2.0, 3.0, 1.0),
        Vec3::new(1.0, 3.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![3, 2, 4, 5]],
      polygon_type_indices: vec![0, 0],
    }
    .validate()
    .expect("is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  nav_data.add_island().set_nav_mesh(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::clone(&nav_mesh),
    HashMap::new(),
  );
  let island_2_id = nav_data
    .add_island()
    .set_nav_mesh(
      Transform { translation: Vec3::new(1.0, -1.0, 0.0), rotation: 0.0 },
      Arc::clone(&nav_mesh),
      HashMap::new(),
    )
    .id();

  nav_data.update(/* edge_link_distance= */ 1e-6);

  assert_eq!(nav_data.modified_nodes.len(), 2);

  nav_data.remove_island(island_2_id);

  nav_data.update(/* edge_link_distance= */ 1e-6);

  assert_eq!(nav_data.modified_nodes.len(), 0);
}

#[test]
fn empty_navigation_mesh_is_safe() {
  let full_nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let empty_nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![],
      polygons: vec![],
      polygon_type_indices: vec![],
    }
    .validate()
    .expect("An empty nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();
  nav_data.add_island().set_nav_mesh(
    Transform::default(),
    full_nav_mesh,
    HashMap::new(),
  );
  nav_data.add_island().set_nav_mesh(
    Transform::default(),
    empty_nav_mesh,
    HashMap::new(),
  );

  // Nothing should panic here.
  nav_data.update(/* edge_link_distance= */ 1e-6);
}

#[test]
fn cannot_remove_used_node_type() {
  let mut archipelago = Archipelago::<XY>::new();

  let node_type_1 = archipelago.create_node_type(2.0);
  let node_type_2 = archipelago.create_node_type(3.0);

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("mesh is valid"),
  );

  let island_id_1 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform::default(),
      nav_mesh.clone(),
      HashMap::from([(0, node_type_1)]),
    )
    .id();

  let island_id_2 = archipelago
    .add_island()
    .set_nav_mesh(
      Transform::default(),
      nav_mesh.clone(),
      HashMap::from([(0, node_type_1)]),
    )
    .id();

  // Another island that has no effect since it doesn't mention `node_type_1`.
  archipelago.add_island().set_nav_mesh(
    Transform::default(),
    nav_mesh.clone(),
    HashMap::from([(0, node_type_2)]),
  );

  assert_eq!(
    archipelago.nav_data.get_node_types().collect::<Vec<_>>(),
    [(node_type_1, 2.0), (node_type_2, 3.0)]
  );

  // Two islands still reference `node_type_1`.
  assert!(!archipelago.remove_node_type(node_type_1));
  assert_eq!(
    archipelago.nav_data.get_node_types().collect::<Vec<_>>(),
    [(node_type_1, 2.0), (node_type_2, 3.0)]
  );

  archipelago.remove_island(island_id_1);
  // One island still references `node_type_1`.
  assert!(!archipelago.remove_node_type(node_type_1));
  assert_eq!(
    archipelago.nav_data.get_node_types().collect::<Vec<_>>(),
    [(node_type_1, 2.0), (node_type_2, 3.0)]
  );

  archipelago.remove_island(island_id_2);
  // Now we can delete it!
  assert!(archipelago.remove_node_type(node_type_1));

  // We can't delete it twice.
  assert!(!archipelago.remove_node_type(node_type_1));

  assert_eq!(
    archipelago.nav_data.get_node_types().collect::<Vec<_>>(),
    [(node_type_2, 3.0)]
  );
}

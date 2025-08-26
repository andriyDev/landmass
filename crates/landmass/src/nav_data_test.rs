use std::{
  cmp::Ordering,
  collections::{HashMap, HashSet},
  f32::consts::PI,
  sync::Arc,
};

use glam::{Vec2, Vec3};
use googletest::{
  expect_eq, expect_false, expect_that, expect_true, matchers::*,
  prelude::container_eq,
};
use slotmap::{HopSlotMap, SlotMap};

use crate::{
  AgentOptions, Archipelago, CoordinateSystem, FromAgentRadius,
  HeightNavigationMesh, HeightPolygon, IslandId, PointSampleDistance3d,
  SetTypeIndexCostError, Transform,
  coords::{CorePointSampleDistance, XY, XYZ},
  island::Island,
  link::{AnimationLink, NodePortal},
  nav_data::{KindedOffMeshLink, NodeRef, OffMeshLink},
  nav_mesh::NavigationMesh,
};

use super::{
  ModifiedNode, NavigationData, OffMeshLinkId, island_edges_bbh,
  link_edges_between_islands,
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
    height_mesh: None,
  }
  .validate()
  .expect("is valid");
  let nav_mesh = Arc::new(nav_mesh);

  let mut nav_data = NavigationData::<XYZ>::new();
  let island_id_1 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::clone(&nav_mesh),
  ));
  let island_id_2 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(5.0, 0.0, 0.1), rotation: PI * 0.5 },
    Arc::clone(&nav_mesh),
  ));

  // Just above island 1 node.
  assert_eq!(
    nav_data.sample_point(
      Vec3::new(1.5, 1.5, 1.09),
      &CorePointSampleDistance::new(&PointSampleDistance3d {
        horizontal_distance: 0.1,
        distance_below: 0.1,
        distance_above: 0.1,
        vertical_preference_ratio: 1.0,
      }),
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
      &CorePointSampleDistance::new(&PointSampleDistance3d {
        horizontal_distance: 0.1,
        distance_below: 0.1,
        distance_above: 0.1,
        vertical_preference_ratio: 1.0,
      }),
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
      &CorePointSampleDistance::new(&PointSampleDistance3d {
        horizontal_distance: 0.1,
        distance_below: 0.1,
        distance_above: 0.1,
        vertical_preference_ratio: 1.0,
      }),
    ),
    Some((
      Vec3::new(3.5, 1.5, 1.0),
      NodeRef { island_id: island_id_1, polygon_index: 2 }
    )),
  );
  // At overlap, but closer to island 2.
  assert_eq!(
    nav_data
      .sample_point(
        Vec3::new(3.5, 1.5, 1.06),
        &CorePointSampleDistance::new(&PointSampleDistance3d {
          horizontal_distance: 0.1,
          distance_below: 0.1,
          distance_above: 0.1,
          vertical_preference_ratio: 1.0,
        }),
      )
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
  boundary_links: &SlotMap<OffMeshLinkId, OffMeshLink>,
  node_to_boundary_link_ids: &HashMap<NodeRef, HashSet<OffMeshLinkId>>,
  island_order: &[IslandId],
  round_amount: f32,
) -> Vec<(NodeRef, Vec<OffMeshLink>)> {
  let mut links = node_to_boundary_link_ids
    .iter()
    .map(|(key, value)| {
      (*key, {
        let mut v = value
          .iter()
          .map(|link_id| boundary_links.get(*link_id).unwrap())
          .map(|link| OffMeshLink {
            destination_node: link.destination_node,
            destination_type_index: link.destination_type_index,
            portal: (
              (link.portal.0 / round_amount).round() * round_amount,
              (link.portal.1 / round_amount).round() * round_amount,
            ),
            // We can't store the right link IDs anyway, so just make it
            // default.
            kinded: KindedOffMeshLink::BoundaryLink {
              reverse_link: OffMeshLinkId::default(),
            },
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

#[googletest::test]
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
      polygon_type_indices: vec![0, 1, 1, 1, 0, 0],
      height_mesh: None,
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
      polygon_type_indices: vec![0, 0, 0, 0, 2],
      height_mesh: None,
    }
    .validate()
    .expect("is valid."),
  );

  // Create unused slotmaps just to get `IslandId`s and `NodeType`s.
  let mut slotmap = HopSlotMap::<IslandId, _>::with_key();
  let island_1_id = slotmap.insert(0);
  let island_2_id = slotmap.insert(0);

  let transform =
    Transform { translation: Vec3::new(1.0, 2.0, 3.0), rotation: PI * -0.25 };

  let island_1 = Island::new(transform.clone(), Arc::clone(&nav_mesh_1));
  let island_2 = Island::new(transform.clone(), Arc::clone(&nav_mesh_2));

  let island_1_edge_bbh = island_edges_bbh(&island_1);
  let island_2_edge_bbh = island_edges_bbh(&island_2);

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
      (transform.apply(a) * 1e5).round() * 1e-5,
      (transform.apply(b) * 1e5).round() * 1e-5,
    )
  }

  let kinded_default =
    KindedOffMeshLink::BoundaryLink { reverse_link: Default::default() };
  let expected_links = [
    (
      NodeRef { island_id: island_1_id, polygon_index: 0 },
      vec![OffMeshLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 2 },
        destination_type_index: 0,
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(1.0, 0.0, 1.0),
          Vec3::new(1.0, 0.5, 1.0),
        ),
        kinded: kinded_default.clone(),
      }],
    ),
    (
      NodeRef { island_id: island_1_id, polygon_index: 1 },
      vec![OffMeshLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 3 },
        destination_type_index: 0,
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(0.5, 1.0, 1.0),
          Vec3::new(-0.5, 1.0, 1.0),
        ),
        kinded: kinded_default.clone(),
      }],
    ),
    (
      NodeRef { island_id: island_1_id, polygon_index: 2 },
      vec![OffMeshLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 0 },
        destination_type_index: 0,
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(-1.0, 0.5, 1.0),
          Vec3::new(-1.0, 0.0, 1.0),
        ),
        kinded: kinded_default.clone(),
      }],
    ),
    (
      NodeRef { island_id: island_1_id, polygon_index: 3 },
      vec![OffMeshLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 0 },
        destination_type_index: 0,
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(-1.0, 0.0, 1.0),
          Vec3::new(-1.0, -0.5, 1.0),
        ),
        kinded: kinded_default.clone(),
      }],
    ),
    (
      NodeRef { island_id: island_1_id, polygon_index: 4 },
      vec![OffMeshLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 4 },
        destination_type_index: 2,
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(-0.5, -1.0, 1.0),
          Vec3::new(0.5, -1.0, 1.0),
        ),
        kinded: kinded_default.clone(),
      }],
    ),
    (
      NodeRef { island_id: island_1_id, polygon_index: 5 },
      vec![OffMeshLink {
        destination_node: NodeRef { island_id: island_2_id, polygon_index: 2 },
        destination_type_index: 0,
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(1.0, -0.5, 1.0),
          Vec3::new(1.0, 0.0, 1.0),
        ),
        kinded: kinded_default.clone(),
      }],
    ),
    // Reverse links
    (
      NodeRef { island_id: island_2_id, polygon_index: 0 },
      vec![
        OffMeshLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 2,
          },
          destination_type_index: 1,
          portal: transform_and_round_portal(
            &transform,
            Vec3::new(-1.0, 0.0, 1.0),
            Vec3::new(-1.0, 0.5, 1.0),
          ),
          kinded: kinded_default.clone(),
        },
        OffMeshLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 3,
          },
          destination_type_index: 1,
          portal: transform_and_round_portal(
            &transform,
            Vec3::new(-1.0, -0.5, 1.0),
            Vec3::new(-1.0, 0.0, 1.0),
          ),
          kinded: kinded_default.clone(),
        },
      ],
    ),
    (
      NodeRef { island_id: island_2_id, polygon_index: 2 },
      vec![
        OffMeshLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 0,
          },
          destination_type_index: 0,
          portal: transform_and_round_portal(
            &transform,
            Vec3::new(1.0, 0.5, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
          ),
          kinded: kinded_default.clone(),
        },
        OffMeshLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 5,
          },
          destination_type_index: 0,
          portal: transform_and_round_portal(
            &transform,
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, -0.5, 1.0),
          ),
          kinded: kinded_default.clone(),
        },
      ],
    ),
    (
      NodeRef { island_id: island_2_id, polygon_index: 3 },
      vec![OffMeshLink {
        destination_node: NodeRef { island_id: island_1_id, polygon_index: 1 },
        destination_type_index: 1,
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(-0.5, 1.0, 1.0),
          Vec3::new(0.5, 1.0, 1.0),
        ),
        kinded: kinded_default.clone(),
      }],
    ),
    (
      NodeRef { island_id: island_2_id, polygon_index: 4 },
      vec![OffMeshLink {
        destination_node: NodeRef { island_id: island_1_id, polygon_index: 4 },
        destination_type_index: 0,
        portal: transform_and_round_portal(
          &transform,
          Vec3::new(0.5, -1.0, 1.0),
          Vec3::new(-0.5, -1.0, 1.0),
        ),
        kinded: kinded_default.clone(),
      }],
    ),
  ];
  expect_that!(
    clone_sort_round_links(
      &boundary_links,
      &node_to_boundary_link_ids,
      &[island_1_id, island_2_id],
      1e-5
    ),
    container_eq(expected_links.clone())
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
    /* edge_link_distance= */ 1e-3,
    &mut boundary_links,
    &mut node_to_boundary_link_ids,
    &mut modified_node_refs_to_update,
  );
  expect_that!(
    clone_sort_round_links(
      &boundary_links,
      &node_to_boundary_link_ids,
      &[island_1_id, island_2_id],
      1e-5
    ),
    container_eq(expected_links)
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
      height_mesh: None,
    }
    .validate()
    .expect("is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_1_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::clone(&nav_mesh),
  ));
  let island_2_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: PI * 0.5 },
    Arc::clone(&nav_mesh),
  ));
  let island_3_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: PI },
    Arc::clone(&nav_mesh),
  ));
  let island_4_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(3.0, 0.0, 0.0), rotation: PI },
    Arc::clone(&nav_mesh),
  ));
  let island_5_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 3.0, 0.0), rotation: PI * -0.5 },
    Arc::clone(&nav_mesh),
  ));

  nav_data.update(
    /* edge_link_distance= */ 0.01,
    /* animation_link_distance */ 0.01,
  );

  let kinded_default =
    KindedOffMeshLink::BoundaryLink { reverse_link: Default::default() };
  assert_eq!(
    clone_sort_round_links(
      &nav_data.off_mesh_links,
      &nav_data.node_to_off_mesh_link_ids,
      &[island_1_id, island_2_id, island_3_id, island_4_id, island_5_id],
      1e-6
    ),
    [
      (
        NodeRef { island_id: island_1_id, polygon_index: 0 },
        vec![
          OffMeshLink {
            destination_node: NodeRef {
              island_id: island_4_id,
              polygon_index: 0,
            },
            destination_type_index: 0,
            portal: (Vec3::new(2.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 1.0)),
            kinded: kinded_default.clone(),
          },
          OffMeshLink {
            destination_node: NodeRef {
              island_id: island_5_id,
              polygon_index: 0,
            },
            destination_type_index: 0,
            portal: (Vec3::new(2.0, 2.0, 1.0), Vec3::new(2.0, 1.0, 1.0)),
            kinded: kinded_default.clone(),
          },
        ],
      ),
      (
        NodeRef { island_id: island_1_id, polygon_index: 1 },
        vec![OffMeshLink {
          destination_node: NodeRef {
            island_id: island_2_id,
            polygon_index: 0,
          },
          destination_type_index: 0,
          portal: (Vec3::new(0.0, 1.0, 1.0), Vec3::new(0.0, 2.0, 1.0)),
          kinded: kinded_default.clone(),
        }],
      ),
      (
        NodeRef { island_id: island_2_id, polygon_index: 0 },
        vec![OffMeshLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 1,
          },
          destination_type_index: 0,
          portal: (Vec3::new(0.0, 2.0, 1.0), Vec3::new(0.0, 1.0, 1.0)),
          kinded: kinded_default.clone(),
        }],
      ),
      (
        NodeRef { island_id: island_2_id, polygon_index: 1 },
        vec![OffMeshLink {
          destination_node: NodeRef {
            island_id: island_3_id,
            polygon_index: 0,
          },
          destination_type_index: 0,
          portal: (Vec3::new(-1.0, 0.0, 1.0), Vec3::new(-2.0, 0.0, 1.0)),
          kinded: kinded_default.clone(),
        }],
      ),
      (
        NodeRef { island_id: island_3_id, polygon_index: 0 },
        vec![OffMeshLink {
          destination_node: NodeRef {
            island_id: island_2_id,
            polygon_index: 1,
          },
          destination_type_index: 0,
          portal: (Vec3::new(-2.0, 0.0, 1.0), Vec3::new(-1.0, 0.0, 1.0)),
          kinded: kinded_default.clone(),
        }],
      ),
      (
        NodeRef { island_id: island_4_id, polygon_index: 0 },
        vec![OffMeshLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 0,
          },
          destination_type_index: 0,
          portal: (Vec3::new(1.0, 0.0, 1.0), Vec3::new(2.0, 0.0, 1.0)),
          kinded: kinded_default.clone(),
        }],
      ),
      (
        NodeRef { island_id: island_5_id, polygon_index: 0 },
        vec![OffMeshLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 0,
          },
          destination_type_index: 0,
          portal: (Vec3::new(2.0, 1.0, 1.0), Vec3::new(2.0, 2.0, 1.0)),
          kinded: kinded_default.clone(),
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
    .set_transform(Transform { translation: Vec3::ZERO, rotation: PI * 0.5 });

  nav_data.update(
    /* edge_link_distance= */ 0.01,
    /* animation_link_distance */ 0.01,
  );

  assert_eq!(
    clone_sort_round_links(
      &nav_data.off_mesh_links,
      &nav_data.node_to_off_mesh_link_ids,
      &[island_1_id, island_2_id, island_3_id, island_4_id, island_5_id],
      1e-6
    ),
    [
      (
        NodeRef { island_id: island_1_id, polygon_index: 1 },
        vec![OffMeshLink {
          destination_node: NodeRef {
            island_id: island_5_id,
            polygon_index: 0,
          },
          destination_type_index: 0,
          portal: (Vec3::new(0.0, 1.0, 1.0), Vec3::new(0.0, 2.0, 1.0)),
          kinded: kinded_default.clone(),
        }],
      ),
      (
        NodeRef { island_id: island_3_id, polygon_index: 0 },
        vec![OffMeshLink {
          destination_node: NodeRef {
            island_id: island_5_id,
            polygon_index: 1,
          },
          destination_type_index: 0,
          portal: (Vec3::new(-2.0, 0.0, 1.0), Vec3::new(-1.0, 0.0, 1.0)),
          kinded: kinded_default.clone(),
        }],
      ),
      (
        NodeRef { island_id: island_5_id, polygon_index: 0 },
        vec![OffMeshLink {
          destination_node: NodeRef {
            island_id: island_1_id,
            polygon_index: 1,
          },
          destination_type_index: 0,
          portal: (Vec3::new(0.0, 2.0, 1.0), Vec3::new(0.0, 1.0, 1.0)),
          kinded: kinded_default.clone(),
        }],
      ),
      (
        NodeRef { island_id: island_5_id, polygon_index: 1 },
        vec![OffMeshLink {
          destination_node: NodeRef {
            island_id: island_3_id,
            polygon_index: 0,
          },
          destination_type_index: 0,
          portal: (Vec3::new(-1.0, 0.0, 1.0), Vec3::new(-2.0, 0.0, 1.0)),
          kinded: kinded_default.clone(),
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
      height_mesh: None,
    }
    .validate()
    .expect("is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_1_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::clone(&nav_mesh),
  ));
  let island_2_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(1.0, -1.0, 0.0), rotation: 0.0 },
    Arc::clone(&nav_mesh),
  ));
  let island_3_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 3.5, 0.0), rotation: PI * -0.5 },
    Arc::clone(&nav_mesh),
  ));

  nav_data.update(
    /* edge_link_distance= */ 1e-6,
    /* animation_link_distance */ 1e-6,
  );

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
      height_mesh: None,
    }
    .validate()
    .expect("is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  nav_data.add_island(Island::new(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    Arc::clone(&nav_mesh),
  ));
  let island_2_id = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(1.0, -1.0, 0.0), rotation: 0.0 },
    Arc::clone(&nav_mesh),
  ));

  nav_data.update(
    /* edge_link_distance= */ 1e-6,
    /* animation_link_distance */ 1e-6,
  );

  assert_eq!(nav_data.modified_nodes.len(), 2);

  nav_data.remove_island(island_2_id);

  nav_data.update(
    /* edge_link_distance= */ 1e-6,
    /* animation_link_distance */ 1e-6,
  );

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
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let empty_nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![],
      polygons: vec![],
      polygon_type_indices: vec![],
      height_mesh: None,
    }
    .validate()
    .expect("An empty nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();
  nav_data.add_island(Island::new(Transform::default(), full_nav_mesh));
  nav_data.add_island(Island::new(Transform::default(), empty_nav_mesh));

  // Nothing should panic here.
  nav_data.update(
    /* edge_link_distance= */ 1e-6,
    /* animation_link_distance */ 1e-6,
  );
}

#[test]
fn error_on_set_zero_or_negative_type_index_cost() {
  let mut archipelago =
    Archipelago::<XY>::new(AgentOptions::from_agent_radius(0.5));
  assert_eq!(
    archipelago.set_type_index_cost(0, 0.0),
    Err(SetTypeIndexCostError::NonPositiveCost(0.0))
  );
  assert_eq!(
    archipelago.set_type_index_cost(0, -1.0),
    Err(SetTypeIndexCostError::NonPositiveCost(-1.0))
  );
}

#[googletest::test]
fn changed_island_rebuilds_region_connectivity() {
  let mut nav_data = NavigationData::<XY>::new();

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
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let island_1 =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));
  let island_2 = nav_data.add_island(Island::new(
    Transform { translation: Vec2::new(0.0, 2.0), rotation: 0.0 },
    nav_mesh,
  ));
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  expect_false!(nav_data.are_nodes_connected(
    NodeRef { island_id: island_1, polygon_index: 0 },
    NodeRef { island_id: island_2, polygon_index: 0 },
  ));

  // Making the islands touch should result in the regions being connected.
  nav_data.get_island_mut(island_2).unwrap().set_transform(Transform {
    translation: Vec2::new(0.0, 1.0),
    rotation: 0.0,
  });
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  expect_true!(nav_data.are_nodes_connected(
    NodeRef { island_id: island_1, polygon_index: 0 },
    NodeRef { island_id: island_2, polygon_index: 0 },
  ));

  // Making the islands no longer touch again should remove the connectivity.
  nav_data.get_island_mut(island_2).unwrap().set_transform(Transform {
    translation: Vec2::new(0.0, 2.0),
    rotation: 0.0,
  });
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  expect_false!(nav_data.are_nodes_connected(
    NodeRef { island_id: island_1, polygon_index: 0 },
    NodeRef { island_id: island_2, polygon_index: 0 },
  ));
}

fn get_off_mesh_links_for_node<CS: CoordinateSystem>(
  nav_data: &NavigationData<CS>,
  node_ref: NodeRef,
) -> Option<Vec<&OffMeshLink>> {
  Some(
    nav_data
      .node_to_off_mesh_link_ids
      .get(&node_ref)?
      .iter()
      .map(|&link_id| nav_data.off_mesh_links.get(link_id).unwrap())
      .collect::<Vec<_>>(),
  )
}

#[googletest::test]
fn generates_animation_links() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 13.0),
        Vec3::new(1.0, 0.0, 13.0),
        Vec3::new(1.0, 1.0, 13.0),
        Vec3::new(0.0, 1.0, 13.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_1 =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));
  let island_2 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));
  let island_3 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(-2.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));
  let island_4 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(0.0, 4.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));

  let link_id_1 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.1, 0.9, 13.0), Vec3::new(0.9, 0.9, 13.0)),
    end_edge: (Vec3::new(0.1, 4.1, 13.0), Vec3::new(0.9, 4.1, 13.0)),
    cost: 1.0,
    kind: 0,
  });
  let link_id_2 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.9, 0.1, 13.0), Vec3::new(0.9, 0.9, 13.0)),
    end_edge: (Vec3::new(2.1, 0.1, 13.0), Vec3::new(2.1, 0.9, 13.0)),
    cost: 1.0,
    kind: 0,
  });
  let link_id_3 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(-1.1, 0.1, 13.0), Vec3::new(-1.1, 0.9, 13.0)),
    end_edge: (Vec3::new(0.1, 0.1, 13.0), Vec3::new(0.1, 0.9, 13.0)),
    cost: 1.0,
    kind: 0,
  });

  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_4, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(
    link_2.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_2.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  let link_3 = &nav_data.animation_links[link_id_3];
  expect_that!(
    link_3.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_3, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_3.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 2);
  let island_1_off_mesh_links = get_off_mesh_links_for_node(
    &nav_data,
    NodeRef { island_id: island_1, polygon_index: 0 },
  );
  expect_that!(
    island_1_off_mesh_links,
    some(unordered_elements_are!(
      &&OffMeshLink {
        destination_node: NodeRef { island_id: island_4, polygon_index: 0 },
        destination_type_index: 0,
        portal: (Vec3::new(0.1, 0.9, 13.0), Vec3::new(0.9, 0.9, 13.0)),
        kinded: KindedOffMeshLink::AnimationLink {
          destination_portal: (
            Vec3::new(0.1, 4.1, 13.0),
            Vec3::new(0.9, 4.1, 13.0)
          ),
          cost: 1.0,
          animation_link: link_id_1,
        },
      },
      &&OffMeshLink {
        destination_node: NodeRef { island_id: island_2, polygon_index: 0 },
        destination_type_index: 0,
        portal: (Vec3::new(0.9, 0.1, 13.0), Vec3::new(0.9, 0.9, 13.0)),
        kinded: KindedOffMeshLink::AnimationLink {
          destination_portal: (
            Vec3::new(2.1, 0.1, 13.0),
            Vec3::new(2.1, 0.9, 13.0)
          ),
          cost: 1.0,
          animation_link: link_id_2,
        },
      },
    ))
  );
  let island_3_off_mesh_links = get_off_mesh_links_for_node(
    &nav_data,
    NodeRef { island_id: island_3, polygon_index: 0 },
  );
  expect_that!(
    island_3_off_mesh_links,
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_1, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(-1.1, 0.1, 13.0), Vec3::new(-1.1, 0.9, 13.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.1, 0.1, 13.0),
          Vec3::new(0.1, 0.9, 13.0)
        ),
        cost: 1.0,
        animation_link: link_id_3,
      },
    }))
  );
}

#[googletest::test]
fn removing_animation_link_removes_off_mesh_links() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 7.0),
        Vec3::new(1.0, 0.0, 7.0),
        Vec3::new(1.0, 2.0, 7.0),
        Vec3::new(0.0, 2.0, 7.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_1 =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));
  let island_2 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));

  // Prevent the islands from being brand new.
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_id_1 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.9, 0.1, 7.0), Vec3::new(0.9, 0.9, 7.0)),
    end_edge: (Vec3::new(2.1, 0.1, 7.0), Vec3::new(2.1, 0.9, 7.0)),
    cost: 1.0,
    kind: 0,
  });
  let link_id_2 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(2.1, 1.1, 7.0), Vec3::new(2.1, 1.9, 7.0)),
    end_edge: (Vec3::new(0.9, 1.1, 7.0), Vec3::new(0.9, 1.9, 7.0)),
    cost: 1.0,
    kind: 0,
  });

  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(
    link_2.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_2.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 2);
  let island_1_off_mesh_links = get_off_mesh_links_for_node(
    &nav_data,
    NodeRef { island_id: island_1, polygon_index: 0 },
  );
  expect_that!(
    island_1_off_mesh_links,
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_2, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.9, 0.1, 7.0), Vec3::new(0.9, 0.9, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(2.1, 0.1, 7.0),
          Vec3::new(2.1, 0.9, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_1,
      },
    }))
  );
  let island_2_off_mesh_links = get_off_mesh_links_for_node(
    &nav_data,
    NodeRef { island_id: island_2, polygon_index: 0 },
  );
  expect_that!(
    island_2_off_mesh_links,
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_1, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(2.1, 1.1, 7.0), Vec3::new(2.1, 1.9, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.9, 1.1, 7.0),
          Vec3::new(0.9, 1.9, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_2,
      },
    }))
  );

  // Removing the animation link should remove its off mesh links.
  nav_data.remove_animation_link(link_id_1);
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 1);
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_1, polygon_index: 0 },
    ),
    none()
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_2, polygon_index: 0 },
    ),
    some(len(eq(1)))
  );

  nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.9, 0.1, 7.0), Vec3::new(0.9, 0.9, 7.0)),
    end_edge: (Vec3::new(2.1, 0.1, 7.0), Vec3::new(2.1, 0.9, 7.0)),
    cost: 1.0,
    kind: 0,
  });
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 2);
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_1, polygon_index: 0 },
    ),
    some(len(eq(1)))
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_2, polygon_index: 0 },
    ),
    some(len(eq(1)))
  );
}

#[googletest::test]
fn existing_animation_link_is_linked_for_new_island() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 7.0),
        Vec3::new(1.0, 0.0, 7.0),
        Vec3::new(1.0, 2.0, 7.0),
        Vec3::new(0.0, 2.0, 7.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let link_id_1 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.9, 0.1, 7.0), Vec3::new(0.9, 0.9, 7.0)),
    end_edge: (Vec3::new(2.1, 0.1, 7.0), Vec3::new(2.1, 0.9, 7.0)),
    cost: 1.0,
    kind: 0,
  });
  let link_id_2 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(2.1, 1.1, 7.0), Vec3::new(2.1, 1.9, 7.0)),
    end_edge: (Vec3::new(0.9, 1.1, 7.0), Vec3::new(0.9, 1.9, 7.0)),
    cost: 1.0,
    kind: 0,
  });

  // Prevent the animation links from being brand new.
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  // Neither link has any portals yet since there are no islands.
  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(link_1.start_portals, is_empty());
  expect_that!(link_1.end_portals, is_empty());
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(link_2.start_portals, is_empty());
  expect_that!(link_2.end_portals, is_empty());

  expect_that!(nav_data.node_to_off_mesh_link_ids, is_empty());
  expect_that!(nav_data.off_mesh_links, is_empty());

  let island_1 =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  // Now that we have the first island, we have a couple portals, but neither
  // link has both start and end portals, so still no off mesh links.
  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(link_1.end_portals, is_empty());
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(link_2.start_portals, is_empty());
  expect_that!(
    link_2.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  expect_that!(nav_data.node_to_off_mesh_link_ids, is_empty());

  // Adding the second island should now complete the links.
  let island_2 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(
    link_2.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_2.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 2);
  let island_1_off_mesh_links = get_off_mesh_links_for_node(
    &nav_data,
    NodeRef { island_id: island_1, polygon_index: 0 },
  );
  expect_that!(
    island_1_off_mesh_links,
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_2, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.9, 0.1, 7.0), Vec3::new(0.9, 0.9, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(2.1, 0.1, 7.0),
          Vec3::new(2.1, 0.9, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_1,
      },
    }))
  );
  let island_2_off_mesh_links = get_off_mesh_links_for_node(
    &nav_data,
    NodeRef { island_id: island_2, polygon_index: 0 },
  );
  expect_that!(
    island_2_off_mesh_links,
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_1, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(2.1, 1.1, 7.0), Vec3::new(2.1, 1.9, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.9, 1.1, 7.0),
          Vec3::new(0.9, 1.9, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_2,
      },
    }))
  );
}

#[googletest::test]
fn added_island_mixes_new_and_old_portals() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 7.0),
        Vec3::new(1.0, 0.0, 7.0),
        Vec3::new(1.0, 1.0, 7.0),
        Vec3::new(0.0, 1.0, 7.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_00 =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));
  let island_11 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 2.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));

  let link_id_1 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.5, 0.5, 7.0), Vec3::new(2.5, 0.5, 7.0)),
    end_edge: (Vec3::new(0.5, 2.5, 7.0), Vec3::new(2.5, 2.5, 7.0)),
    cost: 1.0,
    kind: 0,
  });
  let link_id_2 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.5, 2.5, 7.0), Vec3::new(2.5, 2.5, 7.0)),
    end_edge: (Vec3::new(0.5, 0.5, 7.0), Vec3::new(2.5, 0.5, 7.0)),
    cost: 1.0,
    kind: 0,
  });

  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_00, polygon_index: 0 },
      interval: (0.0, 0.25),
    })
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_11, polygon_index: 0 },
      interval: (0.75, 1.0),
    })
  );
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(
    link_2.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_11, polygon_index: 0 },
      interval: (0.75, 1.0),
    })
  );
  expect_that!(
    link_2.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_00, polygon_index: 0 },
      interval: (0.0, 0.25),
    })
  );

  expect_that!(nav_data.node_to_off_mesh_link_ids, is_empty());

  // Add in the remaining corners.
  let island_01 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(0.0, 2.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));
  let island_10 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));

  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(
      &NodePortal {
        node: NodeRef { island_id: island_00, polygon_index: 0 },
        interval: (0.0, 0.25),
      },
      &NodePortal {
        node: NodeRef { island_id: island_10, polygon_index: 0 },
        interval: (0.75, 1.0),
      }
    )
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(
      &NodePortal {
        node: NodeRef { island_id: island_01, polygon_index: 0 },
        interval: (0.0, 0.25),
      },
      &NodePortal {
        node: NodeRef { island_id: island_11, polygon_index: 0 },
        interval: (0.75, 1.0),
      }
    )
  );
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(
    link_2.start_portals,
    unordered_elements_are!(
      &NodePortal {
        node: NodeRef { island_id: island_01, polygon_index: 0 },
        interval: (0.0, 0.25),
      },
      &NodePortal {
        node: NodeRef { island_id: island_11, polygon_index: 0 },
        interval: (0.75, 1.0),
      }
    )
  );
  expect_that!(
    link_2.end_portals,
    unordered_elements_are!(
      &NodePortal {
        node: NodeRef { island_id: island_00, polygon_index: 0 },
        interval: (0.0, 0.25),
      },
      &NodePortal {
        node: NodeRef { island_id: island_10, polygon_index: 0 },
        interval: (0.75, 1.0),
      }
    )
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 4);
  expect_eq!(nav_data.off_mesh_links.len(), 4);

  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_00, polygon_index: 0 },
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_01, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.5, 0.5, 7.0), Vec3::new(1.0, 0.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.5, 2.5, 7.0),
          Vec3::new(1.0, 2.5, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_1,
      },
    }))
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_10, polygon_index: 0 },
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_11, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(2.0, 0.5, 7.0), Vec3::new(2.5, 0.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(2.0, 2.5, 7.0),
          Vec3::new(2.5, 2.5, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_1,
      },
    }))
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_01, polygon_index: 0 },
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_00, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.5, 2.5, 7.0), Vec3::new(1.0, 2.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.5, 0.5, 7.0),
          Vec3::new(1.0, 0.5, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_2,
      },
    }))
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_11, polygon_index: 0 },
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_10, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(2.0, 2.5, 7.0), Vec3::new(2.5, 2.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(2.0, 0.5, 7.0),
          Vec3::new(2.5, 0.5, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_2,
      },
    }))
  );
}

#[googletest::test]
fn same_island_animation_link() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 2.0),
        Vec2::new(0.0, 2.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XY>::new();

  let island =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));

  let link_id = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec2::new(0.1, 0.5), Vec2::new(0.9, 0.5)),
    end_edge: (Vec2::new(0.1, 1.5), Vec2::new(0.9, 1.5)),
    cost: 1.0,
    kind: 0,
  });

  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link = &nav_data.animation_links[link_id];
  expect_that!(
    link.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 1);
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island, polygon_index: 0 },
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.1, 0.5, 0.0), Vec3::new(0.9, 0.5, 0.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.1, 1.5, 0.0),
          Vec3::new(0.9, 1.5, 0.0)
        ),
        cost: 1.0,
        animation_link: link_id,
      },
    }))
  );
}

#[googletest::test]
fn point_animation_links_are_connected() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 7.0),
        Vec3::new(1.0, 0.0, 7.0),
        Vec3::new(1.0, 1.0, 7.0),
        Vec3::new(0.0, 1.0, 7.0),
        //
        Vec3::new(0.0, 3.0, 7.0),
        Vec3::new(1.0, 3.0, 7.0),
        Vec3::new(1.0, 4.0, 7.0),
        Vec3::new(0.0, 4.0, 7.0),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![4, 5, 6, 7]],
      polygon_type_indices: vec![0; 2],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));

  // This link has a whole start edge, but the end edge is actually a point. All
  // links here should go to that one point.
  let link_id_1 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.0, 0.5, 7.0), Vec3::new(1.0, 0.5, 7.0)),
    end_edge: (Vec3::new(0.5, 3.5, 7.0), Vec3::new(0.5, 3.5, 7.0)),
    cost: 1.0,
    kind: 0,
  });
  // This link has a whole end edge, but the start edge is actually a point. As
  // a result, the end edge should actually be treated as a point.
  let link_id_2 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.5, 3.5, 7.0), Vec3::new(0.5, 3.5, 7.0)),
    end_edge: (Vec3::new(0.0, 0.5, 7.0), Vec3::new(1.0, 0.5, 7.0)),
    cost: 1.0,
    kind: 0,
  });

  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island, polygon_index: 1 },
      interval: (0.0, 1.0),
    })
  );
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(
    link_2.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island, polygon_index: 1 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_2.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 2);
  expect_eq!(nav_data.off_mesh_links.len(), 2);

  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island, polygon_index: 0 },
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island, polygon_index: 1 },
      destination_type_index: 0,
      portal: (Vec3::new(0.0, 0.5, 7.0), Vec3::new(1.0, 0.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.5, 3.5, 7.0),
          Vec3::new(0.5, 3.5, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_1,
      },
    }))
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island, polygon_index: 1 },
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.5, 3.5, 7.0), Vec3::new(0.5, 3.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.5, 0.5, 7.0),
          Vec3::new(0.5, 0.5, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_2,
      },
    }))
  );
}

#[googletest::test]
fn point_animation_links_are_connected_when_not_new() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 7.0),
        Vec3::new(1.0, 0.0, 7.0),
        Vec3::new(1.0, 1.0, 7.0),
        Vec3::new(0.0, 1.0, 7.0),
        //
        Vec3::new(0.0, 3.0, 7.0),
        Vec3::new(1.0, 3.0, 7.0),
        Vec3::new(1.0, 4.0, 7.0),
        Vec3::new(0.0, 4.0, 7.0),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![4, 5, 6, 7]],
      polygon_type_indices: vec![0; 2],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  // This link has a whole start edge, but the end edge is actually a point. All
  // links here should go to that one point.
  let link_id_1 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.0, 0.5, 7.0), Vec3::new(1.0, 0.5, 7.0)),
    end_edge: (Vec3::new(0.5, 3.5, 7.0), Vec3::new(0.5, 3.5, 7.0)),
    cost: 1.0,
    kind: 0,
  });
  // This link has a whole end edge, but the start edge is actually a point. As
  // a result, the end edge should actually be treated as a point.
  let link_id_2 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.5, 3.5, 7.0), Vec3::new(0.5, 3.5, 7.0)),
    end_edge: (Vec3::new(0.0, 0.5, 7.0), Vec3::new(1.0, 0.5, 7.0)),
    cost: 1.0,
    kind: 0,
  });

  // Make the links not new to see that adding islands underneath also maintains
  // the point links.
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let island =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));

  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island, polygon_index: 1 },
      interval: (0.0, 1.0),
    })
  );
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(
    link_2.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island, polygon_index: 1 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_2.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 2);
  expect_eq!(nav_data.off_mesh_links.len(), 2);

  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island, polygon_index: 0 },
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island, polygon_index: 1 },
      destination_type_index: 0,
      portal: (Vec3::new(0.0, 0.5, 7.0), Vec3::new(1.0, 0.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.5, 3.5, 7.0),
          Vec3::new(0.5, 3.5, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_1,
      },
    }))
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island, polygon_index: 1 },
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.5, 3.5, 7.0), Vec3::new(0.5, 3.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.5, 0.5, 7.0),
          Vec3::new(0.5, 0.5, 7.0)
        ),
        cost: 1.0,
        animation_link: link_id_2,
      },
    }))
  );
}

#[googletest::test]
fn changing_island_does_not_cause_duplicate_off_mesh_links() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 7.0),
        Vec3::new(1.0, 0.0, 7.0),
        Vec3::new(1.0, 1.0, 7.0),
        Vec3::new(0.0, 1.0, 7.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_00 =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));
  let island_10 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(3.5, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));
  let island_11 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(3.0, 3.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));
  let island_01 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(-0.5, 3.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));

  let link_id = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.0, 0.5, 7.0), Vec3::new(4.0, 0.5, 7.0)),
    end_edge: (Vec3::new(0.0, 3.5, 7.0), Vec3::new(4.0, 3.5, 7.0)),
    cost: 1.0,
    kind: 0,
  });

  // Do an initial update so all the links are built.
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link = &nav_data.animation_links[link_id];
  expect_that!(
    link.start_portals,
    unordered_elements_are!(
      &NodePortal {
        node: NodeRef { island_id: island_00, polygon_index: 0 },
        interval: (0.0, 0.25),
      },
      &NodePortal {
        node: NodeRef { island_id: island_10, polygon_index: 0 },
        interval: (0.875, 1.0),
      },
    )
  );
  expect_that!(
    link.end_portals,
    unordered_elements_are!(
      &NodePortal {
        node: NodeRef { island_id: island_01, polygon_index: 0 },
        interval: (0.0, 0.125),
      },
      &NodePortal {
        node: NodeRef { island_id: island_11, polygon_index: 0 },
        interval: (0.75, 1.0),
      },
    )
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 2);
  expect_eq!(nav_data.off_mesh_links.len(), 2);

  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_00, polygon_index: 0 }
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_01, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.0, 0.5, 7.0), Vec3::new(0.5, 0.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.0, 3.5, 7.0),
          Vec3::new(0.5, 3.5, 7.0),
        ),
        cost: 1.0,
        animation_link: link_id,
      },
    }))
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_10, polygon_index: 0 }
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_11, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(3.5, 0.5, 7.0), Vec3::new(4.0, 0.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(3.5, 3.5, 7.0),
          Vec3::new(4.0, 3.5, 7.0),
        ),
        cost: 1.0,
        animation_link: link_id,
      },
    }))
  );

  // Now we change two islands to see that the links are updated.
  nav_data.get_island_mut(island_01).unwrap().set_transform(Transform {
    translation: Vec3::new(0.0, 3.0, 0.0),
    rotation: 0.0,
  });
  nav_data.get_island_mut(island_10).unwrap().set_transform(Transform {
    translation: Vec3::new(3.0, 0.0, 0.0),
    rotation: 0.0,
  });
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link = &nav_data.animation_links[link_id];
  expect_that!(
    link.start_portals,
    unordered_elements_are!(
      &NodePortal {
        node: NodeRef { island_id: island_00, polygon_index: 0 },
        interval: (0.0, 0.25),
      },
      &NodePortal {
        node: NodeRef { island_id: island_10, polygon_index: 0 },
        interval: (0.75, 1.0),
      },
    )
  );
  expect_that!(
    link.end_portals,
    unordered_elements_are!(
      &NodePortal {
        node: NodeRef { island_id: island_01, polygon_index: 0 },
        interval: (0.0, 0.25),
      },
      &NodePortal {
        node: NodeRef { island_id: island_11, polygon_index: 0 },
        interval: (0.75, 1.0),
      },
    )
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 2);
  expect_eq!(nav_data.off_mesh_links.len(), 2);

  // Despite the portals being updated, the off mesh links end up looking the
  // same.
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_00, polygon_index: 0 }
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_01, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.0, 0.5, 7.0), Vec3::new(1.0, 0.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.0, 3.5, 7.0),
          Vec3::new(1.0, 3.5, 7.0),
        ),
        cost: 1.0,
        animation_link: link_id,
      },
    }))
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_10, polygon_index: 0 }
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_11, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(3.0, 0.5, 7.0), Vec3::new(4.0, 0.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(3.0, 3.5, 7.0),
          Vec3::new(4.0, 3.5, 7.0),
        ),
        cost: 1.0,
        animation_link: link_id,
      },
    }))
  );
}

#[googletest::test]
fn changing_island_does_not_cause_duplicate_off_mesh_links_for_point_links() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 7.0),
        Vec3::new(1.0, 0.0, 7.0),
        Vec3::new(1.0, 1.0, 7.0),
        Vec3::new(0.0, 1.0, 7.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_1 =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));
  let island_2 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(0.0, 2.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));

  let link_id_1 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.25, 0.5, 7.0), Vec3::new(0.25, 0.5, 7.0)),
    end_edge: (Vec3::new(0.0, 2.5, 7.0), Vec3::new(0.5, 2.5, 7.0)),
    cost: 1.0,
    kind: 0,
  });
  let link_id_2 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.5, 2.5, 7.0), Vec3::new(1.0, 2.5, 7.0)),
    end_edge: (Vec3::new(0.75, 0.5, 7.0), Vec3::new(0.75, 0.5, 7.0)),
    cost: 1.0,
    kind: 0,
  });

  // Do an initial update so all the links are built.
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(
    link_2.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_2.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 2);
  expect_eq!(nav_data.off_mesh_links.len(), 2);

  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_1, polygon_index: 0 }
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_2, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.25, 0.5, 7.0), Vec3::new(0.25, 0.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.25, 2.5, 7.0),
          Vec3::new(0.25, 2.5, 7.0),
        ),
        cost: 1.0,
        animation_link: link_id_1,
      },
    }))
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_2, polygon_index: 0 }
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_1, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.5, 2.5, 7.0), Vec3::new(1.0, 2.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.75, 0.5, 7.0),
          Vec3::new(0.75, 0.5, 7.0),
        ),
        cost: 1.0,
        animation_link: link_id_2,
      },
    }))
  );

  let old_node_to_off_mesh_link_ids =
    nav_data.node_to_off_mesh_link_ids.clone();

  // Now we change one of the islands to see that the links are updated.
  nav_data.get_island_mut(island_2).unwrap().set_transform(Transform {
    translation: Vec3::new(0.1, 2.0, 0.0),
    rotation: 0.0,
  });
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  // All links should have been changed.
  for (node, links) in nav_data.node_to_off_mesh_link_ids.iter() {
    let old_links = old_node_to_off_mesh_link_ids.get(node).unwrap();
    for old_link in old_links {
      expect_that!(links, not(contains(eq(old_link))));
    }
  }

  // All the links still look the same.
  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(
    link_2.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_2.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 2);
  expect_eq!(nav_data.off_mesh_links.len(), 2);

  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_1, polygon_index: 0 }
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_2, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.25, 0.5, 7.0), Vec3::new(0.25, 0.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.25, 2.5, 7.0),
          Vec3::new(0.25, 2.5, 7.0),
        ),
        cost: 1.0,
        animation_link: link_id_1,
      },
    }))
  );
  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_2, polygon_index: 0 }
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_1, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.5, 2.5, 7.0), Vec3::new(1.0, 2.5, 7.0)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(0.75, 0.5, 7.0),
          Vec3::new(0.75, 0.5, 7.0),
        ),
        cost: 1.0,
        animation_link: link_id_2,
      },
    }))
  );
}

#[googletest::test]
fn generates_animation_links_at_correct_height() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 5.0),
        Vec3::new(1.0, 0.0, 5.0),
        Vec3::new(1.0, 1.0, 5.0),
        Vec3::new(0.0, 1.0, 5.0),
        Vec3::new(0.0, 0.0, 10.0),
        Vec3::new(1.0, 0.0, 10.0),
        Vec3::new(1.0, 1.0, 10.0),
        Vec3::new(0.0, 1.0, 10.0),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![4, 5, 6, 7]],
      polygon_type_indices: vec![0; 2],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_1 =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));
  let island_2 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));

  // This link has both edges just inside the vertical limit of the animation
  // links.
  let link_id_1 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.9, 0.1, 4.1), Vec3::new(0.9, 0.9, 4.1)),
    end_edge: (Vec3::new(2.1, 0.1, 10.9), Vec3::new(2.1, 0.9, 10.9)),
    cost: 1.0,
    kind: 0,
  });
  // This link has both edges just outside the vertical limit of the
  // animation links.
  let link_id_2 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.9, 0.1, 3.9), Vec3::new(0.9, 0.9, 3.9)),
    end_edge: (Vec3::new(2.1, 0.1, 11.1), Vec3::new(2.1, 0.9, 11.1)),
    cost: 1.0,
    kind: 0,
  });

  // Use an animation link distance of 1.0.
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 1 },
      interval: (0.0, 1.0),
    })
  );
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(link_2.start_portals, len(eq(0)));
  expect_that!(link_2.end_portals, len(eq(0)));

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 1);
  expect_eq!(nav_data.off_mesh_links.len(), 1);

  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_1, polygon_index: 0 }
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_2, polygon_index: 1 },
      destination_type_index: 0,
      portal: (Vec3::new(0.9, 0.1, 4.1), Vec3::new(0.9, 0.9, 4.1)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(2.1, 0.1, 10.9),
          Vec3::new(2.1, 0.9, 10.9),
        ),
        cost: 1.0,
        animation_link: link_id_1,
      },
    }))
  );
}

#[googletest::test]
fn generates_animation_links_using_height_mesh() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        // Both polygons are very far away.
        Vec3::new(0.0, 0.0, -100.0),
        Vec3::new(1.0, 0.0, -100.0),
        Vec3::new(1.0, 1.0, -100.0),
        Vec3::new(0.0, 1.0, -100.0),
        Vec3::new(0.0, 0.0, 100.0),
        Vec3::new(1.0, 0.0, 100.0),
        Vec3::new(1.0, 1.0, 100.0),
        Vec3::new(0.0, 1.0, 100.0),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![4, 5, 6, 7]],
      polygon_type_indices: vec![0; 2],
      height_mesh: Some(HeightNavigationMesh {
        vertices: vec![
          // The height polygons have more reasonable heights.
          Vec3::new(0.0, 0.0, 5.0),
          Vec3::new(1.0, 0.0, 5.0),
          Vec3::new(1.0, 1.0, 5.0),
          Vec3::new(0.0, 1.0, 5.0),
          Vec3::new(0.0, 0.0, 10.0),
          Vec3::new(1.0, 0.0, 10.0),
          Vec3::new(1.0, 1.0, 10.0),
          Vec3::new(0.0, 1.0, 10.0),
        ],
        triangles: vec![[0, 1, 2], [2, 3, 0], [0, 1, 2], [2, 3, 0]],
        polygons: vec![
          HeightPolygon {
            base_vertex_index: 0,
            vertex_count: 4,
            base_triangle_index: 0,
            triangle_count: 2,
          },
          HeightPolygon {
            base_vertex_index: 4,
            vertex_count: 4,
            base_triangle_index: 2,
            triangle_count: 2,
          },
        ],
      }),
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_1 =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));
  let island_2 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));

  // This link has both edges just inside the vertical limit of the animation
  // links.
  let link_id_1 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.9, 0.1, 4.1), Vec3::new(0.9, 0.9, 4.1)),
    end_edge: (Vec3::new(2.1, 0.1, 10.9), Vec3::new(2.1, 0.9, 10.9)),
    cost: 1.0,
    kind: 0,
  });
  // This link has both edges just outside the vertical limit of the
  // animation links.
  let link_id_2 = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.9, 0.1, 3.9), Vec3::new(0.9, 0.9, 3.9)),
    end_edge: (Vec3::new(2.1, 0.1, 11.1), Vec3::new(2.1, 0.9, 11.1)),
    cost: 1.0,
    kind: 0,
  });

  // Use an animation link distance of 1.0.
  nav_data.update(
    /* edge_link_distance= */ 1e-5, /* animation_link_distance */ 1.0,
  );

  let link_1 = &nav_data.animation_links[link_id_1];
  expect_that!(
    link_1.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 0 },
      interval: (0.0, 1.0),
    })
  );
  expect_that!(
    link_1.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 1 },
      interval: (0.0, 1.0),
    })
  );
  let link_2 = &nav_data.animation_links[link_id_2];
  expect_that!(link_2.start_portals, len(eq(0)));
  expect_that!(link_2.end_portals, len(eq(0)));

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 1);
  expect_eq!(nav_data.off_mesh_links.len(), 1);

  expect_that!(
    get_off_mesh_links_for_node(
      &nav_data,
      NodeRef { island_id: island_1, polygon_index: 0 }
    ),
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_2, polygon_index: 1 },
      destination_type_index: 0,
      portal: (Vec3::new(0.9, 0.1, 4.1), Vec3::new(0.9, 0.9, 4.1)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(2.1, 0.1, 10.9),
          Vec3::new(2.1, 0.9, 10.9),
        ),
        cost: 1.0,
        animation_link: link_id_1,
      },
    }))
  );
}

#[googletest::test]
fn animation_link_along_angled_surface() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 2.0, 1.0),
        Vec3::new(0.0, 2.0, 1.0),
        Vec3::new(1.0, 3.0, 1.0),
        Vec3::new(0.0, 3.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![3, 2, 4, 5], vec![5, 4, 6, 7]],
      polygon_type_indices: vec![0; 3],
      height_mesh: None,
    }
    .validate()
    .expect("A square nav mesh is valid."),
  );

  let mut nav_data = NavigationData::<XYZ>::new();

  let island_1 =
    nav_data.add_island(Island::new(Transform::default(), nav_mesh.clone()));
  let island_2 = nav_data.add_island(Island::new(
    Transform { translation: Vec3::new(2.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  ));

  let link_id = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.5, 0.5, 0.5), Vec3::new(0.5, 2.5, 0.5)),
    end_edge: (Vec3::new(2.5, 0.5, 0.5), Vec3::new(2.5, 2.5, 0.5)),
    kind: 0,
    cost: 1.0,
  });

  nav_data.update(0.01, 0.4);

  let link = &nav_data.animation_links[link_id];
  expect_that!(
    link.start_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_1, polygon_index: 1 },
      interval: (0.3, 0.7),
    })
  );
  expect_that!(
    link.end_portals,
    unordered_elements_are!(&NodePortal {
      node: NodeRef { island_id: island_2, polygon_index: 1 },
      interval: (0.3, 0.7),
    })
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 1);
  let island_1_node_1_off_mesh_links = get_off_mesh_links_for_node(
    &nav_data,
    NodeRef { island_id: island_1, polygon_index: 1 },
  );
  expect_that!(
    island_1_node_1_off_mesh_links,
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_2, polygon_index: 1 },
      destination_type_index: 0,
      portal: (Vec3::new(0.5, 1.1, 0.5), Vec3::new(0.5, 1.9, 0.5)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(2.5, 1.1, 0.5),
          Vec3::new(2.5, 1.9, 0.5)
        ),
        cost: 1.0,
        animation_link: link_id,
      },
    }))
  );

  nav_data.remove_animation_link(link_id);

  // Readd the same link, but now we will use a larger vertical distance.
  let link_id = nav_data.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.5, 0.5, 0.5), Vec3::new(0.5, 2.5, 0.5)),
    end_edge: (Vec3::new(2.5, 0.5, 0.5), Vec3::new(2.5, 2.5, 0.5)),
    kind: 0,
    cost: 1.0,
  });

  nav_data.update(0.01, 0.6);

  let link = &nav_data.animation_links[link_id];
  expect_that!(
    link.start_portals,
    unordered_elements_are!(
      &NodePortal {
        node: NodeRef { island_id: island_1, polygon_index: 0 },
        interval: (0.0, 0.25),
      },
      &NodePortal {
        node: NodeRef { island_id: island_1, polygon_index: 1 },
        interval: (0.25, 0.75),
      },
      &NodePortal {
        node: NodeRef { island_id: island_1, polygon_index: 2 },
        interval: (0.75, 1.0),
      }
    )
  );
  expect_that!(
    link.end_portals,
    unordered_elements_are!(
      &NodePortal {
        node: NodeRef { island_id: island_2, polygon_index: 0 },
        interval: (0.0, 0.25),
      },
      &NodePortal {
        node: NodeRef { island_id: island_2, polygon_index: 1 },
        interval: (0.25, 0.75),
      },
      &NodePortal {
        node: NodeRef { island_id: island_2, polygon_index: 2 },
        interval: (0.75, 1.0),
      }
    )
  );

  expect_eq!(nav_data.node_to_off_mesh_link_ids.len(), 3);
  let island_1_node_0_off_mesh_links = get_off_mesh_links_for_node(
    &nav_data,
    NodeRef { island_id: island_1, polygon_index: 0 },
  );
  expect_that!(
    island_1_node_0_off_mesh_links,
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_2, polygon_index: 0 },
      destination_type_index: 0,
      portal: (Vec3::new(0.5, 0.5, 0.5), Vec3::new(0.5, 1.0, 0.5)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(2.5, 0.5, 0.5),
          Vec3::new(2.5, 1.0, 0.5)
        ),
        cost: 1.0,
        animation_link: link_id,
      },
    }))
  );
  let island_1_node_1_off_mesh_links = get_off_mesh_links_for_node(
    &nav_data,
    NodeRef { island_id: island_1, polygon_index: 1 },
  );
  expect_that!(
    island_1_node_1_off_mesh_links,
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_2, polygon_index: 1 },
      destination_type_index: 0,
      portal: (Vec3::new(0.5, 1.0, 0.5), Vec3::new(0.5, 2.0, 0.5)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(2.5, 1.0, 0.5),
          Vec3::new(2.5, 2.0, 0.5)
        ),
        cost: 1.0,
        animation_link: link_id,
      },
    }))
  );
  let island_1_node_2_off_mesh_links = get_off_mesh_links_for_node(
    &nav_data,
    NodeRef { island_id: island_1, polygon_index: 2 },
  );
  expect_that!(
    island_1_node_2_off_mesh_links,
    some(unordered_elements_are!(&&OffMeshLink {
      destination_node: NodeRef { island_id: island_2, polygon_index: 2 },
      destination_type_index: 0,
      portal: (Vec3::new(0.5, 2.0, 0.5), Vec3::new(0.5, 2.5, 0.5)),
      kinded: KindedOffMeshLink::AnimationLink {
        destination_portal: (
          Vec3::new(2.5, 2.0, 0.5),
          Vec3::new(2.5, 2.5, 0.5)
        ),
        cost: 1.0,
        animation_link: link_id,
      },
    }))
  );
}

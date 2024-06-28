use std::{f32::consts::PI, sync::Arc};

use glam::Vec3;

use crate::{
  path::{IslandSegment, Path, PathIndex},
  Agent, Archipelago, NavigationMesh, TargetReachedCondition, Transform,
};

#[test]
fn has_reached_target_at_end_node() {
  let nav_mesh =
    NavigationMesh { mesh_bounds: None, vertices: vec![], polygons: vec![] }
      .validate()
      .expect("nav mesh is valid");
  let transform =
    Transform { translation: Vec3::new(2.0, 3.0, 4.0), rotation: PI * 0.85 };
  let mut archipelago = Archipelago::new();
  let island_id = archipelago.add_island();
  archipelago
    .get_island_mut(island_id)
    .set_nav_mesh(transform, Arc::new(nav_mesh));
  let mut agent = Agent::create(
    /* position= */ transform.apply(Vec3::new(1.0, 0.0, 1.0)),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* max_velocity= */ 0.0,
  );

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0],
      portal_edge_index: vec![],
    }],
    boundary_link_segments: vec![],
  };

  let first_path_index = PathIndex::from_corridor_index(0, 0);
  for condition in [
    TargetReachedCondition::Distance(Some(2.0)),
    TargetReachedCondition::StraightPathDistance(Some(2.0)),
    TargetReachedCondition::VisibleAtDistance(Some(2.0)),
  ] {
    agent.target_reached_condition = condition;

    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (first_path_index, transform.apply(Vec3::new(2.5, 0.0, 1.0))),
      (first_path_index, transform.apply(Vec3::new(2.5, 0.0, 1.0))),
    ));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (first_path_index, transform.apply(Vec3::new(3.5, 0.0, 1.0))),
      (first_path_index, transform.apply(Vec3::new(3.5, 0.0, 1.0))),
    ));
    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (first_path_index, transform.apply(Vec3::new(2.0, 0.0, 2.0))),
      (first_path_index, transform.apply(Vec3::new(2.0, 0.0, 2.0))),
    ));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (first_path_index, transform.apply(Vec3::new(2.5, 0.0, 2.5))),
      (first_path_index, transform.apply(Vec3::new(2.5, 0.0, 2.5))),
    ));
  }
}

#[test]
fn long_detour_reaches_target_in_different_ways() {
  let nav_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(1.0, 0.0, 11.0),
      Vec3::new(0.0, 0.0, 12.0),
      Vec3::new(2.0, 0.0, 11.0),
      Vec3::new(3.0, 0.0, 12.0),
      Vec3::new(2.0, 0.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 2], vec![2, 1, 3, 4], vec![4, 3, 5]],
  }
  .validate()
  .expect("nav mesh is valid");

  let transform =
    Transform { translation: Vec3::new(2.0, 3.0, 4.0), rotation: PI * 0.85 };
  let mut archipelago = Archipelago::new();
  let island_id = archipelago.add_island();
  archipelago
    .get_island_mut(island_id)
    .set_nav_mesh(transform, Arc::new(nav_mesh));

  let mut agent = Agent::create(
    /* position= */ Vec3::ZERO,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 0.0,
    /* max_velocity= */ 0.0,
  );

  let path = Path {
    island_segments: vec![IslandSegment {
      island_id,
      corridor: vec![0, 1, 2],
      portal_edge_index: vec![1, 2],
    }],
    boundary_link_segments: vec![],
  };

  {
    agent.position = transform.apply(Vec3::new(1.0, 0.0, 1.0));
    agent.target_reached_condition =
      TargetReachedCondition::Distance(Some(1.1));

    // Agent started within 1.1 units of the destination, so they are close
    // enough.
    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 0.0, 11.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
    ));

    // Agent is just outside of 1.1 units, so they still have not reached the
    // end.
    agent.position = transform.apply(Vec3::new(1.0, 0.0, 2.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 0.0, 11.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
    ));
  }

  {
    agent.target_reached_condition =
      TargetReachedCondition::VisibleAtDistance(Some(15.0));

    // The agent cannot see the target and its path is still too long.
    agent.position = transform.apply(Vec3::new(1.0, 0.0, 1.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 0.0, 11.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
    ));

    // The agent only has 12 units left to travel to the target, and yet the
    // agent still hasn't reached the target, since the target is not visible.
    agent.position = transform.apply(Vec3::new(1.0, 0.0, 10.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 0.0, 11.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
    ));

    // The agent has now "rounded the corner", and so can see the target (and
    // is within the correct distance).
    agent.position = transform.apply(Vec3::new(2.0, 0.0, 11.0));
    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
    ));

    // The agent can see the target but is still too far away.
    agent.position = transform.apply(Vec3::new(2.0, 0.0, 20.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
    ));
  }

  {
    agent.target_reached_condition =
      TargetReachedCondition::StraightPathDistance(Some(15.0));

    // The agent's path is too long (21 units).
    agent.position = transform.apply(Vec3::new(1.0, 0.0, 1.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 0.0, 11.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
    ));

    // The agent only has 12 units left to travel to the target, so they have
    // reached the target.
    agent.position = transform.apply(Vec3::new(1.0, 0.0, 10.0));
    assert!(agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (
        PathIndex::from_corridor_index(0, 1),
        transform.apply(Vec3::new(1.0, 0.0, 11.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
    ));

    // The agent can see the target but is still too far away.
    agent.position = transform.apply(Vec3::new(2.0, 0.0, 20.0));
    assert!(!agent.has_reached_target(
      &path,
      &archipelago.nav_data,
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
      (
        PathIndex::from_corridor_index(0, 2),
        transform.apply(Vec3::new(2.0, 0.0, 1.0))
      ),
    ));
  }
}

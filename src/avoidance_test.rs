use std::{collections::HashMap, sync::Arc};

use glam::Vec3;

use crate::{
  avoidance::apply_avoidance_to_agents, island::Island, nav_data::NodeRef,
  Agent, AgentId, AgentOptions, IslandId, NavigationData, NavigationMesh,
  Transform,
};

use super::nav_mesh_borders_to_dodgy_obstacles;

fn obstacle_matches(
  left: &dodgy_2d::Obstacle,
  right: &dodgy_2d::Obstacle,
) -> bool {
  match (left, right) {
    (
      dodgy_2d::Obstacle::Closed { vertices: left_vertices },
      dodgy_2d::Obstacle::Closed { vertices: right_vertices },
    ) => {
      for left_offset in 0..left_vertices.len() {
        if left_vertices[left_offset..]
          .iter()
          .cloned()
          .chain(left_vertices[..left_offset].iter().cloned())
          .collect::<Vec<_>>()
          == *right_vertices
        {
          return true;
        }
      }
      false
    }
    (
      dodgy_2d::Obstacle::Open { vertices: left_vertices },
      dodgy_2d::Obstacle::Open { vertices: right_vertices },
    ) => left_vertices == right_vertices,
    _ => false,
  }
}

macro_rules! assert_obstacles_match {
  ($left: expr, $right: expr) => {{
    let left = $left;
    let mut right = $right;

    'outer: for (left_index, left_obstacle) in left.iter().enumerate() {
      for (right_index, right_obstacle) in right.iter().enumerate() {
        if obstacle_matches(left_obstacle, right_obstacle) {
          right.remove(right_index);
          continue 'outer;
        }
      }
      panic!("Failed to match left obstacle: index={} obstacle={:?} left_obstacles={:?} remaining_obstacles={:?}",
        left_index, left_obstacle, left, right);
    }

    if !right.is_empty() {
      panic!("Failed to match right obstacles: left_obstacles={:?} remaining_obstacles={:?}", left, right);
    }
  }};
}

#[test]
fn computes_obstacle_for_box() {
  let nav_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(2.0, 0.0, 1.0),
      Vec3::new(2.0, 0.0, 2.0),
      Vec3::new(1.0, 0.0, 2.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
  }
  .validate()
  .expect("Validation succeeds");

  let mut nav_data = NavigationData::new();

  let island_offset = Vec3::new(130.0, 20.0, -50.0);
  let island_offset_dodgy =
    dodgy_2d::Vec2::new(island_offset.x, island_offset.z);

  let island_id = IslandId(1);

  nav_data.islands.insert(island_id, {
    let mut island = Island::new();
    island.set_nav_mesh(
      Transform { translation: island_offset, rotation: 0.0 },
      Arc::new(nav_mesh),
    );
    island
  });

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (
        Vec3::new(1.5, 0.0, 1.5) + island_offset,
        NodeRef { island_id, polygon_index: 0 }
      ),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![dodgy_2d::Obstacle::Closed {
      vertices: vec![
        dodgy_2d::Vec2::new(1.0, 1.0) + island_offset_dodgy,
        dodgy_2d::Vec2::new(1.0, 2.0) + island_offset_dodgy,
        dodgy_2d::Vec2::new(2.0, 2.0) + island_offset_dodgy,
        dodgy_2d::Vec2::new(2.0, 1.0) + island_offset_dodgy
      ]
    }]
  );
}

#[test]
fn dead_end_makes_open_obstacle() {
  let nav_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(2.0, 0.0, 1.0),
      Vec3::new(2.0, 0.0, 2.0),
      Vec3::new(1.0, 0.0, 2.0),
      Vec3::new(3.0, 0.0, 1.0),
      Vec3::new(3.0, 0.0, 2.0),
      Vec3::new(4.0, 0.0, 1.0),
      Vec3::new(4.0, 0.0, 2.0),
      Vec3::new(4.0, 0.0, 3.0),
      Vec3::new(3.0, 0.0, 3.0),
      Vec3::new(4.0, 0.0, 4.0),
      Vec3::new(3.0, 0.0, 4.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3],
      vec![2, 1, 4, 5],
      vec![5, 4, 6, 7],
      vec![5, 7, 8, 9],
      vec![9, 8, 10, 11],
    ],
  }
  .validate()
  .expect("Validation succeeds");

  let island_id = IslandId(1);

  let mut nav_data = NavigationData::new();
  nav_data.islands.insert(island_id, {
    let mut island = Island::new();
    island.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::new(nav_mesh),
    );
    island
  });

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (Vec3::new(1.5, 0.0, 1.5), NodeRef { island_id, polygon_index: 0 }),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![dodgy_2d::Obstacle::Open {
      vertices: vec![
        dodgy_2d::Vec2::new(4.0, 3.0),
        dodgy_2d::Vec2::new(4.0, 2.0),
        dodgy_2d::Vec2::new(4.0, 1.0),
        dodgy_2d::Vec2::new(3.0, 1.0),
        dodgy_2d::Vec2::new(2.0, 1.0),
        dodgy_2d::Vec2::new(1.0, 1.0),
        dodgy_2d::Vec2::new(1.0, 2.0),
        dodgy_2d::Vec2::new(2.0, 2.0),
        dodgy_2d::Vec2::new(3.0, 2.0),
      ]
    }]
  );

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (Vec3::new(3.5, 0.0, 3.5), NodeRef { island_id, polygon_index: 4 }),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![dodgy_2d::Obstacle::Open {
      vertices: vec![
        dodgy_2d::Vec2::new(3.0, 2.0),
        dodgy_2d::Vec2::new(3.0, 3.0),
        dodgy_2d::Vec2::new(3.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 3.0),
        dodgy_2d::Vec2::new(4.0, 2.0),
        dodgy_2d::Vec2::new(4.0, 1.0),
        dodgy_2d::Vec2::new(3.0, 1.0),
        dodgy_2d::Vec2::new(2.0, 1.0),
      ]
    }]
  );

  // Decrease the distance limit to limit the size of the open obstacle.
  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (Vec3::new(3.5, 0.0, 3.5), NodeRef { island_id, polygon_index: 4 }),
      &nav_data,
      /* distance_limit= */ 1.0,
    ),
    vec![dodgy_2d::Obstacle::Open {
      vertices: vec![
        dodgy_2d::Vec2::new(3.0, 2.0),
        dodgy_2d::Vec2::new(3.0, 3.0),
        dodgy_2d::Vec2::new(3.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 3.0),
        dodgy_2d::Vec2::new(4.0, 2.0),
      ]
    }]
  );

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (Vec3::new(3.5, 0.0, 1.5), NodeRef { island_id, polygon_index: 2 }),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![dodgy_2d::Obstacle::Closed {
      vertices: vec![
        dodgy_2d::Vec2::new(1.0, 1.0),
        dodgy_2d::Vec2::new(1.0, 2.0),
        dodgy_2d::Vec2::new(2.0, 2.0),
        dodgy_2d::Vec2::new(3.0, 2.0),
        dodgy_2d::Vec2::new(3.0, 3.0),
        dodgy_2d::Vec2::new(3.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 4.0),
        dodgy_2d::Vec2::new(4.0, 3.0),
        dodgy_2d::Vec2::new(4.0, 2.0),
        dodgy_2d::Vec2::new(4.0, 1.0),
        dodgy_2d::Vec2::new(3.0, 1.0),
        dodgy_2d::Vec2::new(2.0, 1.0),
      ]
    }]
  );
}

#[test]
fn split_borders() {
  let nav_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(1.0, 0.0, 1.0),
      Vec3::new(2.0, 0.0, 1.0),
      Vec3::new(3.0, 0.0, 1.0),
      Vec3::new(4.0, 0.0, 1.0),
      Vec3::new(5.0, 0.0, 1.0),
      Vec3::new(5.0, 0.0, 2.0),
      Vec3::new(5.0, 0.0, 3.0),
      Vec3::new(5.0, 0.0, 4.0),
      Vec3::new(6.0, 0.0, 4.0),
      Vec3::new(6.0, 0.0, 3.0),
      Vec3::new(6.0, 0.0, 2.0),
      Vec3::new(6.0, 0.0, 1.0),
      Vec3::new(6.0, 0.0, 0.0),
      Vec3::new(5.0, 0.0, 0.0),
      Vec3::new(4.0, 0.0, 0.0),
      Vec3::new(3.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(0.0, 0.0, 1.0),
      Vec3::new(0.0, 0.0, 2.0),
      Vec3::new(0.0, 0.0, 3.0),
      Vec3::new(0.0, 0.0, 4.0),
      Vec3::new(1.0, 0.0, 4.0),
      Vec3::new(1.0, 0.0, 3.0),
      Vec3::new(1.0, 0.0, 2.0),
    ],
    polygons: vec![
      vec![0, 17, 16, 1],
      vec![1, 16, 15, 2],
      vec![2, 15, 14, 3],
      vec![3, 14, 13, 4],
      vec![4, 13, 12, 11],
      vec![4, 11, 10, 5],
      vec![5, 10, 9, 6],
      vec![6, 9, 8, 7],
      vec![0, 19, 18, 17],
      vec![25, 20, 19, 0],
      vec![24, 21, 20, 25],
      vec![23, 22, 21, 24],
    ],
  }
  .validate()
  .expect("Validation succeeds");

  let island_id = IslandId(1);

  let mut nav_data = NavigationData::new();
  nav_data.islands.insert(island_id, {
    let mut island = Island::new();
    island.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::new(nav_mesh),
    );
    island
  });

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (Vec3::new(3.0, 0.0, 0.9), NodeRef { island_id, polygon_index: 0 }),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![
      dodgy_2d::Obstacle::Open {
        vertices: vec![
          dodgy_2d::Vec2::new(1.0, 1.0),
          dodgy_2d::Vec2::new(2.0, 1.0),
          dodgy_2d::Vec2::new(3.0, 1.0),
          dodgy_2d::Vec2::new(4.0, 1.0),
          dodgy_2d::Vec2::new(5.0, 1.0),
        ]
      },
      dodgy_2d::Obstacle::Open {
        vertices: vec![
          dodgy_2d::Vec2::new(6.0, 2.0),
          dodgy_2d::Vec2::new(6.0, 1.0),
          dodgy_2d::Vec2::new(6.0, 0.0),
          dodgy_2d::Vec2::new(5.0, 0.0),
          dodgy_2d::Vec2::new(4.0, 0.0),
          dodgy_2d::Vec2::new(3.0, 0.0),
          dodgy_2d::Vec2::new(2.0, 0.0),
          dodgy_2d::Vec2::new(1.0, 0.0),
          dodgy_2d::Vec2::new(0.0, 0.0),
          dodgy_2d::Vec2::new(0.0, 1.0),
          dodgy_2d::Vec2::new(0.0, 2.0),
        ]
      }
    ]
  );
}

#[test]
fn creates_obstacles_across_boundary_link() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      mesh_bounds: None,
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 2.0),
        Vec3::new(1.0, 1.0, 2.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
    }
    .validate()
    .expect("Validation succeeds"),
  );

  let island_id_1 = IslandId(1);
  let island_id_2 = IslandId(2);

  let mut nav_data = NavigationData::new();
  nav_data.islands.insert(island_id_1, {
    let mut island = Island::new();
    island.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::clone(&nav_mesh),
    );
    island
  });
  nav_data.islands.insert(island_id_2, {
    let mut island = Island::new();
    island.set_nav_mesh(
      Transform { translation: Vec3::new(1.0, 0.0, 0.0), rotation: 0.0 },
      nav_mesh,
    );
    island
  });

  nav_data.update(0.01);

  assert_obstacles_match!(
    nav_mesh_borders_to_dodgy_obstacles(
      (
        Vec3::new(2.5, 1.0, 1.5),
        NodeRef { island_id: island_id_2, polygon_index: 0 }
      ),
      &nav_data,
      /* distance_limit= */ 10.0,
    ),
    vec![
      dodgy_2d::Obstacle::Open {
        vertices: vec![
          dodgy_2d::Vec2::new(2.0, 1.0),
          dodgy_2d::Vec2::new(1.0, 1.0),
          dodgy_2d::Vec2::new(1.0, 2.0),
          dodgy_2d::Vec2::new(2.0, 2.0),
        ]
      },
      dodgy_2d::Obstacle::Open {
        vertices: vec![
          dodgy_2d::Vec2::new(2.0, 2.0),
          dodgy_2d::Vec2::new(3.0, 2.0),
          dodgy_2d::Vec2::new(3.0, 1.0),
          dodgy_2d::Vec2::new(2.0, 1.0),
        ]
      }
    ]
  );
}

#[test]
fn applies_no_avoidance_for_far_agents() {
  const AGENT_1: AgentId = AgentId(1);
  const AGENT_2: AgentId = AgentId(2);
  const AGENT_3: AgentId = AgentId(3);

  let island_id = IslandId(1);

  let mut agents = HashMap::new();
  agents.insert(AGENT_1, {
    let mut agent = Agent::create(
      /* position= */ Vec3::new(1.0, 0.0, 1.0),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.01,
      /* max_velocity= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(1.0, 0.0, 0.0);
    agent
  });
  agents.insert(AGENT_2, {
    let mut agent = Agent::create(
      /* position= */ Vec3::new(11.0, 0.0, 1.0),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.01,
      /* max_velocity= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(-1.0, 0.0, 0.0);
    agent
  });
  agents.insert(AGENT_3, {
    let mut agent = Agent::create(
      /* position= */ Vec3::new(5.0, 0.0, 4.0),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.01,
      /* max_velocity= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(0.0, 0.0, 1.0);
    agent
  });

  let mut agent_id_to_agent_node = HashMap::new();
  agent_id_to_agent_node.insert(
    AGENT_1,
    (
      agents.get(&AGENT_1).unwrap().position,
      NodeRef { island_id, polygon_index: 0 },
    ),
  );
  agent_id_to_agent_node.insert(
    AGENT_2,
    (
      agents.get(&AGENT_2).unwrap().position,
      NodeRef { island_id, polygon_index: 0 },
    ),
  );
  // `AGENT_3` is not on a node.

  let nav_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(-1.0, 0.0, -1.0),
      Vec3::new(13.0, 0.0, -1.0),
      Vec3::new(13.0, 0.0, 3.0),
      Vec3::new(-1.0, 0.0, 3.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
  }
  .validate()
  .expect("Validation succeeded.");

  let mut nav_data = NavigationData::new();
  nav_data.islands.insert(island_id, {
    let mut island = Island::new();
    island.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::new(nav_mesh),
    );
    island
  });

  apply_avoidance_to_agents(
    &mut agents,
    &agent_id_to_agent_node,
    &nav_data,
    &AgentOptions { neighbourhood: 5.0, ..Default::default() },
    0.01,
  );

  assert_eq!(
    agents.get(&AGENT_1).unwrap().get_desired_velocity(),
    Vec3::new(1.0, 0.0, 0.0)
  );
  assert_eq!(
    agents.get(&AGENT_2).unwrap().get_desired_velocity(),
    Vec3::new(-1.0, 0.0, 0.0)
  );
  assert_eq!(
    agents.get(&AGENT_3).unwrap().get_desired_velocity(),
    Vec3::new(0.0, 0.0, 1.0)
  );
}

#[test]
fn applies_avoidance_for_two_agents() {
  const AGENT_1: AgentId = AgentId(1);
  const AGENT_2: AgentId = AgentId(2);

  let island_id = IslandId(1);

  let mut agents = HashMap::new();
  agents.insert(AGENT_1, {
    let mut agent = Agent::create(
      /* position= */ Vec3::new(1.0, 0.0, 1.0),
      /* velocity= */ Vec3::new(1.0, 0.0, 0.0),
      /* radius= */ 1.0,
      /* max_velocity= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(1.0, 0.0, 0.0);
    agent
  });
  agents.insert(AGENT_2, {
    let mut agent = Agent::create(
      /* position= */ Vec3::new(11.0, 0.0, 1.01),
      /* velocity= */ Vec3::new(-1.0, 0.0, 0.0),
      /* radius= */ 1.0,
      /* max_velocity= */ 1.0,
    );
    agent.current_desired_move = Vec3::new(-1.0, 0.0, 0.0);
    agent
  });

  let mut agent_id_to_agent_node = HashMap::new();
  agent_id_to_agent_node.insert(
    AGENT_1,
    (
      agents.get(&AGENT_1).unwrap().position,
      NodeRef { island_id, polygon_index: 0 },
    ),
  );
  agent_id_to_agent_node.insert(
    AGENT_2,
    (
      agents.get(&AGENT_2).unwrap().position,
      NodeRef { island_id, polygon_index: 0 },
    ),
  );

  let nav_mesh = NavigationMesh {
    mesh_bounds: None,
    vertices: vec![
      Vec3::new(-1.0, 0.0, -1.0),
      Vec3::new(13.0, 0.0, -1.0),
      Vec3::new(13.0, 0.0, 3.0),
      Vec3::new(-1.0, 0.0, 3.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
  }
  .validate()
  .expect("Validation succeeded.");

  let mut nav_data = NavigationData::new();
  nav_data.islands.insert(island_id, {
    let mut island = Island::new();
    island.set_nav_mesh(
      Transform { translation: Vec3::ZERO, rotation: 0.0 },
      Arc::new(nav_mesh),
    );
    island
  });

  apply_avoidance_to_agents(
    &mut agents,
    &agent_id_to_agent_node,
    &nav_data,
    &AgentOptions {
      neighbourhood: 15.0,
      avoidance_time_horizon: 15.0,
      ..Default::default()
    },
    0.01,
  );

  // The agents each have a radius of 1, and they are separated by a distance
  // of 10 (they start at (1,0) and (11,0)). So in order to pass each other, one
  // agent must go to (6,1) and the other agent must go to (6,-1). That's a rise
  // over run of 1/5 or 0.2, which is our expected Z velocity. We derive the X
  // velocity by just making the length of the vector 1 (the agent's max speed).
  let agent_1_desired_velocity =
    agents.get(&AGENT_1).unwrap().get_desired_velocity();
  assert!(
    agent_1_desired_velocity.abs_diff_eq(Vec3::new(0.98, 0.0, -0.2), 0.05),
    "left={agent_1_desired_velocity}, right=Vec3(0.98, 0.0, -0.2)"
  );
  let agent_2_desired_velocity =
    agents.get(&AGENT_2).unwrap().get_desired_velocity();
  assert!(
    agent_2_desired_velocity.abs_diff_eq(Vec3::new(-0.98, 0.0, 0.2), 0.05),
    "left={agent_2_desired_velocity}, right=Vec3(-0.98, 0.0, 0.2)"
  );
}

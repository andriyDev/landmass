mod agent;
mod astar;
mod avoidance;
mod nav_mesh;
mod path;
mod pathfinding;
mod util;

use glam::Vec3;
use rand::Rng;
use std::collections::HashMap;

use nav_mesh::MeshNodeRef;

pub use agent::{Agent, AgentId, TargetReachedCondition};
pub use nav_mesh::{NavigationMesh, ValidNavigationMesh};
pub use util::BoundingBox;

use crate::avoidance::{DodgyAvoidance, LocalCollisionAvoidance};

pub struct Archipelago {
  pub agent_options: AgentOptions,
  nav_data: NavigationData,
  agents: HashMap<AgentId, Agent>,
  local_collision_avoidance: Box<dyn LocalCollisionAvoidance>,
}

struct NavigationData {
  nav_mesh: ValidNavigationMesh,
}

// Options that apply to all agents
pub struct AgentOptions {
  // The distance to use when sampling agent and target points.
  pub node_sample_distance: f32,
  // The distance that an agent will consider avoiding another agent.
  pub neighbourhood: f32,
  // The time into the future that collisions with other agents should be
  // avoided.
  pub avoidance_time_horizon: f32,
  // The distance to stay away from the border of the nav mesh.
  pub obstacle_avoidance_margin: f32,
  // The time into the future that collisions with obstacles should be avoided.
  pub obstacle_avoidance_time_horizon: f32,
}

impl Default for AgentOptions {
  fn default() -> Self {
    Self {
      node_sample_distance: 0.1,
      neighbourhood: 5.0,
      obstacle_avoidance_margin: 0.1,
      avoidance_time_horizon: 1.0,
      obstacle_avoidance_time_horizon: 0.5,
    }
  }
}

impl Archipelago {
  pub fn create_from_navigation_mesh(
    navigation_mesh: ValidNavigationMesh,
  ) -> Self {
    Self {
      nav_data: NavigationData { nav_mesh: navigation_mesh },
      agent_options: AgentOptions::default(),
      agents: HashMap::new(),
      local_collision_avoidance: Box::new(DodgyAvoidance),
    }
  }

  pub fn add_agent(&mut self, agent: Agent) -> AgentId {
    let mut rng = rand::thread_rng();

    let agent_id: AgentId = rng.gen();
    assert!(self.agents.insert(agent_id, agent).is_none());

    agent_id
  }

  pub fn remove_agent(&mut self, agent_id: AgentId) {
    self
      .agents
      .remove(&agent_id)
      .expect("Agent should be present in the archipelago");
  }

  pub fn get_agent(&self, agent_id: AgentId) -> &Agent {
    self.agents.get(&agent_id).unwrap()
  }

  pub fn get_agent_mut(&mut self, agent_id: AgentId) -> &mut Agent {
    self.agents.get_mut(&agent_id).unwrap()
  }

  pub fn get_agent_ids(&self) -> impl ExactSizeIterator<Item = AgentId> + '_ {
    self.agents.keys().copied()
  }

  pub fn update(&mut self, delta_time: f32) {
    let mut agent_id_to_agent_node = HashMap::new();
    let mut agent_id_to_target_node = HashMap::new();

    for (agent_id, agent) in self.agents.iter() {
      let agent_node_and_point = match self
        .nav_data
        .nav_mesh
        .sample_point(agent.position, self.agent_options.node_sample_distance)
      {
        None => continue,
        Some(node_and_point) => node_and_point,
      };
      let inserted = agent_id_to_agent_node
        .insert(*agent_id, agent_node_and_point.clone())
        .is_none();
      debug_assert!(inserted);

      if let Some(target) = agent.current_target {
        let target_node_and_point = match self
          .nav_data
          .nav_mesh
          .sample_point(target, self.agent_options.node_sample_distance)
        {
          None => continue,
          Some(node_and_point) => node_and_point,
        };

        let inserted = agent_id_to_target_node
          .insert(*agent_id, target_node_and_point)
          .is_none();
        debug_assert!(inserted);
      }
    }

    let mut agent_id_to_follow_path_indices = HashMap::new();

    for (agent_id, agent) in self.agents.iter_mut() {
      let agent_node = agent_id_to_agent_node
        .get(agent_id)
        .map(|node_and_point| node_and_point.1.clone());
      let target_node = agent_id_to_target_node
        .get(agent_id)
        .map(|node_and_point| node_and_point.1.clone());
      match does_agent_need_repath(
        agent,
        agent_node.clone(),
        target_node.clone(),
      ) {
        RepathResult::DoNothing => {}
        RepathResult::FollowPath(
          agent_node_in_corridor,
          target_node_in_corridor,
        ) => {
          agent_id_to_follow_path_indices.insert(
            *agent_id,
            (agent_node_in_corridor, target_node_in_corridor),
          );
        }
        RepathResult::ClearPath => agent.current_path = None,
        RepathResult::NeedsRepath => {
          agent.current_path = None;

          let new_path = match pathfinding::find_path(
            &self.nav_data,
            agent_node.unwrap(),
            target_node.unwrap(),
          ) {
            Err(_) => continue,
            Ok(pathfinding::PathResult { path, .. }) => path,
          };

          agent_id_to_follow_path_indices
            .insert(*agent_id, (0, new_path.corridor.len() - 1));
          agent.current_path = Some(new_path);
        }
      }
    }

    for (agent_id, agent) in self.agents.iter_mut() {
      let path = match &agent.current_path {
        None => {
          agent.current_desired_move = Vec3::ZERO;
          continue;
        }
        Some(path) => path,
      };

      let agent_point = agent_id_to_agent_node
        .get(agent_id)
        .expect("Agent has a path, so should have a valid start node")
        .0;
      let target_point = agent_id_to_target_node
        .get(agent_id)
        .expect("Agent has a path, so should have a valid target node")
        .0;

      let &(agent_node_index_in_corridor, target_node_index_in_corridor) =
        agent_id_to_follow_path_indices.get(agent_id).expect(
          "Any agent with a path must have its follow path indices filled out.",
        );

      let next_waypoint = path.find_next_point_in_straight_path(
        &self.nav_data,
        agent_node_index_in_corridor,
        agent_point,
        target_node_index_in_corridor,
        target_point,
      );

      if agent.has_reached_target(
        path,
        &self.nav_data,
        next_waypoint,
        (target_node_index_in_corridor, target_point),
      ) {
        agent.current_desired_move = Vec3::ZERO;
      } else {
        agent.current_desired_move = (next_waypoint.1 - agent.position)
          .normalize_or_zero()
          * agent.max_velocity;
      }
    }

    self.local_collision_avoidance.apply_avoidance_to_agents(
      &mut self.agents,
      &agent_id_to_agent_node,
      &self.nav_data,
      &self.agent_options,
      delta_time,
    );
  }
}

#[derive(PartialEq, Eq, Debug)]
enum RepathResult {
  DoNothing,
  FollowPath(usize, usize),
  ClearPath,
  NeedsRepath,
}

fn does_agent_need_repath(
  agent: &Agent,
  agent_node: Option<MeshNodeRef>,
  target_node: Option<MeshNodeRef>,
) -> RepathResult {
  if let None = agent.current_target {
    if agent.current_path.is_some() {
      return RepathResult::ClearPath;
    } else {
      return RepathResult::DoNothing;
    }
  }

  let agent_node = match agent_node {
    None => return RepathResult::ClearPath,
    Some(result) => result,
  };
  let target_node = match target_node {
    None => return RepathResult::ClearPath,
    Some(result) => result,
  };

  let current_path = match &agent.current_path {
    None => return RepathResult::NeedsRepath,
    Some(current_path) => current_path,
  };

  let agent_node_index_in_corridor =
    match current_path.corridor.iter().position(|x| x == &agent_node) {
      None => return RepathResult::NeedsRepath,
      Some(index) => index,
    };

  let target_node_index_in_corridor =
    match current_path.corridor.iter().rev().position(|x| x == &target_node) {
      None => return RepathResult::NeedsRepath,
      Some(index) => current_path.corridor.len() - 1 - index,
    };

  match agent_node_index_in_corridor <= target_node_index_in_corridor {
    true => RepathResult::FollowPath(
      agent_node_index_in_corridor,
      target_node_index_in_corridor,
    ),
    false => RepathResult::NeedsRepath,
  }
}

#[cfg(test)]
mod tests {
  use std::{cell::Cell, collections::HashMap, rc::Rc};

  use glam::Vec3;

  use crate::{
    avoidance::LocalCollisionAvoidance, does_agent_need_repath,
    nav_mesh::MeshNodeRef, path::Path, Agent, AgentId, AgentOptions,
    Archipelago, NavigationData, NavigationMesh, RepathResult,
  };

  #[test]
  fn nothing_or_clear_path_for_no_target() {
    let mut agent = Agent::create(
      /* position= */ Vec3::ZERO,
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.0,
      /* max_velocity= */ 0.0,
    );

    assert_eq!(
      does_agent_need_repath(&agent, None, None),
      RepathResult::DoNothing
    );

    agent.current_path =
      Some(Path { corridor: vec![], portal_edge_index: vec![] });

    assert_eq!(
      does_agent_need_repath(&agent, None, None),
      RepathResult::ClearPath,
    );
  }

  #[test]
  fn clears_path_for_missing_nodes() {
    let mut agent = Agent::create(
      /* position= */ Vec3::ZERO,
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.0,
      /* max_velocity= */ 0.0,
    );
    agent.current_target = Some(Vec3::ZERO);

    assert_eq!(
      does_agent_need_repath(
        &agent,
        None,
        Some(MeshNodeRef { polygon_index: 0 }),
      ),
      RepathResult::ClearPath,
    );

    assert_eq!(
      does_agent_need_repath(
        &agent,
        Some(MeshNodeRef { polygon_index: 0 }),
        None
      ),
      RepathResult::ClearPath,
    );
  }

  #[test]
  fn repaths_for_invalid_path_or_nodes_off_path() {
    let mut agent = Agent::create(
      /* position= */ Vec3::ZERO,
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.0,
      /* max_velocity= */ 0.0,
    );
    agent.current_target = Some(Vec3::ZERO);

    assert_eq!(
      does_agent_need_repath(
        &agent,
        Some(MeshNodeRef { polygon_index: 1 }),
        Some(MeshNodeRef { polygon_index: 3 }),
      ),
      RepathResult::NeedsRepath,
    );

    agent.current_path = Some(Path {
      corridor: vec![
        MeshNodeRef { polygon_index: 2 },
        MeshNodeRef { polygon_index: 3 },
        MeshNodeRef { polygon_index: 4 },
        MeshNodeRef { polygon_index: 1 },
        MeshNodeRef { polygon_index: 0 },
      ],
      portal_edge_index: vec![],
    });

    // Missing agent node.
    assert_eq!(
      does_agent_need_repath(
        &agent,
        Some(MeshNodeRef { polygon_index: 5 }),
        Some(MeshNodeRef { polygon_index: 1 }),
      ),
      RepathResult::NeedsRepath,
    );

    // Missing target node.
    assert_eq!(
      does_agent_need_repath(
        &agent,
        Some(MeshNodeRef { polygon_index: 3 }),
        Some(MeshNodeRef { polygon_index: 6 }),
      ),
      RepathResult::NeedsRepath,
    );

    // Agent and target are in the wrong order.
    assert_eq!(
      does_agent_need_repath(
        &agent,
        Some(MeshNodeRef { polygon_index: 1 }),
        Some(MeshNodeRef { polygon_index: 3 }),
      ),
      RepathResult::NeedsRepath,
    );

    // Following is now fine.
    assert_eq!(
      does_agent_need_repath(
        &agent,
        Some(MeshNodeRef { polygon_index: 3 }),
        Some(MeshNodeRef { polygon_index: 1 }),
      ),
      RepathResult::FollowPath(1, 3),
    );
  }

  #[test]
  fn add_and_remove_agents() {
    let mut archipelago = Archipelago::create_from_navigation_mesh(
      NavigationMesh { mesh_bounds: None, vertices: vec![], polygons: vec![] }
        .validate()
        .unwrap(),
    );

    let agent_1 = archipelago.add_agent(Agent::create(
      /* position= */ Vec3::ZERO,
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 1.0,
      /* max_velocity= */ 0.0,
    ));

    let agent_2 = archipelago.add_agent(Agent::create(
      /* position= */ Vec3::ZERO,
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 2.0,
      /* max_velocity= */ 0.0,
    ));

    let agent_3 = archipelago.add_agent(Agent::create(
      /* position= */ Vec3::ZERO,
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 3.0,
      /* max_velocity= */ 0.0,
    ));

    fn sorted(mut v: Vec<AgentId>) -> Vec<AgentId> {
      v.sort();
      v
    }

    assert_eq!(
      sorted(archipelago.get_agent_ids().collect::<Vec<_>>()),
      sorted(vec![agent_1, agent_2, agent_3]),
    );
    assert_eq!(
      [
        archipelago.get_agent(agent_1).radius,
        archipelago.get_agent(agent_2).radius,
        archipelago.get_agent(agent_3).radius,
      ],
      [1.0, 2.0, 3.0],
    );

    archipelago.remove_agent(agent_2);

    assert_eq!(
      sorted(archipelago.get_agent_ids().collect::<Vec<_>>()),
      sorted(vec![agent_1, agent_3]),
    );
    assert_eq!(
      [
        archipelago.get_agent(agent_1).radius,
        archipelago.get_agent(agent_3).radius,
      ],
      [1.0, 3.0],
    );

    archipelago.remove_agent(agent_3);

    assert_eq!(
      sorted(archipelago.get_agent_ids().collect::<Vec<_>>()),
      sorted(vec![agent_1]),
    );
    assert_eq!([archipelago.get_agent(agent_1).radius], [1.0]);

    archipelago.remove_agent(agent_1);

    assert_eq!(archipelago.get_agent_ids().collect::<Vec<_>>(), []);
  }

  #[test]
  fn computes_and_follows_path() {
    let mut archipelago = Archipelago::create_from_navigation_mesh(
      NavigationMesh {
        mesh_bounds: None,
        vertices: vec![
          Vec3::new(1.0, 1.0, 1.0),
          Vec3::new(2.0, 1.0, 1.0),
          Vec3::new(3.0, 1.0, 1.0),
          Vec3::new(4.0, 1.0, 1.0),
          Vec3::new(4.0, 1.0, 2.0),
          Vec3::new(4.0, 1.0, 3.0),
          Vec3::new(4.0, 1.0, 4.0),
          Vec3::new(3.0, 1.0, 4.0),
          Vec3::new(3.0, 1.0, 3.0),
          Vec3::new(3.0, 1.0, 2.0),
          Vec3::new(2.0, 1.0, 2.0),
          Vec3::new(1.0, 1.0, 2.0),
        ],
        polygons: vec![
          vec![0, 1, 10, 11],
          vec![1, 2, 9, 10],
          vec![2, 3, 4, 9],
          vec![4, 5, 8, 9],
          vec![5, 6, 7, 8],
        ],
      }
      .validate()
      .expect("is valid"),
    );

    struct MockAvoidance(Rc<Cell<u32>>);
    impl LocalCollisionAvoidance for MockAvoidance {
      fn apply_avoidance_to_agents(
        &mut self,
        _agents: &mut HashMap<AgentId, Agent>,
        _agent_id_to_agent_node: &HashMap<AgentId, (Vec3, MeshNodeRef)>,
        _nav_data: &NavigationData,
        _agent_options: &AgentOptions,
        _delta_time: f32,
      ) {
        // Do nothing, just count the call.
        self.0.set(self.0.get() + 1);
      }
    }

    let avoidance_called = Rc::new(Cell::new(0));
    let avoidance = Box::new(MockAvoidance(avoidance_called.clone()));
    archipelago.local_collision_avoidance = avoidance;

    archipelago.update(/* delta_time= */ 0.01);
    // Updating does nothing other than calling the avoidance with no agents.
    assert_eq!(avoidance_called.get(), 1);

    let agent_1 = archipelago.add_agent(Agent::create(
      /* position= */ Vec3::new(1.5, 1.09, 1.5),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.5,
      /* max_velocity= */ 2.0,
    ));
    let agent_2 = archipelago.add_agent(Agent::create(
      /* position= */ Vec3::new(3.5, 0.95, 3.5),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.5,
      /* max_velocity= */ 2.0,
    ));
    let agent_off_mesh = archipelago.add_agent(Agent::create(
      /* position= */ Vec3::new(1.5, 1.0, 2.5),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.5,
      /* max_velocity= */ 2.0,
    ));
    let agent_too_high_above_mesh = archipelago.add_agent(Agent::create(
      /* position= */ Vec3::new(1.5, 1.11, 1.5),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.5,
      /* max_velocity= */ 2.0,
    ));

    archipelago.get_agent_mut(agent_1).current_target =
      Some(Vec3::new(3.5, 0.95, 3.5));
    archipelago.get_agent_mut(agent_off_mesh).current_target =
      Some(Vec3::new(3.5, 0.95, 3.5));
    archipelago.get_agent_mut(agent_too_high_above_mesh).current_target =
      Some(Vec3::new(3.5, 0.95, 3.5));
    archipelago.get_agent_mut(agent_2).current_target =
      Some(Vec3::new(1.5, 1.09, 1.5));

    // Nothing has happened yet.
    assert_eq!(
      archipelago.get_agent(agent_1).get_desired_velocity(),
      Vec3::ZERO
    );
    assert_eq!(
      archipelago.get_agent(agent_2).get_desired_velocity(),
      Vec3::ZERO
    );
    assert_eq!(
      archipelago.get_agent(agent_off_mesh).get_desired_velocity(),
      Vec3::ZERO
    );
    assert_eq!(
      archipelago.get_agent(agent_too_high_above_mesh).get_desired_velocity(),
      Vec3::ZERO
    );
    assert_eq!(avoidance_called.get(), 1);

    archipelago.update(/* delta_time= */ 0.01);

    // These agents found a path and started following it.
    assert!(archipelago
      .get_agent(agent_1)
      .get_desired_velocity()
      .abs_diff_eq(Vec3::new(1.5, -0.09, 0.5).normalize() * 2.0, 1e-7));
    assert!(archipelago
      .get_agent(agent_2)
      .get_desired_velocity()
      .abs_diff_eq(Vec3::new(-0.5, 0.05, -1.5).normalize() * 2.0, 1e-7));
    // These agents are not on the nav mesh, so they don't do anything.
    assert_eq!(
      archipelago.get_agent(agent_off_mesh).get_desired_velocity(),
      Vec3::ZERO
    );
    assert_eq!(
      archipelago.get_agent(agent_too_high_above_mesh).get_desired_velocity(),
      Vec3::ZERO
    );
    assert_eq!(avoidance_called.get(), 2);

    // Move agent_1 forward.
    archipelago.get_agent_mut(agent_1).position = Vec3::new(2.5, 1.0, 1.5);
    archipelago.update(/* delta_time= */ 0.01);

    assert!(archipelago
      .get_agent(agent_1)
      .get_desired_velocity()
      .abs_diff_eq(Vec3::new(0.5, 0.0, 0.5).normalize() * 2.0, 1e-7));
    // These agents don't change.
    assert!(archipelago
      .get_agent(agent_2)
      .get_desired_velocity()
      .abs_diff_eq(Vec3::new(-0.5, 0.05, -1.5).normalize() * 2.0, 1e-7));
    assert_eq!(
      archipelago.get_agent(agent_off_mesh).get_desired_velocity(),
      Vec3::ZERO
    );
    assert_eq!(
      archipelago.get_agent(agent_too_high_above_mesh).get_desired_velocity(),
      Vec3::ZERO
    );
    assert_eq!(avoidance_called.get(), 3);

    // Move agent_1 close enough to destination and agent_2 forward.
    archipelago.get_agent_mut(agent_1).position = Vec3::new(3.4, 1.0, 3.4);
    archipelago.get_agent_mut(agent_2).position = Vec3::new(3.5, 1.0, 2.5);
    archipelago.update(/* delta_time= */ 0.01);

    assert_eq!(
      archipelago.get_agent(agent_1).get_desired_velocity(),
      Vec3::ZERO
    );
    assert!(archipelago
      .get_agent(agent_2)
      .get_desired_velocity()
      .abs_diff_eq(Vec3::new(-0.5, 0.0, -0.5).normalize() * 2.0, 1e-7));
    // These agents don't change.
    assert_eq!(
      archipelago.get_agent(agent_off_mesh).get_desired_velocity(),
      Vec3::ZERO
    );
    assert_eq!(
      archipelago.get_agent(agent_too_high_above_mesh).get_desired_velocity(),
      Vec3::ZERO
    );
    assert_eq!(avoidance_called.get(), 4);
  }
}

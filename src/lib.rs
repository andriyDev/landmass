mod agent;
mod archipelago_pathfinding;
mod nav_mesh;
mod path;
mod pathfinding;
mod util;

use glam::Vec3;
use rand::Rng;
use std::collections::HashMap;

use nav_mesh::MeshNodeRef;

pub use agent::{Agent, AgentId};
pub use nav_mesh::{NavigationMesh, ValidNavigationMesh};
pub use util::BoundingBox;

pub struct Archipelago {
  nav_data: NavigationData,
  agents: HashMap<AgentId, Agent>,
}

struct NavigationData {
  nav_mesh: ValidNavigationMesh,
}

impl Archipelago {
  pub fn create_from_navigation_mesh(
    navigation_mesh: ValidNavigationMesh,
  ) -> Self {
    Self {
      nav_data: NavigationData { nav_mesh: navigation_mesh },
      agents: HashMap::new(),
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

  pub fn update(&mut self) {
    const NODE_DISTANCE: f32 = 0.1;

    let mut agent_id_to_agent_node = HashMap::new();
    let mut agent_id_to_target_node = HashMap::new();

    for (agent_id, agent) in self.agents.iter() {
      let agent_node_and_point = match self
        .nav_data
        .nav_mesh
        .sample_point(agent.get_position(), NODE_DISTANCE)
      {
        None => continue,
        Some(node_and_point) => node_and_point,
      };
      let inserted = agent_id_to_agent_node
        .insert(*agent_id, agent_node_and_point)
        .is_none();
      debug_assert!(inserted);

      if let Some(target) = agent.current_target {
        let target_node_and_point =
          match self.nav_data.nav_mesh.sample_point(target, NODE_DISTANCE) {
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

          let new_path = match archipelago_pathfinding::find_path(
            &self.nav_data,
            agent_node.unwrap(),
            target_node.unwrap(),
          ) {
            Err(_) => continue,
            Ok(archipelago_pathfinding::PathResult { path, .. }) => path,
          };

          agent_id_to_follow_path_indices
            .insert(*agent_id, (0, new_path.corridor.len()));
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

      let waypoint = path
        .find_next_point_in_straight_path(
          &self.nav_data,
          agent_node_index_in_corridor,
          agent_point,
          target_node_index_in_corridor,
          target_point,
        )
        .1;

      agent.current_desired_move =
        (waypoint - agent.position).normalize_or_zero() * agent.max_velocity;
    }
  }
}

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

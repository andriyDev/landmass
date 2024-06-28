#![doc = include_str!("../README.md")]

mod agent;
mod astar;
mod avoidance;
mod character;
mod geometry;
mod island;
mod nav_data;
mod nav_mesh;
mod path;
mod pathfinding;
mod util;

use path::PathIndex;
use slotmap::HopSlotMap;
use std::collections::{HashMap, HashSet};

use nav_data::{BoundaryLinkId, NavigationData, NodeRef};

pub use glam::Vec3;

pub mod debug;

pub use agent::{Agent, AgentId, AgentState, TargetReachedCondition};
pub use character::{Character, CharacterId};
pub use island::{Island, IslandId};
pub use nav_mesh::{NavigationMesh, ValidNavigationMesh, ValidationError};
pub use util::{BoundingBox, Transform};

use crate::avoidance::apply_avoidance_to_agents;

pub struct Archipelago {
  pub agent_options: AgentOptions,
  nav_data: NavigationData,
  agents: HopSlotMap<AgentId, Agent>,
  characters: HopSlotMap<CharacterId, Character>,
  pathing_results: Vec<PathingResult>,
}

/// Options that apply to all agents
pub struct AgentOptions {
  /// The distance to use when sampling agent and target points.
  pub node_sample_distance: f32,
  /// The distance that an agent will consider avoiding another agent.
  pub neighbourhood: f32,
  // The time into the future that collisions with other agents should be
  /// avoided.
  pub avoidance_time_horizon: f32,
  /// The time into the future that collisions with obstacles should be
  /// avoided.
  pub obstacle_avoidance_time_horizon: f32,
}

impl Default for AgentOptions {
  fn default() -> Self {
    Self {
      node_sample_distance: 0.1,
      neighbourhood: 5.0,
      avoidance_time_horizon: 1.0,
      obstacle_avoidance_time_horizon: 0.5,
    }
  }
}

impl Archipelago {
  pub fn new() -> Self {
    Self {
      nav_data: NavigationData::new(),
      agent_options: AgentOptions::default(),
      agents: HopSlotMap::with_key(),
      characters: HopSlotMap::with_key(),
      pathing_results: Vec::new(),
    }
  }

  pub fn add_agent(&mut self, agent: Agent) -> AgentId {
    self.agents.insert(agent)
  }

  pub fn remove_agent(&mut self, agent_id: AgentId) {
    self
      .agents
      .remove(agent_id)
      .expect("Agent should be present in the archipelago");
  }

  pub fn get_agent(&self, agent_id: AgentId) -> &Agent {
    self.agents.get(agent_id).unwrap()
  }

  pub fn get_agent_mut(&mut self, agent_id: AgentId) -> &mut Agent {
    self.agents.get_mut(agent_id).unwrap()
  }

  pub fn get_agent_ids(&self) -> impl ExactSizeIterator<Item = AgentId> + '_ {
    self.agents.keys()
  }

  pub fn add_character(&mut self, character: Character) -> CharacterId {
    self.characters.insert(character)
  }

  pub fn remove_character(&mut self, character_id: CharacterId) {
    self
      .characters
      .remove(character_id)
      .expect("Character should be present in the archipelago");
  }

  pub fn get_character(&self, character_id: CharacterId) -> &Character {
    self.characters.get(character_id).unwrap()
  }

  pub fn get_character_mut(
    &mut self,
    character_id: CharacterId,
  ) -> &mut Character {
    self.characters.get_mut(character_id).unwrap()
  }

  pub fn get_character_ids(
    &self,
  ) -> impl ExactSizeIterator<Item = CharacterId> + '_ {
    self.characters.keys()
  }

  pub fn add_island(&mut self) -> IslandId {
    self.nav_data.add_island()
  }

  pub fn remove_island(&mut self, island_id: IslandId) {
    self.nav_data.remove_island(island_id)
  }

  pub fn get_island(&self, island_id: IslandId) -> &Island {
    self.nav_data.islands.get(island_id).unwrap()
  }

  pub fn get_island_mut(&mut self, island_id: IslandId) -> &mut Island {
    self.nav_data.islands.get_mut(island_id).unwrap()
  }

  pub fn get_island_ids(&self) -> impl ExactSizeIterator<Item = IslandId> + '_ {
    self.nav_data.islands.keys()
  }

  /// Gets the pathing results from the last [`Self::update`] call.
  pub fn get_pathing_results(&self) -> &[PathingResult] {
    &self.pathing_results
  }

  pub fn update(&mut self, delta_time: f32) {
    self.pathing_results.clear();

    // TODO: make the edge_link_distance configurable.
    let (invalidated_boundary_links, invalidated_islands) =
      self.nav_data.update(/* edge_link_distance= */ 0.01);

    let mut agent_id_to_agent_node = HashMap::new();
    let mut agent_id_to_target_node = HashMap::new();

    for (agent_id, agent) in self.agents.iter() {
      let agent_node_and_point = match self
        .nav_data
        .sample_point(agent.position, self.agent_options.node_sample_distance)
      {
        None => continue,
        Some(node_and_point) => node_and_point,
      };
      let inserted =
        agent_id_to_agent_node.insert(agent_id, agent_node_and_point).is_none();
      debug_assert!(inserted);

      if let Some(target) = agent.current_target {
        let target_node_and_point = match self
          .nav_data
          .sample_point(target, self.agent_options.node_sample_distance)
        {
          None => continue,
          Some(node_and_point) => node_and_point,
        };

        let inserted = agent_id_to_target_node
          .insert(agent_id, target_node_and_point)
          .is_none();
        debug_assert!(inserted);
      }
    }

    let mut character_id_to_nav_mesh_point = HashMap::new();
    for (character_id, character) in self.characters.iter() {
      let character_point = match self.nav_data.sample_point(
        character.position,
        self.agent_options.node_sample_distance,
      ) {
        None => continue,
        Some(point_and_node) => point_and_node.0,
      };
      character_id_to_nav_mesh_point.insert(character_id, character_point);
    }

    let mut agent_id_to_follow_path_indices = HashMap::new();

    for (agent_id, agent) in self.agents.iter_mut() {
      let agent_node = agent_id_to_agent_node
        .get(&agent_id)
        .map(|node_and_point| node_and_point.1);
      let target_node = agent_id_to_target_node
        .get(&agent_id)
        .map(|node_and_point| node_and_point.1);
      match does_agent_need_repath(
        agent,
        agent_node,
        target_node,
        &invalidated_boundary_links,
        &invalidated_islands,
      ) {
        RepathResult::DoNothing => {}
        RepathResult::FollowPath(
          agent_node_in_corridor,
          target_node_in_corridor,
        ) => {
          agent_id_to_follow_path_indices.insert(
            agent_id,
            (agent_node_in_corridor, target_node_in_corridor),
          );
        }
        RepathResult::ClearPathNoTarget => {
          agent.state = AgentState::Idle;
          agent.current_path = None;
        }
        RepathResult::ClearPathBadAgent => {
          agent.state = AgentState::AgentNotOnNavMesh;
          agent.current_path = None;
        }
        RepathResult::ClearPathBadTarget => {
          agent.state = AgentState::TargetNotOnNavMesh;
          agent.current_path = None;
        }
        RepathResult::NeedsRepath => {
          agent.current_path = None;

          let new_path = match pathfinding::find_path(
            &self.nav_data,
            agent_node.unwrap(),
            target_node.unwrap(),
          ) {
            Err(stats) => {
              agent.state = AgentState::NoPath;
              self.pathing_results.push(PathingResult {
                agent: agent_id,
                success: false,
                explored_nodes: stats.explored_nodes,
              });
              continue;
            }
            Ok(pathfinding::PathResult { path, stats }) => {
              self.pathing_results.push(PathingResult {
                agent: agent_id,
                success: true,
                explored_nodes: stats.explored_nodes,
              });
              path
            }
          };

          agent_id_to_follow_path_indices.insert(
            agent_id,
            (PathIndex::from_corridor_index(0, 0), new_path.last_index()),
          );
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
        .get(&agent_id)
        .expect("Agent has a path, so should have a valid start node")
        .0;
      let target_point = agent_id_to_target_node
        .get(&agent_id)
        .expect("Agent has a path, so should have a valid target node")
        .0;

      let &(agent_node_index_in_corridor, target_node_index_in_corridor) =
        agent_id_to_follow_path_indices.get(&agent_id).expect(
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
        agent.state = AgentState::ReachedTarget;
      } else {
        agent.current_desired_move = (next_waypoint.1 - agent.position)
          .normalize_or_zero()
          * agent.max_velocity;
        agent.state = AgentState::Moving;
      }
    }

    apply_avoidance_to_agents(
      &mut self.agents,
      &agent_id_to_agent_node,
      &self.characters,
      &character_id_to_nav_mesh_point,
      &self.nav_data,
      &self.agent_options,
      delta_time,
    );
  }
}

impl Default for Archipelago {
  fn default() -> Self {
    Self::new()
  }
}

/// The result of path finding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PathingResult {
  /// The agent that searched for the path.
  pub agent: AgentId,
  /// Whether the pathing succeeded or failed.
  pub success: bool,
  /// The number of "nodes" explored while finding the path. Note this may be
  /// zero if the start and end point are known to be disconnected.
  pub explored_nodes: u32,
}

#[derive(PartialEq, Eq, Debug)]
enum RepathResult {
  DoNothing,
  FollowPath(PathIndex, PathIndex),
  ClearPathNoTarget,
  ClearPathBadAgent,
  ClearPathBadTarget,
  NeedsRepath,
}

fn does_agent_need_repath(
  agent: &Agent,
  agent_node: Option<NodeRef>,
  target_node: Option<NodeRef>,
  invalidated_boundary_links: &HashSet<BoundaryLinkId>,
  invalidated_islands: &HashSet<IslandId>,
) -> RepathResult {
  if agent.current_target.is_none() {
    if agent.current_path.is_some() {
      return RepathResult::ClearPathNoTarget;
    } else {
      return RepathResult::DoNothing;
    }
  }

  let agent_node = match agent_node {
    None => return RepathResult::ClearPathBadAgent,
    Some(result) => result,
  };
  let target_node = match target_node {
    None => return RepathResult::ClearPathBadTarget,
    Some(result) => result,
  };

  let current_path = match &agent.current_path {
    None => return RepathResult::NeedsRepath,
    Some(current_path) => current_path,
  };

  if !current_path.is_valid(invalidated_boundary_links, invalidated_islands) {
    return RepathResult::NeedsRepath;
  }

  let Some(agent_node_index_in_path) =
    current_path.find_index_of_node(agent_node)
  else {
    return RepathResult::NeedsRepath;
  };

  let Some(target_node_index_in_path) =
    current_path.find_index_of_node_rev(target_node)
  else {
    return RepathResult::NeedsRepath;
  };

  if agent_node_index_in_path > target_node_index_in_path {
    return RepathResult::NeedsRepath;
  }

  RepathResult::FollowPath(agent_node_index_in_path, target_node_index_in_path)
}

#[cfg(test)]
#[path = "lib_test.rs"]
mod test;
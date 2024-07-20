#![doc = include_str!("../README.md")]

mod agent;
mod astar;
mod avoidance;
mod character;
mod coords;
mod geometry;
mod island;
mod nav_data;
mod nav_mesh;
mod path;
mod pathfinding;
mod query;
mod util;

use agent::{does_agent_need_repath, RepathResult};
use path::PathIndex;
use slotmap::HopSlotMap;
use std::collections::HashMap;

use nav_data::NavigationData;

pub use glam::Vec3;

pub mod debug;

pub use agent::{Agent, AgentId, AgentState, TargetReachedCondition};
pub use character::{Character, CharacterId};
pub use coords::{CoordinateSystem, XYZ};
pub use island::{Island, IslandId};
pub use nav_data::{
  IslandMut, NewNodeTypeError, NodeType, SetNodeTypeCostError,
};
pub use nav_mesh::{NavigationMesh, ValidNavigationMesh, ValidationError};
pub use query::{FindPathError, SamplePointError, SampledPoint};
pub use util::Transform;

use crate::avoidance::apply_avoidance_to_agents;

pub struct Archipelago<CS: CoordinateSystem> {
  pub agent_options: AgentOptions,
  nav_data: NavigationData<CS>,
  agents: HopSlotMap<AgentId, Agent<CS>>,
  characters: HopSlotMap<CharacterId, Character<CS>>,
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

impl<CS: CoordinateSystem> Archipelago<CS> {
  pub fn new() -> Self {
    Self {
      nav_data: NavigationData::new(),
      agent_options: AgentOptions::default(),
      agents: HopSlotMap::with_key(),
      characters: HopSlotMap::with_key(),
      pathing_results: Vec::new(),
    }
  }

  pub fn add_agent(&mut self, agent: Agent<CS>) -> AgentId {
    self.agents.insert(agent)
  }

  pub fn remove_agent(&mut self, agent_id: AgentId) {
    self
      .agents
      .remove(agent_id)
      .expect("Agent should be present in the archipelago");
  }

  pub fn get_agent(&self, agent_id: AgentId) -> Option<&Agent<CS>> {
    self.agents.get(agent_id)
  }

  pub fn get_agent_mut(&mut self, agent_id: AgentId) -> Option<&mut Agent<CS>> {
    self.agents.get_mut(agent_id)
  }

  pub fn get_agent_ids(&self) -> impl ExactSizeIterator<Item = AgentId> + '_ {
    self.agents.keys()
  }

  pub fn add_character(&mut self, character: Character<CS>) -> CharacterId {
    self.characters.insert(character)
  }

  pub fn remove_character(&mut self, character_id: CharacterId) {
    self
      .characters
      .remove(character_id)
      .expect("Character should be present in the archipelago");
  }

  pub fn get_character(
    &self,
    character_id: CharacterId,
  ) -> Option<&Character<CS>> {
    self.characters.get(character_id)
  }

  pub fn get_character_mut(
    &mut self,
    character_id: CharacterId,
  ) -> Option<&mut Character<CS>> {
    self.characters.get_mut(character_id)
  }

  pub fn get_character_ids(
    &self,
  ) -> impl ExactSizeIterator<Item = CharacterId> + '_ {
    self.characters.keys()
  }

  pub fn add_island(&mut self, island: Island<CS>) -> IslandId {
    self.nav_data.add_island(island)
  }

  pub fn remove_island(&mut self, island_id: IslandId) {
    self.nav_data.remove_island(island_id)
  }

  pub fn get_island(&self, island_id: IslandId) -> Option<&Island<CS>> {
    self.nav_data.get_island(island_id)
  }

  pub fn get_island_mut(
    &mut self,
    island_id: IslandId,
  ) -> Option<IslandMut<'_, CS>> {
    self.nav_data.get_island_mut(island_id)
  }

  pub fn get_island_ids(&self) -> impl ExactSizeIterator<Item = IslandId> + '_ {
    self.nav_data.get_island_ids()
  }

  /// Creates a new node type with the specified `cost`. The cost is a
  /// multiplier on the distance travelled along this node (essentially the cost
  /// per meter). Agents will prefer to travel along low-cost terrain. The
  /// returned node type is distinct from all other node types (for this
  /// archipelago).
  pub fn add_node_type(
    &mut self,
    cost: f32,
  ) -> Result<NodeType, NewNodeTypeError> {
    self.nav_data.add_node_type(cost)
  }

  /// Sets the cost of `node_type` to `cost`. See
  /// [`Archipelago::add_node_type`] for the meaning of cost.
  pub fn set_node_type_cost(
    &mut self,
    node_type: NodeType,
    cost: f32,
  ) -> Result<(), SetNodeTypeCostError> {
    self.nav_data.set_node_type_cost(node_type, cost)
  }

  /// Gets the cost of `node_type`. Returns [`None`] if `node_type` is not in
  /// this archipelago.
  pub fn get_node_type_cost(&self, node_type: NodeType) -> Option<f32> {
    self.nav_data.get_node_type_cost(node_type)
  }

  /// Removes the node type from the archipelago. Returns false if this
  /// archipelago does not contain `node_type` or any islands still use this
  /// node type (so the node type cannot be removed). Otherwise, returns true.
  pub fn remove_node_type(&mut self, node_type: NodeType) -> bool {
    self.nav_data.remove_node_type(node_type)
  }

  /// Gets the pathing results from the last [`Self::update`] call.
  pub fn get_pathing_results(&self) -> &[PathingResult] {
    &self.pathing_results
  }

  /// Finds the nearest point on the navigation meshes to (and within
  /// `distance_to_node` of) `point`.
  pub fn sample_point(
    &self,
    point: CS::Coordinate,
    distance_to_node: f32,
  ) -> Result<SampledPoint<'_, CS>, SamplePointError> {
    query::sample_point(self, point, distance_to_node)
  }

  /// Finds a path from `start_point` and `end_point` along the navigation
  /// meshes. Only [`SampledPoint`]s from this archipelago are supported. This
  /// should only be used for querying (e.g., finding the walking distance to an
  /// object), not for controlling movement. For controlling movement, use
  /// agents.
  pub fn find_path(
    &self,
    start_point: &SampledPoint<'_, CS>,
    end_point: &SampledPoint<'_, CS>,
    override_node_type_costs: &HashMap<NodeType, f32>,
  ) -> Result<Vec<CS::Coordinate>, FindPathError> {
    query::find_path(self, start_point, end_point, override_node_type_costs)
  }

  pub fn update(&mut self, delta_time: f32) {
    self.pathing_results.clear();

    // TODO: make the edge_link_distance configurable.
    let (invalidated_boundary_links, invalidated_islands) =
      self.nav_data.update(/* edge_link_distance= */ 0.01);

    let mut agent_id_to_agent_node = HashMap::new();
    let mut agent_id_to_target_node = HashMap::new();

    for (agent_id, agent) in self.agents.iter() {
      let agent_node_and_point = match self.nav_data.sample_point(
        CS::to_landmass(&agent.position),
        self.agent_options.node_sample_distance,
      ) {
        None => continue,
        Some(node_and_point) => node_and_point,
      };
      let inserted =
        agent_id_to_agent_node.insert(agent_id, agent_node_and_point).is_none();
      debug_assert!(inserted);

      if let Some(target) = &agent.current_target {
        let target_node_and_point = match self.nav_data.sample_point(
          CS::to_landmass(target),
          self.agent_options.node_sample_distance,
        ) {
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
        CS::to_landmass(&character.position),
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

          let path_result = pathfinding::find_path(
            &self.nav_data,
            agent_node.unwrap(),
            target_node.unwrap(),
            &agent.override_node_type_to_cost,
          );

          self.pathing_results.push(PathingResult {
            agent: agent_id,
            success: path_result.path.is_some(),
            explored_nodes: path_result.stats.explored_nodes,
          });

          let Some(new_path) = path_result.path else {
            agent.state = AgentState::NoPath;
            continue;
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
          agent.current_desired_move = CS::from_landmass(&Vec3::ZERO);
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
        agent.current_desired_move = CS::from_landmass(&Vec3::ZERO);
        agent.state = AgentState::ReachedTarget;
      } else {
        let desired_move = (next_waypoint.1 - CS::to_landmass(&agent.position))
          .normalize_or_zero()
          * agent.max_velocity;
        agent.current_desired_move = CS::from_landmass(&desired_move);
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

impl<CS: CoordinateSystem> Default for Archipelago<CS> {
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

#[cfg(test)]
#[path = "lib_test.rs"]
mod test;

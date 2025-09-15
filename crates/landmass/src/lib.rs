#![doc = include_str!("../README.md")]

mod agent;
mod astar;
mod avoidance;
mod character;
mod coords;
mod geometry;
mod island;
mod link;
mod nav_data;
mod nav_mesh;
mod path;
mod pathfinding;
mod query;
mod util;

use agent::{RepathResult, does_agent_need_repath};
use glam::Vec3Swizzles;
use path::PathIndex;
use slotmap::HopSlotMap;
use std::collections::HashMap;

use nav_data::NavigationData;

pub use glam::Vec3;

pub mod debug;

pub use agent::{
  Agent, AgentId, AgentState, NotReachedAnimationLinkError,
  PermittedAnimationLinks, ReachedAnimationLink, TargetReachedCondition,
};
pub use character::{Character, CharacterId};
pub use coords::{
  CoordinateSystem, FromAgentRadius, PointSampleDistance,
  PointSampleDistance3d, XY, XYZ,
};
pub use island::{Island, IslandId};
pub use link::{AnimationLink, AnimationLinkId};
pub use nav_data::{IslandMut, SetTypeIndexCostError};
pub use nav_mesh::{
  HeightNavigationMesh, HeightPolygon, NavigationMesh, ValidNavigationMesh,
  ValidationError,
};
pub use query::{FindPathError, PathStep, SamplePointError, SampledPoint};
pub use util::Transform;

use crate::{
  avoidance::apply_avoidance_to_agents, coords::CorePointSampleDistance,
  nav_data::NodeRef, path::StraightPathStep,
};

pub struct Archipelago<CS: CoordinateSystem> {
  pub archipelago_options: ArchipelagoOptions<CS>,
  nav_data: NavigationData<CS>,
  agents: HopSlotMap<AgentId, Agent<CS>>,
  characters: HopSlotMap<CharacterId, Character<CS>>,
  pathing_results: Vec<PathingResult>,
}

/// Options that apply to the entire archipelago.
pub struct ArchipelagoOptions<CS: CoordinateSystem> {
  /// The options for sampling agent and target points.
  pub point_sample_distance: CS::SampleDistance,
  /// The distance that an agent will consider avoiding another agent.
  pub neighbourhood: f32,
  // The time into the future that collisions with other agents should be
  /// avoided.
  pub avoidance_time_horizon: f32,
  /// The time into the future that collisions with obstacles should be
  /// avoided.
  pub obstacle_avoidance_time_horizon: f32,
  /// The avoidance responsibility to use when an agent has reached its target.
  /// A value of 1.0 is the default avoidance responsibility. A value of 0.0
  /// would mean no avoidance responsibility, but a value of 0.0 is invalid and
  /// may panic. This should be a value between 0.0 and 1.0.
  pub reached_destination_avoidance_responsibility: f32,
}

impl<CS: CoordinateSystem<SampleDistance: FromAgentRadius>> FromAgentRadius
  for ArchipelagoOptions<CS>
{
  fn from_agent_radius(radius: f32) -> Self {
    Self {
      point_sample_distance: CS::SampleDistance::from_agent_radius(radius),
      neighbourhood: 10.0 * radius,
      avoidance_time_horizon: 1.0,
      obstacle_avoidance_time_horizon: 0.5,
      reached_destination_avoidance_responsibility: 0.1,
    }
  }
}

impl<CS: CoordinateSystem> Archipelago<CS> {
  pub fn new(archipelago_options: ArchipelagoOptions<CS>) -> Self {
    Self {
      archipelago_options,
      nav_data: NavigationData::new(),
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

  pub fn add_animation_link(
    &mut self,
    link: AnimationLink<CS>,
  ) -> AnimationLinkId {
    self.nav_data.add_animation_link(link)
  }

  pub fn remove_animation_link(&mut self, link_id: AnimationLinkId) {
    self.nav_data.remove_animation_link(link_id)
  }

  pub fn get_animation_link(
    &self,
    link_id: AnimationLinkId,
  ) -> Option<&AnimationLink<CS>> {
    self.nav_data.get_animation_link(link_id)
  }

  pub fn get_animation_link_ids(
    &self,
  ) -> impl ExactSizeIterator<Item = AnimationLinkId> {
    self.nav_data.get_animation_link_ids()
  }

  /// Sets the cost of `type_index` to `cost`. The cost is a multiplier on the
  /// distance travelled along this node (essentially the cost per meter).
  /// Agents will prefer to travel along low-cost terrain.
  pub fn set_type_index_cost(
    &mut self,
    type_index: usize,
    cost: f32,
  ) -> Result<(), SetTypeIndexCostError> {
    self.nav_data.set_type_index_cost(type_index, cost)
  }

  /// Gets the cost of `type_index`.
  pub fn get_type_index_cost(&self, type_index: usize) -> Option<f32> {
    self.nav_data.get_type_index_cost(type_index)
  }

  /// Gets the current registered type indices and their costs.
  pub fn get_type_index_costs(
    &self,
  ) -> impl Iterator<Item = (usize, f32)> + '_ {
    self.nav_data.get_type_index_costs()
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
    point_sample_distance: &CS::SampleDistance,
  ) -> Result<SampledPoint<'_, CS>, SamplePointError> {
    query::sample_point(
      self,
      point,
      &CorePointSampleDistance::new(point_sample_distance),
    )
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
    override_type_index_costs: &HashMap<usize, f32>,
    permitted_animation_links: PermittedAnimationLinks,
  ) -> Result<Vec<PathStep<CS>>, FindPathError> {
    query::find_path(
      self,
      start_point,
      end_point,
      override_type_index_costs,
      permitted_animation_links,
    )
  }

  pub fn update(&mut self, delta_time: f32) {
    self.pathing_results.clear();

    // TODO: make the edge_link_distance configurable.
    let (invalidated_off_mesh_links, invalidated_islands) =
      self.nav_data.update(
        /* edge_link_distance= */ 0.01,
        /* animation_link_distance= */ 0.01,
      );

    let mut agent_id_to_agent_node = HashMap::new();
    let mut agent_id_to_target_node = HashMap::new();

    for (agent_id, agent) in self.agents.iter_mut() {
      if agent.paused {
        // We don't care to sample the agent location if the agent is paused.
        agent.state = AgentState::Paused;
        continue;
      }
      if agent.using_animation_link {
        // We don't care to sample the agent location if the agent is using an
        // animation link.
        agent.state = AgentState::UsingAnimationLink;
        continue;
      }
      let agent_node_and_point = match self.nav_data.sample_point(
        CS::to_landmass(&agent.position),
        &CorePointSampleDistance::new(
          &self.archipelago_options.point_sample_distance,
        ),
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
          &CorePointSampleDistance::new(
            &self.archipelago_options.point_sample_distance,
          ),
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
        &CorePointSampleDistance::new(
          &self.archipelago_options.point_sample_distance,
        ),
      ) {
        None => continue,
        Some(point_and_node) => point_and_node.0,
      };
      character_id_to_nav_mesh_point.insert(character_id, character_point);
    }

    let mut agent_id_to_follow_path_indices = HashMap::new();

    for (agent_id, agent) in self.agents.iter_mut() {
      // Clear the animation link whether the agent is paused or not. If we
      // still reached the same animation link, we'll re-set it.
      agent.current_animation_link = None;

      if agent.paused || agent.using_animation_link {
        if let Some(path) = agent.current_path.as_ref()
          && !path.is_valid(&invalidated_off_mesh_links, &invalidated_islands)
        {
          // If the path has been invalidated, clear the path to keep the agent
          // consistent.
          agent.current_path = None;
        }
        continue;
      }
      let agent_point_and_node = agent_id_to_agent_node.get(&agent_id);
      let target_point_and_node = agent_id_to_target_node.get(&agent_id);
      match does_agent_need_repath(
        agent,
        agent_point_and_node.map(|(_, node)| *node),
        target_point_and_node.map(|(_, node)| *node),
        &invalidated_off_mesh_links,
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

          let (agent_point, agent_node) = agent_point_and_node.unwrap();
          let (target_point, target_node) = target_point_and_node.unwrap();
          let path_result = pathfinding::find_path(
            &self.nav_data,
            *agent_node,
            *agent_point,
            *target_node,
            *target_point,
            &agent.override_type_index_to_cost,
            agent.permitted_animation_links.clone(),
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

      let Some(agent_point) =
        agent_id_to_agent_node.get(&agent_id).map(|x| x.0)
      else {
        // If the agent is paused, they may not have an agent node, even if the
        // agent has a path.
        continue;
      };
      let target_point = agent_id_to_target_node
        .get(&agent_id)
        .expect("Agent has a path and is not paused, so should have a valid target node")
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
        let waypoint = match next_waypoint.1 {
          StraightPathStep::Waypoint(point) => {
            agent.state = AgentState::Moving;
            point
          }
          StraightPathStep::AnimationLink {
            start_point,
            end_point,
            link_id,
            start_node,
            end_node,
          } => {
            // TODO: Consider moving this into find_next_point_in_straight_path.
            fn sample_point<CS: CoordinateSystem>(
              nav_data: &NavigationData<CS>,
              point: Vec3,
              node: NodeRef,
            ) -> Vec3 {
              let island = nav_data.get_island(node.island_id).unwrap();
              let point = island.transform.apply_inverse(point);
              let point =
                island.nav_mesh.sample_point_on_node(point, node.polygon_index);
              island.transform.apply(point)
            }
            // Refine the start and end points of the animation link to be
            // actually on the nav mesh.
            let start_point =
              sample_point(&self.nav_data, start_point, start_node);
            let end_point = sample_point(&self.nav_data, end_point, end_node);

            let distance = agent_point.distance(start_point);
            if distance <= agent.animation_link_reached_distance() {
              agent.state = AgentState::ReachedAnimationLink;
              agent.current_animation_link = Some(ReachedAnimationLink {
                start_point: CS::from_landmass(&start_point),
                end_point: CS::from_landmass(&end_point),
                link_id,
              });
            } else {
              agent.state = AgentState::Moving;
            }
            start_point
          }
        };

        let desired_move = (waypoint - CS::to_landmass(&agent.position))
          .xy()
          .normalize_or_zero()
          * agent.desired_speed;

        agent.current_desired_move =
          CS::from_landmass(&desired_move.extend(0.0));
      }
    }

    apply_avoidance_to_agents(
      &mut self.agents,
      &agent_id_to_agent_node,
      &self.characters,
      &character_id_to_nav_mesh_point,
      &self.nav_data,
      &self.archipelago_options,
      delta_time,
    );
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

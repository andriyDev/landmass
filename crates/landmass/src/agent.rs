use std::collections::{HashMap, HashSet};

use glam::Vec3;
use slotmap::new_key_type;

use crate::{
  nav_data::{BoundaryLinkId, NodeRef},
  path::{Path, PathIndex},
  CoordinateSystem, IslandId, NavigationData, NodeType,
};

new_key_type! {
  /// The ID of an agent.
  pub struct AgentId;
}

/// The state of an agent.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AgentState {
  /// The agent is idle, due to not having a target. Note this does not mean
  /// that they are motionless. An agent will still avoid nearby agents.
  Idle,
  /// The agent has reached their target. The agent may resume moving if the
  /// target moves or otherwise changes.
  ReachedTarget,
  /// The agent has a path and is moving towards their target.
  Moving,
  /// The agent is not on a nav mesh.
  AgentNotOnNavMesh,
  /// The target is not on a nav mesh.
  TargetNotOnNavMesh,
  /// The agent has a target but cannot find a path to it.
  NoPath,
}

/// An agent in an archipelago.
pub struct Agent<CS: CoordinateSystem> {
  /// The current position of the agent.
  pub position: CS::Coordinate,
  /// The current velocity of the agent.
  pub velocity: CS::Coordinate,
  /// The radius of the agent.
  pub radius: f32,
  /// The speed the agent prefers to move at. This should often be set lower
  /// than the [`Self::max_speed`] to allow the agent to "speed up" in order to
  /// get out of another agent's way.
  pub desired_speed: f32,
  /// The maximum speed that the agent can move at.
  pub max_speed: f32,
  /// The current target to move towards. Modifying this every update is fine.
  /// Paths will be reused for target points near each other if possible.
  /// However, swapping between two distant targets every update can be
  /// detrimental to be performance.
  pub current_target: Option<CS::Coordinate>,
  /// The condition to test for reaching the target.
  pub target_reached_condition: TargetReachedCondition,
  #[cfg(feature = "debug-avoidance")]
  /// If true, avoidance debug data will be stored during update iterations.
  /// This can later be used for visualization.
  pub keep_avoidance_data: bool,
  /// Overrides for the "default" costs of each [`NodeType`].
  pub(crate) override_node_type_to_cost: HashMap<NodeType, f32>,
  /// The current path of the agent. None if a path is unavailable or a new
  /// path has not been computed yet (i.e., no path).
  pub(crate) current_path: Option<Path>,
  /// The desired velocity of the agent to move towards its goal.
  pub(crate) current_desired_move: CS::Coordinate,
  /// The state of the agent.
  pub(crate) state: AgentState,
  #[cfg(feature = "debug-avoidance")]
  /// The avoidance data from the most recent update iteration. Only populated
  /// if [`Self::keep_avoidance_data`] is true.
  pub(crate) avoidance_data: Option<dodgy_2d::debug::DebugData>,
}

/// The condition to consider the agent as having reached its target. When this
/// condition is satisfied, the agent will stop moving.
#[derive(Clone, Copy, Debug)]
pub enum TargetReachedCondition {
  /// The target is reached if it is within the provided (Euclidean) distance
  /// of the agent. Useful if the target is surrounded by small obstacles
  /// which don't need to be navigated around (e.g. the agent just needs to
  /// be close enough to shoot at the target, which is surrounded by cover).
  /// Alternatively, if the distance is low, this can simply mean "when the
  /// agent is really close to the target". If None, the agent's radius is
  /// used.
  Distance(Option<f32>),
  /// The target is reached if it is "visible" (there is a straight line from
  /// the agent to the target), and the target is within the provided
  /// (Euclidean) distance of the agent. Useful if the agent should be able
  /// to see the target (e.g. a companion character should remain visible to
  /// the player, but should ideally not stand too close). If None, the agent's
  /// radius is used.
  VisibleAtDistance(Option<f32>),
  /// The target is reached if the "straight line" path from the agent to the
  /// target is less than the provided distance. "Straight line" path means if
  /// the agent's path goes around a corner, the distance will be computed
  /// going around the corner. This can be more computationally expensive, as
  /// the straight line path must be computed every update. Useful for agents
  /// that care about the actual walking distance to the target. If None, the
  /// agent's radius is used.
  StraightPathDistance(Option<f32>),
}

impl Default for TargetReachedCondition {
  fn default() -> Self {
    Self::Distance(None)
  }
}

impl<CS: CoordinateSystem> Agent<CS> {
  /// Creates a new agent.
  pub fn create(
    position: CS::Coordinate,
    velocity: CS::Coordinate,
    radius: f32,
    desired_speed: f32,
    max_speed: f32,
  ) -> Self {
    Self {
      position,
      velocity,
      radius,
      desired_speed,
      max_speed,
      current_target: None,
      target_reached_condition: TargetReachedCondition::Distance(None),
      #[cfg(feature = "debug-avoidance")]
      keep_avoidance_data: false,
      override_node_type_to_cost: HashMap::new(),
      current_path: None,
      current_desired_move: CS::from_landmass(&Vec3::ZERO),
      state: AgentState::Idle,
      #[cfg(feature = "debug-avoidance")]
      avoidance_data: None,
    }
  }

  /// Sets the node type cost for this agent to `cost`. Returns false if the
  /// cost is <= 0.0. Otherwise returns true.
  pub fn override_node_type_cost(
    &mut self,
    node_type: NodeType,
    cost: f32,
  ) -> bool {
    if cost <= 0.0 {
      return false;
    }
    self.override_node_type_to_cost.insert(node_type, cost);
    true
  }

  /// Removes the override cost for `node_type`. Returns true if `node_type` was
  /// overridden, false otherwise.
  pub fn remove_overridden_node_type_cost(
    &mut self,
    node_type: NodeType,
  ) -> bool {
    self.override_node_type_to_cost.remove(&node_type).is_some()
  }

  /// Returns the currently overriden node type costs.
  pub fn get_node_type_cost_overrides(
    &self,
  ) -> impl Iterator<Item = (NodeType, f32)> + '_ {
    self
      .override_node_type_to_cost
      .iter()
      .map(|(&node_type, &cost)| (node_type, cost))
  }

  /// Returns the desired velocity. This will only be updated if `update` was
  /// called on the associated [`crate::Archipelago`].
  pub fn get_desired_velocity(&self) -> &CS::Coordinate {
    &self.current_desired_move
  }

  /// Returns the state of the agent. This will only be updated if `update` was
  /// called on the associated [`crate::Archipelago`].
  pub fn state(&self) -> AgentState {
    self.state
  }

  /// Determines if this agent has reached its target. `next_waypoint` and
  /// `target_waypoint` are formatted as an index into the `path` and the point
  /// of the waypoint. `next_waypoint` is the next waypoint on the way to the
  /// target. `target_waypoint` is the final waypoint that corresponds to the
  /// target.
  pub(crate) fn has_reached_target(
    &self,
    path: &Path,
    nav_data: &NavigationData<CS>,
    next_waypoint: (PathIndex, Vec3),
    target_waypoint: (PathIndex, Vec3),
  ) -> bool {
    let position = CS::to_landmass(&self.position);
    match self.target_reached_condition {
      TargetReachedCondition::Distance(distance) => {
        let distance = distance.unwrap_or(self.radius);
        position.distance_squared(target_waypoint.1) < distance * distance
      }
      TargetReachedCondition::VisibleAtDistance(distance) => {
        let distance = distance.unwrap_or(self.radius);
        next_waypoint.0 == target_waypoint.0
          && position.distance_squared(next_waypoint.1) < distance * distance
      }
      TargetReachedCondition::StraightPathDistance(distance) => 'result: {
        let distance = distance.unwrap_or(self.radius);
        // Check Euclidean distance first so we don't do the expensive path
        // following if the agent is not even close.
        if position.distance_squared(target_waypoint.1) > distance * distance {
          break 'result false;
        }

        // If the next waypoint is the target point, then we've already
        // computed the straight line distance and it is below the limit.
        if next_waypoint.0 == target_waypoint.0 {
          break 'result true;
        }

        let mut straight_line_distance = position.distance(next_waypoint.1);
        let mut current_waypoint = next_waypoint;

        while current_waypoint.0 != target_waypoint.0
          && straight_line_distance < distance
        {
          let next_waypoint = path.find_next_point_in_straight_path(
            nav_data,
            current_waypoint.0,
            current_waypoint.1,
            target_waypoint.0,
            target_waypoint.1,
          );

          straight_line_distance +=
            current_waypoint.1.distance(next_waypoint.1);
          current_waypoint = next_waypoint;
        }

        straight_line_distance < distance
      }
    }
  }
}

/// The determination of what to do in regards to an agent's path.
#[derive(PartialEq, Eq, Debug)]
pub(crate) enum RepathResult {
  /// Do nothing.
  DoNothing,
  /// The existing path should be followed. Stores the index in a path for the
  /// first portal and the target point.
  FollowPath(PathIndex, PathIndex),
  /// Clear the path and don't repath, since there is no longer a target.
  ClearPathNoTarget,
  /// Clear the path and don't repath, since the agent is not on a valid node.
  ClearPathBadAgent,
  /// Clear the path and don't repath, since the target is not on a valid node.
  ClearPathBadTarget,
  /// Recompute the path.
  NeedsRepath,
}

pub(crate) fn does_agent_need_repath<CS: CoordinateSystem>(
  agent: &Agent<CS>,
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
#[path = "agent_test.rs"]
mod test;

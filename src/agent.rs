use glam::Vec3;

use crate::{path::Path, NavigationData};

pub type AgentId = u32;

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
pub struct Agent {
  /// The current position of the agent.
  pub position: Vec3,
  /// The current velocity of the agent.
  pub velocity: Vec3,
  /// The radius of the agent.
  pub radius: f32,
  /// The maximum velocity that the agent can move at.
  pub max_velocity: f32,
  /// The current target to move towards. Modifying this every update is fine.
  /// Paths will be reused for target points near each other if possible.
  /// However, swapping between two distant targets every update can be
  /// detrimental to be performance.
  pub current_target: Option<Vec3>,
  /// The condition to test for reaching the target.
  pub target_reached_condition: TargetReachedCondition,
  /// The current path of the agent. None if a path is unavailable or a new
  /// path has not been computed yet (i.e., no path).
  pub(crate) current_path: Option<Path>,
  /// The desired velocity of the agent to move towards its goal.
  pub(crate) current_desired_move: Vec3,
  /// The state of the agent.
  pub(crate) state: AgentState,
}

/// The condition to consider the agent as having reached its target. When this
/// condition is satisfied, the agent will stop moving.
pub enum TargetReachedCondition {
  /// The target is reached if it is within the provided (Euclidean) distance
  /// of the agent. Useful if the target is surrounded by small obstacles
  /// which don't need to be navigated around (e.g. the agent just needs to
  /// be close enough to shoot at the target, which is surrounded by cover).
  /// Alternatively, if the distance is low, this can simply mean "when the
  /// agent is really close to the target".
  Distance(f32),
  /// The target is reached if it is "visible" (there is a straight line from
  /// the agent to the target), and the target is within the provided
  /// (Euclidean) distance of the agent. Useful if the agent should be able
  /// to see the target (e.g. a companion character should remain visible to
  /// the player, but should ideally not stand too close).
  VisibleAtDistance(f32),
  /// The target is reached if the "straight line" path from the agent to the
  /// target is less than the provided distance. "Straight line" path means if
  /// the agent's path goes around a corner, the distance will be computed
  /// going around the corner. This can be more computationally expensive, as
  /// the straight line path must be computed every update. Useful for agents
  /// that care about the actual walking distance to the target.
  StraightPathDistance(f32),
}

impl Agent {
  /// Creates a new agent.
  pub fn create(
    position: Vec3,
    velocity: Vec3,
    radius: f32,
    max_velocity: f32,
  ) -> Self {
    Self {
      position,
      velocity,
      radius,
      max_velocity,
      current_target: None,
      target_reached_condition: TargetReachedCondition::Distance(radius),
      current_path: None,
      current_desired_move: Vec3::ZERO,
      state: AgentState::Idle,
    }
  }

  /// Returns the desired velocity. This will only be updated if `update` was
  /// called on the associated [`crate::Archipelago`].
  pub fn get_desired_velocity(&self) -> Vec3 {
    self.current_desired_move
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
    nav_data: &NavigationData,
    next_waypoint: (usize, Vec3),
    target_waypoint: (usize, Vec3),
  ) -> bool {
    match self.target_reached_condition {
      TargetReachedCondition::Distance(distance) => {
        self.position.distance_squared(target_waypoint.1) < distance * distance
      }
      TargetReachedCondition::VisibleAtDistance(distance) => {
        next_waypoint.0 == target_waypoint.0
          && self.position.distance_squared(next_waypoint.1)
            < distance * distance
      }
      TargetReachedCondition::StraightPathDistance(distance) => 'result: {
        // Check Euclidean distance first so we don't do the expensive path
        // following if the agent is not even close.
        if self.position.distance_squared(target_waypoint.1)
          > distance * distance
        {
          break 'result false;
        }

        // If the next waypoint is the target point, then we've already
        // computed the straight line distance and it is below the limit.
        if next_waypoint.0 == target_waypoint.0 {
          break 'result true;
        }

        let mut straight_line_distance =
          self.position.distance(next_waypoint.1);
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

#[cfg(test)]
#[path = "agent_test.rs"]
mod test;

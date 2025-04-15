use bevy_ecs::component::Component;

/// The state of an agent.
///
/// This does not control an agent's state and is just used to report the
/// agent's state.
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum AgentState {
  /// The agent is idle, due to not having a target. Note this does not mean
  /// that they are motionless. An agent will still avoid nearby agents.
  #[default]
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

impl AgentState {
  /// Converts from the `landmass` state to `bevy_landmass` state.
  pub(crate) fn from_landmass(state: &landmass::AgentState) -> Self {
    match state {
      landmass::AgentState::Idle => Self::Idle,
      landmass::AgentState::ReachedTarget => Self::ReachedTarget,
      landmass::AgentState::Moving => Self::Moving,
      landmass::AgentState::AgentNotOnNavMesh => Self::AgentNotOnNavMesh,
      landmass::AgentState::TargetNotOnNavMesh => Self::TargetNotOnNavMesh,
      landmass::AgentState::NoPath => Self::NoPath,
    }
  }
}

/// The condition to consider the agent as having reached its target. When this
/// condition is satisfied, the agent will stop moving.
#[derive(Component, Clone, Copy, Debug)]
pub enum TargetReachedCondition {
  /// The target is reached if it is within the provided (Euclidean) distance
  /// of the agent. Useful if the target is surrounded by small obstacles
  /// which don't need to be navigated around (e.g. the agent just needs to
  /// be close enough to shoot at the target, which is surrounded by cover).
  /// Alternatively, if the distance is low, this can simply mean "when the
  /// agent is really close to the target".
  Distance(Option<f32>),
  /// The target is reached if it is "visible" (there is a straight line from
  /// the agent to the target), and the target is within the provided
  /// (Euclidean) distance of the agent. Useful if the agent should be able
  /// to see the target (e.g. a companion character should remain visible to
  /// the player, but should ideally not stand too close).
  VisibleAtDistance(Option<f32>),
  /// The target is reached if the "straight line" path from the agent to the
  /// target is less than the provided distance. "Straight line" path means if
  /// the agent's path goes around a corner, the distance will be computed
  /// going around the corner. This can be more computationally expensive, as
  /// the straight line path must be computed every update. Useful for agents
  /// that care about the actual walking distance to the target.
  StraightPathDistance(Option<f32>),
}

impl TargetReachedCondition {
  /// Converts from the `bevy_landmass` condition to `landmass` condition.
  pub(crate) fn to_landmass(self) -> landmass::TargetReachedCondition {
    match self {
      TargetReachedCondition::Distance(d) => {
        landmass::TargetReachedCondition::Distance(d)
      }
      TargetReachedCondition::StraightPathDistance(d) => {
        landmass::TargetReachedCondition::StraightPathDistance(d)
      }
      TargetReachedCondition::VisibleAtDistance(d) => {
        landmass::TargetReachedCondition::VisibleAtDistance(d)
      }
    }
  }
}

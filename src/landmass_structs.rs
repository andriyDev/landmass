use bevy::ecs::component::Component;

// The state of an agent.
//
// This does not control an agent's state and is just used to report the agent's
// state.
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum AgentState {
  // The agent is idle, due to not having a target. Note this does not mean
  // that they are motionless. An agent will still avoid nearby agents.
  #[default]
  Idle,
  // The agent has reached their target. The agent may resume moving if the
  // target moves or otherwise changes.
  ReachedTarget,
  // The agent has a path and is moving towards their target.
  Moving,
  // The agent is not on a nav mesh.
  AgentNotOnNavMesh,
  // The target is not on a nav mesh.
  TargetNotOnNavMesh,
  // The agent has a target but cannot find a path to it.
  NoPath,
}

impl AgentState {
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

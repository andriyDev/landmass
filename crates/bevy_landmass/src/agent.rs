use std::{marker::PhantomData, ops::Deref};

use bevy_ecs::{
  bundle::Bundle, change_detection::DetectChanges, component::Component,
  entity::Entity, query::With, system::Query, world::Ref,
};
use bevy_platform_support::collections::HashMap;
use bevy_transform::{components::Transform, helper::TransformHelper};

use crate::{
  coords::{CoordinateSystem, ThreeD, TwoD},
  AgentState, Archipelago, TargetReachedCondition, Velocity,
};
use crate::{ArchipelagoRef, NodeType};

/// A bundle to create agents. This omits the GlobalTransform component, since
/// this is commonly added in other bundles (which is redundant and can override
/// previous bundles).
#[derive(Bundle)]
pub struct AgentBundle<CS: CoordinateSystem> {
  /// The agent marker.
  pub agent: Agent<CS>,
  /// The agent's settings.
  pub settings: AgentSettings,
  /// A reference pointing to the Archipelago to associate this entity with.
  pub archipelago_ref: ArchipelagoRef<CS>,
}

pub type Agent2dBundle = AgentBundle<TwoD>;
pub type Agent3dBundle = AgentBundle<ThreeD>;

/// A marker component to create all required components for an agent.
#[derive(Component)]
#[require(Transform, Velocity<CS>, AgentTarget<CS>, AgentState, AgentDesiredVelocity<CS>)]
pub struct Agent<CS: CoordinateSystem>(PhantomData<CS>);

pub type Agent2d = Agent<TwoD>;
pub type Agent3d = Agent<ThreeD>;

impl<CS: CoordinateSystem> Default for Agent<CS> {
  fn default() -> Self {
    Self(Default::default())
  }
}

/// The settings for an agent. See [`crate::AgentBundle`] for required related
/// components.
#[derive(Component, Debug)]
pub struct AgentSettings {
  /// The radius of the agent.
  pub radius: f32,
  /// The speed the agent prefers to move at. This should often be set lower
  /// than the [`Self::max_speed`] to allow the agent to "speed up" in order to
  /// get out of another agent's way.
  pub desired_speed: f32,
  /// The max speed of an agent.
  pub max_speed: f32,
}

#[derive(Component, Default, Debug)]
pub struct AgentNodeTypeCostOverrides(HashMap<NodeType, f32>);

impl Deref for AgentNodeTypeCostOverrides {
  type Target = HashMap<NodeType, f32>;
  fn deref(&self) -> &Self::Target {
    &self.0
  }
}

impl AgentNodeTypeCostOverrides {
  /// Sets the node type cost for this agent to `cost`. Returns false if the
  /// cost is <= 0.0. Otherwise returns true.
  pub fn set_node_type_cost(&mut self, node_type: NodeType, cost: f32) -> bool {
    if cost <= 0.0 {
      return false;
    }
    self.0.insert(node_type, cost);
    true
  }

  /// Removes the override cost for `node_type`. Returns true if `node_type` was
  /// overridden, false otherwise.
  pub fn remove_override(&mut self, node_type: NodeType) -> bool {
    self.0.remove(&node_type).is_some()
  }
}

/// The current target of the entity. Note this can be set by either reinserting
/// the component, or dereferencing:
///
/// ```rust
/// # use bevy::prelude::*;
/// # use bevy_landmass::AgentTarget3d;
/// fn clear_targets(mut targets: Query<&mut AgentTarget3d>) {
///   for mut target in targets.iter_mut() {
///     *target = AgentTarget3d::None;
///   }
/// }
/// ```
#[derive(Component)]
pub enum AgentTarget<CS: CoordinateSystem> {
  None,
  Point(CS::Coordinate),
  Entity(Entity),
}

pub type AgentTarget2d = AgentTarget<TwoD>;
pub type AgentTarget3d = AgentTarget<ThreeD>;

impl<CS: CoordinateSystem> Default for AgentTarget<CS> {
  fn default() -> Self {
    Self::None
  }
}

impl<CS: CoordinateSystem<Coordinate: std::fmt::Debug>> std::fmt::Debug
  for AgentTarget<CS>
{
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      Self::None => write!(f, "None"),
      Self::Point(arg0) => f.debug_tuple("Point").field(arg0).finish(),
      Self::Entity(arg0) => f.debug_tuple("Entity").field(arg0).finish(),
    }
  }
}

impl<CS: CoordinateSystem<Coordinate: PartialEq>> PartialEq
  for AgentTarget<CS>
{
  fn eq(&self, other: &Self) -> bool {
    match (self, other) {
      (Self::Point(l0), Self::Point(r0)) => l0 == r0,
      (Self::Entity(l0), Self::Entity(r0)) => l0 == r0,
      _ => core::mem::discriminant(self) == core::mem::discriminant(other),
    }
  }
}

impl<CS: CoordinateSystem<Coordinate: Eq>> Eq for AgentTarget<CS> {}

impl<CS: CoordinateSystem> AgentTarget<CS> {
  /// Converts an agent target to a concrete world position.
  fn to_point(
    &self,
    transform_helper: &TransformHelper,
  ) -> Option<CS::Coordinate> {
    match self {
      Self::Point(point) => Some(point.clone()),
      &Self::Entity(entity) => transform_helper
        .compute_global_transform(entity)
        .ok()
        .map(|transform| CS::from_bevy_position(transform.translation())),
      _ => None,
    }
  }
}

/// The current desired velocity of the agent. This is set by `landmass` (during
/// [`LandmassSystemSet::Output`]).
#[derive(Component)]
pub struct AgentDesiredVelocity<CS: CoordinateSystem>(CS::Coordinate);

pub type AgentDesiredVelocity2d = AgentDesiredVelocity<TwoD>;
pub type AgentDesiredVelocity3d = AgentDesiredVelocity<ThreeD>;

impl<CS: CoordinateSystem> Default for AgentDesiredVelocity<CS> {
  fn default() -> Self {
    Self(Default::default())
  }
}

impl<CS: CoordinateSystem<Coordinate: std::fmt::Debug>> std::fmt::Debug
  for AgentDesiredVelocity<CS>
{
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.debug_tuple("AgentDesiredVelocity").field(&self.0).finish()
  }
}

impl<CS: CoordinateSystem> AgentDesiredVelocity<CS> {
  /// The desired velocity of the agent.
  pub fn velocity(&self) -> CS::Coordinate {
    self.0.clone()
  }
}

#[cfg(feature = "debug-avoidance")]
/// If inserted on an agent, it will record avoidance data that can later be
/// visualized with [`crate::debug::draw_avoidance_data`].
#[derive(Component, Clone, Copy, Debug)]
pub struct KeepAvoidanceData;

/// Ensures every Bevy agent has a corresponding `landmass` agent.
pub(crate) fn add_agents_to_archipelagos<CS: CoordinateSystem>(
  mut archipelago_query: Query<(Entity, &mut Archipelago<CS>)>,
  agent_query: Query<
    (Entity, &AgentSettings, &ArchipelagoRef<CS>),
    With<Transform>,
  >,
) {
  let mut archipelago_to_agents = HashMap::<_, HashMap<_, _>>::default();
  for (entity, agent, archipleago_ref) in agent_query.iter() {
    archipelago_to_agents
      .entry(archipleago_ref.entity)
      .or_default()
      .insert(entity, agent);
  }

  for (archipelago_entity, mut archipelago) in archipelago_query.iter_mut() {
    let mut new_agent_map = archipelago_to_agents
      .remove(&archipelago_entity)
      .unwrap_or_else(HashMap::default);
    let archipelago = archipelago.as_mut();

    // Remove any agents that aren't in the `new_agent_map`. Also remove any
    // agents from the `new_agent_map` that are in the archipelago.
    archipelago.agents.retain(|agent_entity, agent_id| {
      match new_agent_map.remove(agent_entity) {
        None => {
          archipelago.archipelago.remove_agent(*agent_id);
          archipelago.reverse_agents.remove(agent_id);
          false
        }
        Some(_) => true,
      }
    });

    for (new_agent_entity, new_agent) in new_agent_map.drain() {
      let agent_id =
        archipelago.archipelago.add_agent(landmass::Agent::create(
          /* position= */ CS::from_landmass(&landmass::Vec3::ZERO),
          /* velocity= */ CS::from_landmass(&landmass::Vec3::ZERO),
          new_agent.radius,
          new_agent.desired_speed,
          new_agent.max_speed,
        ));
      archipelago.agents.insert(new_agent_entity, agent_id);
      archipelago.reverse_agents.insert(agent_id, new_agent_entity);
    }
  }
}

#[cfg(feature = "debug-avoidance")]
type HasKeepAvoidanceData = bevy::prelude::Has<KeepAvoidanceData>;
#[cfg(not(feature = "debug-avoidance"))]
type HasKeepAvoidanceData = ();

/// Ensures the "input state" (position, velocity, etc) of every Bevy agent
/// matches its `landmass` counterpart.
pub(crate) fn sync_agent_input_state<CS: CoordinateSystem>(
  agent_query: Query<
    (
      Entity,
      &AgentSettings,
      &ArchipelagoRef<CS>,
      Option<&Velocity<CS>>,
      Option<&AgentTarget<CS>>,
      Option<&TargetReachedCondition>,
      Option<Ref<AgentNodeTypeCostOverrides>>,
      HasKeepAvoidanceData,
    ),
    With<Transform>,
  >,
  transform_helper: TransformHelper,
  mut archipelago_query: Query<&mut Archipelago<CS>>,
) {
  for (
    agent_entity,
    agent,
    &ArchipelagoRef { entity: arch_entity, .. },
    velocity,
    target,
    target_reached_condition,
    node_type_cost_overrides,
    keep_avoidance_data,
  ) in agent_query.iter()
  {
    let mut archipelago = match archipelago_query.get_mut(arch_entity) {
      Err(_) => continue,
      Ok(arch) => arch,
    };

    let Ok(transform) = transform_helper.compute_global_transform(agent_entity)
    else {
      continue;
    };

    let landmass_agent = archipelago
      .get_agent_mut(agent_entity)
      .expect("this agent is in the archipelago");
    landmass_agent.position = CS::from_bevy_position(transform.translation());
    if let Some(Velocity { velocity }) = velocity {
      landmass_agent.velocity = velocity.clone();
    }
    landmass_agent.radius = agent.radius;
    landmass_agent.desired_speed = agent.desired_speed;
    landmass_agent.max_speed = agent.max_speed;
    landmass_agent.current_target =
      target.and_then(|target| target.to_point(&transform_helper));
    landmass_agent.target_reached_condition =
      if let Some(target_reached_condition) = target_reached_condition {
        target_reached_condition.to_landmass()
      } else {
        landmass::TargetReachedCondition::Distance(None)
      };
    match node_type_cost_overrides {
      None => {
        for (node_type, _) in
          landmass_agent.get_node_type_cost_overrides().collect::<Vec<_>>()
        {
          landmass_agent.remove_overridden_node_type_cost(node_type);
        }
      }
      Some(node_type_cost_overrides) => {
        if !node_type_cost_overrides.is_changed() {
          continue;
        }

        for (node_type, _) in
          landmass_agent.get_node_type_cost_overrides().collect::<Vec<_>>()
        {
          if node_type_cost_overrides.0.contains_key(&node_type) {
            continue;
          }
          landmass_agent.remove_overridden_node_type_cost(node_type);
        }

        for (&node_type, &cost) in node_type_cost_overrides.0.iter() {
          assert!(landmass_agent.override_node_type_cost(node_type, cost));
        }
      }
    }
    #[cfg(feature = "debug-avoidance")]
    {
      landmass_agent.keep_avoidance_data = keep_avoidance_data;
    }
    #[cfg(not(feature = "debug-avoidance"))]
    #[expect(clippy::let_unit_value)]
    let _ = keep_avoidance_data;
  }
}

/// Copies the agent state from `landmass` agents to their Bevy equivalent.
pub(crate) fn sync_agent_state<CS: CoordinateSystem>(
  mut agent_query: Query<
    (Entity, &ArchipelagoRef<CS>, &mut AgentState),
    With<AgentSettings>,
  >,
  archipelago_query: Query<&Archipelago<CS>>,
) {
  for (agent_entity, &ArchipelagoRef { entity: arch_entity, .. }, mut state) in
    agent_query.iter_mut()
  {
    let archipelago = match archipelago_query.get(arch_entity).ok() {
      None => continue,
      Some(arch) => arch,
    };

    *state = AgentState::from_landmass(
      &archipelago
        .get_agent(agent_entity)
        .expect("the agent is in the archipelago")
        .state(),
    );
  }
}

/// Copies the agent desired velocity from `landmass` agents to their Bevy
/// equivalent.
pub(crate) fn sync_desired_velocity<CS: CoordinateSystem>(
  mut agent_query: Query<
    (Entity, &ArchipelagoRef<CS>, &mut AgentDesiredVelocity<CS>),
    With<AgentSettings>,
  >,
  archipelago_query: Query<&Archipelago<CS>>,
) {
  for (
    agent_entity,
    &ArchipelagoRef { entity: arch_entity, .. },
    mut desired_velocity,
  ) in agent_query.iter_mut()
  {
    let archipelago = match archipelago_query.get(arch_entity).ok() {
      None => continue,
      Some(arch) => arch,
    };

    desired_velocity.0 = archipelago
      .get_agent(agent_entity)
      .expect("the agent is in the archipelago")
      .get_desired_velocity()
      .clone();
  }
}

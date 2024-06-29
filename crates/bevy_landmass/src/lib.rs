#![doc = include_str!("../README.md")]

use std::{
  collections::{HashMap, HashSet},
  sync::Arc,
};

use bevy::{
  asset::{Asset, AssetApp, Assets, Handle},
  prelude::{
    Bundle, Component, Entity, EulerRot, GlobalTransform, IntoSystemConfigs,
    IntoSystemSetConfigs, Plugin, Query, Res, SystemSet, Update, With,
  },
  reflect::TypePath,
  time::Time,
};
use landmass::{AgentId, IslandId};
use util::{bevy_vec3_to_landmass_vec3, landmass_vec3_to_bevy_vec3};

mod landmass_structs;
mod util;

pub use landmass::NavigationMesh;
pub use landmass::ValidNavigationMesh;
pub use landmass::ValidationError;
pub use landmass::Vec3;

pub use landmass_structs::*;

pub mod debug;

#[cfg(feature = "mesh-utils")]
pub mod nav_mesh;

pub struct LandmassPlugin;

pub mod prelude {
  pub use crate::Agent;
  pub use crate::AgentBundle;
  pub use crate::AgentDesiredVelocity;
  pub use crate::AgentState;
  pub use crate::AgentTarget;
  pub use crate::AgentVelocity;
  pub use crate::Archipelago;
  pub use crate::ArchipelagoRef;
  pub use crate::Island;
  pub use crate::IslandBundle;
  pub use crate::LandmassPlugin;
  pub use crate::LandmassSystemSet;
  pub use crate::NavMesh;
  pub use crate::NavigationMesh;
  pub use crate::ValidNavigationMesh;
}

/// A bundle to create agents. This omits the GlobalTransform component, since
/// this is commonly added in other bundles (which is redundant and can override
/// previous bundles).
#[derive(Bundle)]
pub struct AgentBundle {
  /// The agent itself.
  pub agent: Agent,
  /// A reference pointing to the Archipelago to associate this entity with.
  pub archipelago_ref: ArchipelagoRef,
  /// The velocity of the agent.
  pub velocity: AgentVelocity,
  /// The target of the agent.
  pub target: AgentTarget,
  /// The current state of the agent. This is set by `landmass` (during
  /// [`LandmassSystemSet::Output`]).
  pub state: AgentState,
  /// The current desired velocity of the agent. This is set by `landmass`
  /// (during [`LandmassSystemSet::Output`]).
  pub desired_velocity: AgentDesiredVelocity,
}

/// A bundle to create islands. The GlobalTransform component is omitted, since
/// this is commonly added in other bundles (which is redundant and can
/// override previous bundles).
#[derive(Bundle)]
pub struct IslandBundle {
  /// An island marker component.
  pub island: Island,
  /// A reference pointing to the Archipelago to associate this entity with.
  pub archipelago_ref: ArchipelagoRef,
  /// A handle to the nav mesh that this island needs.
  pub nav_mesh: Handle<NavMesh>,
}

/// System set for `landmass` systems.
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum LandmassSystemSet {
  /// Systems for syncing the existence of components with the internal
  /// `landmass` state. Ensure your `landmass` entities are setup before this
  /// point (and not removed until [`LandmassSystemSet::Output`]).
  SyncExistence,
  /// Systems for syncing the values of components with the internal `landmass`
  /// state.
  SyncValues,
  /// The actual `landmass` updating step.
  Update,
  /// Systems for returning the output of `landmass` back to users. Avoid
  /// reading/mutating data from your `landmass` entities until after this
  /// point.
  Output,
}

impl Plugin for LandmassPlugin {
  fn build(&self, app: &mut bevy::prelude::App) {
    app.init_asset::<NavMesh>();
    app.configure_sets(
      Update,
      (
        LandmassSystemSet::SyncExistence.before(LandmassSystemSet::SyncValues),
        LandmassSystemSet::SyncValues.before(LandmassSystemSet::Update),
        LandmassSystemSet::Update.before(LandmassSystemSet::Output),
      ),
    );
    app.add_systems(
      Update,
      (add_agents_to_archipelagos, add_islands_to_archipelago)
        .in_set(LandmassSystemSet::SyncExistence),
    );
    app.add_systems(
      Update,
      (sync_agent_input_state, sync_island_nav_mesh)
        .in_set(LandmassSystemSet::SyncValues),
    );
    app.add_systems(
      Update,
      update_archipelagos.in_set(LandmassSystemSet::Update),
    );
    app.add_systems(
      Update,
      (sync_agent_state, sync_desired_velocity)
        .in_set(LandmassSystemSet::Output),
    );
  }
}

/// An archipelago, holding the internal state of `landmass`.
#[derive(Component)]
pub struct Archipelago {
  /// The `landmass` archipelago.
  archipelago: landmass::Archipelago,
  /// A map from the Bevy entity to its associated island ID in
  /// [`Archipelago::archipelago`].
  islands: HashMap<Entity, IslandId>,
  /// A map from the Bevy entity to its associated agent ID in
  /// [`Archipelago::archipelago`].
  agents: HashMap<Entity, AgentId>,
}

impl Archipelago {
  /// Creates an empty archipelago.
  pub fn new() -> Self {
    Self {
      archipelago: landmass::Archipelago::new(),
      islands: HashMap::new(),
      agents: HashMap::new(),
    }
  }

  /// Gets the agent options.
  pub fn get_agent_options(&self) -> &landmass::AgentOptions {
    &self.archipelago.agent_options
  }

  /// Gets a mutable borrow to the agent options.
  pub fn get_agent_options_mut(&mut self) -> &mut landmass::AgentOptions {
    &mut self.archipelago.agent_options
  }

  /// Gets an agent.
  fn get_agent(&self, entity: Entity) -> &landmass::Agent {
    self.archipelago.get_agent(*self.agents.get(&entity).unwrap())
  }

  /// Gets a mutable borrow to an agent.
  fn get_agent_mut(&mut self, entity: Entity) -> &mut landmass::Agent {
    self.archipelago.get_agent_mut(*self.agents.get(&entity).unwrap())
  }

  /// Gets a mutable borrow to an island (if present).
  fn get_island_mut(
    &mut self,
    entity: Entity,
  ) -> Option<&mut landmass::Island> {
    self
      .islands
      .get(&entity)
      .map(|island_id| self.archipelago.get_island_mut(*island_id))
  }
}

impl Default for Archipelago {
  fn default() -> Self {
    Self::new()
  }
}

/// Updates the archipelago.
fn update_archipelagos(
  time: Res<Time>,
  mut archipelago_query: Query<&mut Archipelago>,
) {
  if time.delta_seconds() == 0.0 {
    return;
  }
  for mut archipelago in archipelago_query.iter_mut() {
    archipelago.archipelago.update(time.delta_seconds());
  }
}

/// A marker component that an entity is an island.
#[derive(Component)]
pub struct Island;

/// An asset holding a `landmass` nav mesh.
#[derive(Asset, TypePath)]
pub struct NavMesh(pub Arc<ValidNavigationMesh>);

/// Ensures every Bevy island has a corresponding `landmass` island.
fn add_islands_to_archipelago(
  mut archipelago_query: Query<(Entity, &mut Archipelago)>,
  island_query: Query<(Entity, &ArchipelagoRef), With<Island>>,
) {
  let mut archipelago_to_islands = HashMap::<_, HashSet<_>>::new();
  for (entity, archipleago_ref) in island_query.iter() {
    archipelago_to_islands.entry(archipleago_ref.0).or_default().insert(entity);
  }

  for (archipelago_entity, mut archipelago) in archipelago_query.iter_mut() {
    let mut new_islands = archipelago_to_islands
      .remove(&archipelago_entity)
      .unwrap_or_else(HashSet::new);
    let archipelago = archipelago.as_mut();

    // Remove any islands that aren't in the `new_islands`. Also remove any
    // islands from the `new_islands` that are in the archipelago.
    archipelago.islands.retain(|island_entity, island_id| {
      match new_islands.remove(island_entity) {
        false => {
          archipelago.archipelago.remove_island(*island_id);
          false
        }
        true => true,
      }
    });

    for new_island_entity in new_islands.drain() {
      let island_id = archipelago.archipelago.add_island();
      archipelago.islands.insert(new_island_entity, island_id);
    }
  }
}

/// Ensures that the island transform and nav mesh are up to date.
fn sync_island_nav_mesh(
  mut archipelago_query: Query<&mut Archipelago>,
  island_query: Query<
    (
      Entity,
      Option<&Handle<NavMesh>>,
      Option<&GlobalTransform>,
      &ArchipelagoRef,
    ),
    With<Island>,
  >,
  nav_meshes: Res<Assets<NavMesh>>,
) {
  for (island_entity, island_nav_mesh, island_transform, archipelago_ref) in
    island_query.iter()
  {
    let mut archipelago = match archipelago_query.get_mut(archipelago_ref.0) {
      Err(_) => continue,
      Ok(arch) => arch,
    };

    let landmass_island = match archipelago.get_island_mut(island_entity) {
      None => continue,
      Some(island) => island,
    };

    let island_nav_mesh = match island_nav_mesh {
      None => {
        if landmass_island.get_nav_mesh().is_some() {
          landmass_island.clear_nav_mesh();
        }
        continue;
      }
      Some(nav_mesh) => nav_mesh,
    };

    let island_nav_mesh = match nav_meshes.get(island_nav_mesh) {
      None => {
        if landmass_island.get_nav_mesh().is_some() {
          landmass_island.clear_nav_mesh();
        }
        continue;
      }
      Some(nav_mesh) => nav_mesh,
    };

    let island_transform = match island_transform {
      None => {
        if landmass_island.get_nav_mesh().is_some() {
          landmass_island.clear_nav_mesh();
        }
        continue;
      }
      Some(transform) => {
        let transform = transform.compute_transform();
        landmass::Transform {
          translation: bevy_vec3_to_landmass_vec3(transform.translation),
          rotation: transform.rotation.to_euler(EulerRot::YXZ).0,
        }
      }
    };

    let set_nav_mesh = match landmass_island
      .get_transform()
      .map(|transform| (transform, landmass_island.get_nav_mesh().unwrap()))
    {
      None => true,
      Some((current_transform, current_nav_mesh)) => {
        current_transform != island_transform
          || !Arc::ptr_eq(&current_nav_mesh, &island_nav_mesh.0)
      }
    };

    if set_nav_mesh {
      landmass_island
        .set_nav_mesh(island_transform, Arc::clone(&island_nav_mesh.0));
    }
  }
}

/// An agent. See [`crate::AgentBundle`] for required related components.
#[derive(Component)]
pub struct Agent {
  /// The radius of the agent.
  pub radius: f32,
  /// The max velocity of an agent.
  pub max_velocity: f32,
}

/// A reference to an archipelago.
#[derive(Component)]
pub struct ArchipelagoRef(pub Entity);

/// The current velocity of the agent. This must be set to match whatever speed
/// the agent is going.
#[derive(Component, Default)]
pub struct AgentVelocity(pub Vec3);

/// The current target of the entity. Note this can be set by either reinserting
/// the component, or dereferencing:
///
/// ```rust
/// use bevy::prelude::*;
/// use bevy_landmass::AgentTarget;
///
/// fn clear_targets(mut targets: Query<&mut AgentTarget>) {
///   for mut target in targets.iter_mut() {
///     *target = AgentTarget::None;
///   }
/// }
/// ```
#[derive(Component, Default)]
pub enum AgentTarget {
  #[default]
  None,
  Point(Vec3),
  Entity(Entity),
}

impl AgentTarget {
  /// Converts an agent target to a concrete world position.
  fn to_point(
    &self,
    global_transform_query: &Query<&GlobalTransform>,
  ) -> Option<Vec3> {
    match *self {
      Self::Point(point) => Some(point),
      Self::Entity(entity) => global_transform_query
        .get(entity)
        .ok()
        .map(|transform| transform.translation()),
      _ => None,
    }
  }
}

/// The current desired velocity of the agent. This is set by `landmass` (during
/// [`LandmassSystemSet::Output`]).
#[derive(Component, Default)]
pub struct AgentDesiredVelocity(Vec3);

impl AgentDesiredVelocity {
  /// The desired velocity of the agent.
  pub fn velocity(&self) -> Vec3 {
    self.0
  }
}

/// Ensures every Bevy agent has a corresponding `landmass` agent.
fn add_agents_to_archipelagos(
  mut archipelago_query: Query<(Entity, &mut Archipelago)>,
  agent_query: Query<(Entity, &Agent, &ArchipelagoRef), With<GlobalTransform>>,
) {
  let mut archipelago_to_agents = HashMap::<_, HashMap<_, _>>::new();
  for (entity, agent, archipleago_ref) in agent_query.iter() {
    archipelago_to_agents
      .entry(archipleago_ref.0)
      .or_default()
      .insert(entity, agent);
  }

  for (archipelago_entity, mut archipelago) in archipelago_query.iter_mut() {
    let mut new_agent_map = archipelago_to_agents
      .remove(&archipelago_entity)
      .unwrap_or_else(HashMap::new);
    let archipelago = archipelago.as_mut();

    // Remove any agents that aren't in the `new_agent_map`. Also remove any
    // agents from the `new_agent_map` that are in the archipelago.
    archipelago.agents.retain(|agent_entity, agent_id| {
      match new_agent_map.remove(agent_entity) {
        None => {
          archipelago.archipelago.remove_agent(*agent_id);
          false
        }
        Some(_) => true,
      }
    });

    for (new_agent_entity, new_agent) in new_agent_map.drain() {
      let agent_id =
        archipelago.archipelago.add_agent(landmass::Agent::create(
          /* position= */ landmass::Vec3::ZERO,
          /* velocity= */ landmass::Vec3::ZERO,
          new_agent.radius,
          new_agent.max_velocity,
        ));
      archipelago.agents.insert(new_agent_entity, agent_id);
    }
  }
}

/// Ensures the "input state" (position, velocity, etc) of every Bevy agent
/// matches its `landmass` counterpart.
fn sync_agent_input_state(
  agent_query: Query<
    (
      Entity,
      &ArchipelagoRef,
      &GlobalTransform,
      Option<&AgentVelocity>,
      Option<&AgentTarget>,
      Option<&TargetReachedCondition>,
    ),
    With<Agent>,
  >,
  global_transform_query: Query<&GlobalTransform>,
  mut archipelago_query: Query<&mut Archipelago>,
) {
  for (
    agent_entity,
    &ArchipelagoRef(arch_entity),
    transform,
    velocity,
    target,
    target_reached_condition,
  ) in agent_query.iter()
  {
    let mut archipelago = match archipelago_query.get_mut(arch_entity) {
      Err(_) => continue,
      Ok(arch) => arch,
    };

    let agent = archipelago.get_agent_mut(agent_entity);
    agent.position = bevy_vec3_to_landmass_vec3(transform.translation());
    if let Some(AgentVelocity(velocity)) = velocity {
      agent.velocity = bevy_vec3_to_landmass_vec3(*velocity);
    }
    agent.current_target = target
      .and_then(|target| target.to_point(&global_transform_query))
      .map(bevy_vec3_to_landmass_vec3);
    if let Some(target_reached_condition) = target_reached_condition {
      agent.target_reached_condition = target_reached_condition.to_landmass();
    } else {
      agent.target_reached_condition =
        landmass::TargetReachedCondition::Distance(None);
    }
  }
}

/// Copies the agent state from `landmass` agents to their Bevy equivalent.
fn sync_agent_state(
  mut agent_query: Query<
    (Entity, &ArchipelagoRef, &mut AgentState),
    With<Agent>,
  >,
  archipelago_query: Query<&Archipelago>,
) {
  for (agent_entity, &ArchipelagoRef(arch_entity), mut state) in
    agent_query.iter_mut()
  {
    let archipelago = match archipelago_query.get(arch_entity).ok() {
      None => continue,
      Some(arch) => arch,
    };

    *state =
      AgentState::from_landmass(&archipelago.get_agent(agent_entity).state());
  }
}

/// Copies the agent desired velocity from `landmass` agents to their Bevy
/// equivalent.
fn sync_desired_velocity(
  mut agent_query: Query<
    (Entity, &ArchipelagoRef, &mut AgentDesiredVelocity),
    With<Agent>,
  >,
  archipelago_query: Query<&Archipelago>,
) {
  for (agent_entity, &ArchipelagoRef(arch_entity), mut desired_velocity) in
    agent_query.iter_mut()
  {
    let archipelago = match archipelago_query.get(arch_entity).ok() {
      None => continue,
      Some(arch) => arch,
    };

    desired_velocity.0 = landmass_vec3_to_bevy_vec3(
      archipelago.get_agent(agent_entity).get_desired_velocity(),
    );
  }
}

#[cfg(test)]
#[path = "lib_test.rs"]
mod test;

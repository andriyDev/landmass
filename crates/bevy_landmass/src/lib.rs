#![doc = include_str!("../README.md")]

use std::{
  collections::{HashMap, HashSet},
  marker::PhantomData,
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
use coords::{CoordinateSystem, ThreeD, TwoD};
use landmass::{AgentId, CharacterId, IslandId};

mod landmass_structs;

pub use landmass::AgentOptions;
pub use landmass::NavigationMesh;
pub use landmass::ValidNavigationMesh;
pub use landmass::ValidationError;

pub use landmass_structs::*;

pub mod coords;
pub mod debug;

#[cfg(feature = "mesh-utils")]
pub mod nav_mesh;

pub struct LandmassPlugin<CS: CoordinateSystem>(PhantomData<CS>);

impl<CS: CoordinateSystem> Default for LandmassPlugin<CS> {
  fn default() -> Self {
    Self(Default::default())
  }
}

pub type Landmass2dPlugin = LandmassPlugin<TwoD>;
pub type Landmass3dPlugin = LandmassPlugin<ThreeD>;

pub mod prelude {
  pub use crate::coords::CoordinateSystem;
  pub use crate::coords::ThreeD;
  pub use crate::coords::TwoD;
  pub use crate::Agent;
  pub use crate::Agent2dBundle;
  pub use crate::Agent3dBundle;
  pub use crate::AgentDesiredVelocity2d;
  pub use crate::AgentDesiredVelocity3d;
  pub use crate::AgentState;
  pub use crate::AgentTarget2d;
  pub use crate::AgentTarget3d;
  pub use crate::Archipelago2d;
  pub use crate::Archipelago3d;
  pub use crate::ArchipelagoRef2d;
  pub use crate::ArchipelagoRef3d;
  pub use crate::Island;
  pub use crate::Island2dBundle;
  pub use crate::Island3dBundle;
  pub use crate::Landmass2dPlugin;
  pub use crate::Landmass3dPlugin;
  pub use crate::LandmassSystemSet;
  pub use crate::NavMesh2d;
  pub use crate::NavMesh3d;
  pub use crate::NavigationMesh2d;
  pub use crate::NavigationMesh3d;
  pub use crate::ValidNavigationMesh2d;
  pub use crate::ValidNavigationMesh3d;
  pub use crate::Velocity2d;
  pub use crate::Velocity3d;
}

pub type NavigationMesh2d = NavigationMesh<TwoD>;
pub type NavigationMesh3d = NavigationMesh<ThreeD>;

pub type ValidNavigationMesh2d = ValidNavigationMesh<TwoD>;
pub type ValidNavigationMesh3d = ValidNavigationMesh<ThreeD>;

/// A bundle to create agents. This omits the GlobalTransform component, since
/// this is commonly added in other bundles (which is redundant and can override
/// previous bundles).
#[derive(Bundle)]
pub struct AgentBundle<CS: CoordinateSystem> {
  /// The agent itself.
  pub agent: Agent,
  /// A reference pointing to the Archipelago to associate this entity with.
  pub archipelago_ref: ArchipelagoRef<CS>,
  /// The velocity of the agent.
  pub velocity: Velocity<CS>,
  /// The target of the agent.
  pub target: AgentTarget<CS>,
  /// The current state of the agent. This is set by `landmass` (during
  /// [`LandmassSystemSet::Output`]).
  pub state: AgentState,
  /// The current desired velocity of the agent. This is set by `landmass`
  /// (during [`LandmassSystemSet::Output`]).
  pub desired_velocity: AgentDesiredVelocity<CS>,
}

pub type Agent2dBundle = AgentBundle<TwoD>;
pub type Agent3dBundle = AgentBundle<ThreeD>;

/// A bundle to create characters. This omits the GlobalTransform component,
/// since this is commonly added in other bundles (which is redundant and can
/// override previous bundles).
#[derive(Bundle)]
pub struct CharacterBundle<CS: CoordinateSystem> {
  /// The character itself.
  pub character: Character,
  /// A reference pointing to the Archipelago to associate this entity with.
  pub archipelago_ref: ArchipelagoRef<CS>,
  /// The velocity of the character.
  pub velocity: Velocity<CS>,
}

pub type Character2dBundle = CharacterBundle<TwoD>;
pub type Character3dBundle = CharacterBundle<ThreeD>;

/// A bundle to create islands. The GlobalTransform component is omitted, since
/// this is commonly added in other bundles (which is redundant and can
/// override previous bundles).
#[derive(Bundle)]
pub struct IslandBundle<CS: CoordinateSystem> {
  /// An island marker component.
  pub island: Island,
  /// A reference pointing to the Archipelago to associate this entity with.
  pub archipelago_ref: ArchipelagoRef<CS>,
  /// A handle to the nav mesh that this island needs.
  pub nav_mesh: Handle<NavMesh<CS>>,
}

pub type Island2dBundle = IslandBundle<TwoD>;
pub type Island3dBundle = IslandBundle<ThreeD>;

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

impl<CS: CoordinateSystem> Plugin for LandmassPlugin<CS> {
  fn build(&self, app: &mut bevy::prelude::App) {
    app.init_asset::<NavMesh<CS>>();
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
      (
        add_agents_to_archipelagos::<CS>,
        add_islands_to_archipelago::<CS>,
        add_characters_to_archipelago::<CS>,
      )
        .in_set(LandmassSystemSet::SyncExistence),
    );
    app.add_systems(
      Update,
      (
        sync_agent_input_state::<CS>,
        sync_island_nav_mesh::<CS>,
        sync_character_state::<CS>,
      )
        .in_set(LandmassSystemSet::SyncValues),
    );
    app.add_systems(
      Update,
      update_archipelagos::<CS>.in_set(LandmassSystemSet::Update),
    );
    app.add_systems(
      Update,
      (sync_agent_state::<CS>, sync_desired_velocity::<CS>)
        .in_set(LandmassSystemSet::Output),
    );
  }
}

/// An archipelago, holding the internal state of `landmass`.
#[derive(Component)]
pub struct Archipelago<CS: CoordinateSystem> {
  /// The `landmass` archipelago.
  archipelago: landmass::Archipelago<CS>,
  /// A map from the Bevy entity to its associated island ID in
  /// [`Archipelago::archipelago`].
  islands: HashMap<Entity, IslandId>,
  /// A map from the Bevy entity to its associated agent ID in
  /// [`Archipelago::archipelago`].
  agents: HashMap<Entity, AgentId>,
  /// A map from the Bevy entity to its associated character ID in
  /// [`Archipelago::archipelago`].
  characters: HashMap<Entity, CharacterId>,
}

pub type Archipelago2d = Archipelago<TwoD>;
pub type Archipelago3d = Archipelago<ThreeD>;

impl<CS: CoordinateSystem> Archipelago<CS> {
  /// Creates an empty archipelago.
  pub fn new() -> Self {
    Self {
      archipelago: landmass::Archipelago::new(),
      islands: HashMap::new(),
      agents: HashMap::new(),
      characters: HashMap::new(),
    }
  }

  /// Gets the agent options.
  pub fn get_agent_options(&self) -> &AgentOptions {
    &self.archipelago.agent_options
  }

  /// Gets a mutable borrow to the agent options.
  pub fn get_agent_options_mut(&mut self) -> &mut AgentOptions {
    &mut self.archipelago.agent_options
  }

  /// Gets an agent.
  fn get_agent(&self, entity: Entity) -> &landmass::Agent<CS> {
    self.archipelago.get_agent(*self.agents.get(&entity).unwrap())
  }

  /// Gets a mutable borrow to an agent.
  fn get_agent_mut(&mut self, entity: Entity) -> &mut landmass::Agent<CS> {
    self.archipelago.get_agent_mut(*self.agents.get(&entity).unwrap())
  }

  /// Gets a mutable borrow to a character.
  #[allow(unused)] // Used in tests.
  fn get_character(&self, entity: Entity) -> &landmass::Character<CS> {
    self.archipelago.get_character(*self.characters.get(&entity).unwrap())
  }

  /// Gets a mutable borrow to a character.
  fn get_character_mut(
    &mut self,
    entity: Entity,
  ) -> &mut landmass::Character<CS> {
    self.archipelago.get_character_mut(*self.characters.get(&entity).unwrap())
  }

  /// Gets a mutable borrow to an island (if present).
  fn get_island_mut(
    &mut self,
    entity: Entity,
  ) -> Option<&mut landmass::Island<CS>> {
    self
      .islands
      .get(&entity)
      .map(|island_id| self.archipelago.get_island_mut(*island_id))
  }
}

impl<CS: CoordinateSystem> Default for Archipelago<CS> {
  fn default() -> Self {
    Self::new()
  }
}

/// Updates the archipelago.
fn update_archipelagos<CS: CoordinateSystem>(
  time: Res<Time>,
  mut archipelago_query: Query<&mut Archipelago<CS>>,
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
pub struct NavMesh<CS: CoordinateSystem>(pub Arc<ValidNavigationMesh<CS>>);

pub type NavMesh2d = NavMesh<TwoD>;
pub type NavMesh3d = NavMesh<ThreeD>;

/// Ensures every Bevy island has a corresponding `landmass` island.
fn add_islands_to_archipelago<CS: CoordinateSystem>(
  mut archipelago_query: Query<(Entity, &mut Archipelago<CS>)>,
  island_query: Query<(Entity, &ArchipelagoRef<CS>), With<Island>>,
) {
  let mut archipelago_to_islands = HashMap::<_, HashSet<_>>::new();
  for (entity, archipleago_ref) in island_query.iter() {
    archipelago_to_islands
      .entry(archipleago_ref.entity)
      .or_default()
      .insert(entity);
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
fn sync_island_nav_mesh<CS: CoordinateSystem>(
  mut archipelago_query: Query<&mut Archipelago<CS>>,
  island_query: Query<
    (
      Entity,
      Option<&Handle<NavMesh<CS>>>,
      Option<&GlobalTransform>,
      &ArchipelagoRef<CS>,
    ),
    With<Island>,
  >,
  nav_meshes: Res<Assets<NavMesh<CS>>>,
) {
  for (island_entity, island_nav_mesh, island_transform, archipelago_ref) in
    island_query.iter()
  {
    let mut archipelago =
      match archipelago_query.get_mut(archipelago_ref.entity) {
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
          translation: CS::from_transform_position(transform.translation),
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
        current_transform != &island_transform
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

/// A character. See [`crate::CharacterBundle`] for required related components.
#[derive(Component)]
pub struct Character {
  /// The radius of the character.
  pub radius: f32,
}

/// A reference to an archipelago.
#[derive(Component)]
pub struct ArchipelagoRef<CS: CoordinateSystem> {
  pub entity: Entity,
  pub marker: PhantomData<CS>,
}

pub type ArchipelagoRef2d = ArchipelagoRef<TwoD>;
pub type ArchipelagoRef3d = ArchipelagoRef<ThreeD>;

impl<CS: CoordinateSystem> ArchipelagoRef<CS> {
  pub fn new(entity: Entity) -> Self {
    Self { entity, marker: Default::default() }
  }
}

/// The current velocity of the agent/character. This must be set to match
/// whatever speed the agent/character is going.
#[derive(Component)]
pub struct Velocity<CS: CoordinateSystem>(pub CS::Coordinate);

pub type Velocity2d = Velocity<TwoD>;
pub type Velocity3d = Velocity<ThreeD>;

impl<CS: CoordinateSystem> Default for Velocity<CS> {
  fn default() -> Self {
    Self(Default::default())
  }
}

/// The current target of the entity. Note this can be set by either reinserting
/// the component, or dereferencing:
///
/// ```rust
/// use bevy::prelude::*;
/// use bevy_landmass::AgentTarget3d;
///
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

impl<CS: CoordinateSystem> AgentTarget<CS> {
  /// Converts an agent target to a concrete world position.
  fn to_point(
    &self,
    global_transform_query: &Query<&GlobalTransform>,
  ) -> Option<CS::Coordinate> {
    match self {
      Self::Point(point) => Some(point.clone()),
      &Self::Entity(entity) => global_transform_query
        .get(entity)
        .ok()
        .map(|transform| transform.translation())
        .map(CS::from_transform_position),
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

impl<CS: CoordinateSystem> AgentDesiredVelocity<CS> {
  /// The desired velocity of the agent.
  pub fn velocity(&self) -> CS::Coordinate {
    self.0.clone()
  }
}

/// Ensures every Bevy agent has a corresponding `landmass` agent.
fn add_agents_to_archipelagos<CS: CoordinateSystem>(
  mut archipelago_query: Query<(Entity, &mut Archipelago<CS>)>,
  agent_query: Query<
    (Entity, &Agent, &ArchipelagoRef<CS>),
    With<GlobalTransform>,
  >,
) {
  let mut archipelago_to_agents = HashMap::<_, HashMap<_, _>>::new();
  for (entity, agent, archipleago_ref) in agent_query.iter() {
    archipelago_to_agents
      .entry(archipleago_ref.entity)
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
          /* position= */ CS::from_landmass(&landmass::Vec3::ZERO),
          /* velocity= */ CS::from_landmass(&landmass::Vec3::ZERO),
          new_agent.radius,
          new_agent.max_velocity,
        ));
      archipelago.agents.insert(new_agent_entity, agent_id);
    }
  }
}

/// Ensures the "input state" (position, velocity, etc) of every Bevy agent
/// matches its `landmass` counterpart.
fn sync_agent_input_state<CS: CoordinateSystem>(
  agent_query: Query<(
    Entity,
    &Agent,
    &ArchipelagoRef<CS>,
    &GlobalTransform,
    Option<&Velocity<CS>>,
    Option<&AgentTarget<CS>>,
    Option<&TargetReachedCondition>,
  )>,
  global_transform_query: Query<&GlobalTransform>,
  mut archipelago_query: Query<&mut Archipelago<CS>>,
) {
  for (
    agent_entity,
    agent,
    &ArchipelagoRef { entity: arch_entity, .. },
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

    let landmass_agent = archipelago.get_agent_mut(agent_entity);
    landmass_agent.position =
      CS::from_transform_position(transform.translation());
    if let Some(Velocity(velocity)) = velocity {
      landmass_agent.velocity = velocity.clone();
    }
    landmass_agent.radius = agent.radius;
    landmass_agent.max_velocity = agent.max_velocity;
    landmass_agent.current_target =
      target.and_then(|target| target.to_point(&global_transform_query));
    landmass_agent.target_reached_condition =
      if let Some(target_reached_condition) = target_reached_condition {
        target_reached_condition.to_landmass()
      } else {
        landmass::TargetReachedCondition::Distance(None)
      };
  }
}

/// Copies the agent state from `landmass` agents to their Bevy equivalent.
fn sync_agent_state<CS: CoordinateSystem>(
  mut agent_query: Query<
    (Entity, &ArchipelagoRef<CS>, &mut AgentState),
    With<Agent>,
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

    *state =
      AgentState::from_landmass(&archipelago.get_agent(agent_entity).state());
  }
}

/// Copies the agent desired velocity from `landmass` agents to their Bevy
/// equivalent.
fn sync_desired_velocity<CS: CoordinateSystem>(
  mut agent_query: Query<
    (Entity, &ArchipelagoRef<CS>, &mut AgentDesiredVelocity<CS>),
    With<Agent>,
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

    desired_velocity.0 =
      archipelago.get_agent(agent_entity).get_desired_velocity().clone();
  }
}

/// Ensures every Bevy character has a corresponding `landmass` character.
fn add_characters_to_archipelago<CS: CoordinateSystem>(
  mut archipelagos: Query<(Entity, &mut Archipelago<CS>)>,
  characters: Query<
    (Entity, &Character, &ArchipelagoRef<CS>),
    With<GlobalTransform>,
  >,
) {
  let mut archipelago_to_characters = HashMap::<_, HashMap<_, _>>::new();
  for (entity, character, archipleago_ref) in characters.iter() {
    archipelago_to_characters
      .entry(archipleago_ref.entity)
      .or_default()
      .insert(entity, character);
  }
  for (entity, mut archipelago) in archipelagos.iter_mut() {
    let mut new_character_map =
      archipelago_to_characters.remove(&entity).unwrap_or_else(HashMap::new);
    let archipelago = archipelago.as_mut();

    archipelago.characters.retain(|character_entity, character_id| {
      if new_character_map.remove(character_entity).is_none() {
        archipelago.archipelago.remove_character(*character_id);
        false
      } else {
        true
      }
    });

    for (new_character_entity, new_character) in new_character_map.drain() {
      let character_id =
        archipelago.archipelago.add_character(landmass::Character {
          position: Default::default(),
          velocity: Default::default(),
          radius: new_character.radius,
        });
      archipelago.characters.insert(new_character_entity, character_id);
    }
  }
}

/// Copies Bevy character states to their associated landmass character.
fn sync_character_state<CS: CoordinateSystem>(
  characters: Query<(
    Entity,
    &Character,
    &ArchipelagoRef<CS>,
    &GlobalTransform,
    Option<&Velocity<CS>>,
  )>,
  mut archipelagos: Query<&mut Archipelago<CS>>,
) {
  for (
    character_entity,
    character,
    &ArchipelagoRef { entity: arch_entity, .. },
    transform,
    velocity,
  ) in characters.iter()
  {
    let Ok(mut archipelago) = archipelagos.get_mut(arch_entity) else {
      continue;
    };

    let landmass_character = archipelago.get_character_mut(character_entity);
    landmass_character.position =
      CS::from_transform_position(transform.translation());
    landmass_character.velocity = if let Some(Velocity(velocity)) = velocity {
      velocity.clone()
    } else {
      CS::Coordinate::default()
    };
    landmass_character.radius = character.radius;
  }
}

#[cfg(test)]
#[path = "lib_test.rs"]
mod test;

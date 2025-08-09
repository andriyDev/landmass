#![doc = include_str!("../README.md")]

use std::{collections::HashMap, marker::PhantomData, sync::Arc};

use bevy_app::{Plugin, RunFixedMainLoop, RunFixedMainLoopSystem};
use bevy_asset::{Asset, AssetApp, Handle};
use bevy_ecs::{
  component::Component,
  entity::Entity,
  intern::Interned,
  schedule::{IntoScheduleConfigs, ScheduleLabel, SystemSet},
  system::{Query, Res},
};
use bevy_reflect::TypePath;
use bevy_time::Time;
use coords::{CoordinateSystem, ThreeD, TwoD};
use landmass::{AgentId, CharacterId, IslandId};

mod agent;
mod character;
mod island;
mod landmass_structs;

pub use landmass::{
  AgentOptions, FindPathError, FromAgentRadius, HeightNavigationMesh,
  HeightPolygon, NavigationMesh, NewNodeTypeError, NodeType,
  PointSampleDistance3d, SamplePointError, SetNodeTypeCostError,
  ValidNavigationMesh, ValidationError,
};

pub use agent::*;
pub use character::*;
pub use island::*;
pub use landmass_structs::*;

pub mod coords;
pub mod debug;

#[cfg(feature = "mesh-utils")]
pub mod nav_mesh;

pub mod prelude {
  pub use crate::Agent2dBundle;
  pub use crate::Agent3dBundle;
  pub use crate::AgentDesiredVelocity2d;
  pub use crate::AgentDesiredVelocity3d;
  pub use crate::AgentOptions;
  pub use crate::AgentSettings;
  pub use crate::AgentState;
  pub use crate::AgentTarget2d;
  pub use crate::AgentTarget3d;
  pub use crate::Archipelago2d;
  pub use crate::Archipelago3d;
  pub use crate::ArchipelagoRef2d;
  pub use crate::ArchipelagoRef3d;
  pub use crate::Character2dBundle;
  pub use crate::Character3dBundle;
  pub use crate::CharacterSettings;
  pub use crate::FromAgentRadius;
  pub use crate::HeightNavigationMesh2d;
  pub use crate::HeightNavigationMesh3d;
  pub use crate::Island;
  pub use crate::Island2dBundle;
  pub use crate::Island3dBundle;
  pub use crate::Landmass2dPlugin;
  pub use crate::Landmass3dPlugin;
  pub use crate::LandmassSystemSet;
  pub use crate::NavMesh2d;
  pub use crate::NavMesh3d;
  pub use crate::NavMeshHandle2d;
  pub use crate::NavMeshHandle3d;
  pub use crate::NavigationMesh2d;
  pub use crate::NavigationMesh3d;
  pub use crate::ValidNavigationMesh2d;
  pub use crate::ValidNavigationMesh3d;
  pub use crate::Velocity2d;
  pub use crate::Velocity3d;
  pub use crate::coords::CoordinateSystem;
  pub use crate::coords::ThreeD;
  pub use crate::coords::TwoD;
}

pub struct LandmassPlugin<CS: CoordinateSystem> {
  schedule: Interned<dyn ScheduleLabel>,
  _marker: PhantomData<CS>,
}

impl<CS: CoordinateSystem> Default for LandmassPlugin<CS> {
  fn default() -> Self {
    Self { schedule: RunFixedMainLoop.intern(), _marker: Default::default() }
  }
}

impl<CS: CoordinateSystem> LandmassPlugin<CS> {
  /// Sets the schedule for running the plugin. Defaults to
  /// [`RunFixedMainLoop`].
  #[must_use]
  pub fn in_schedule(mut self, schedule: impl ScheduleLabel) -> Self {
    self.schedule = schedule.intern();
    self
  }
}

pub type Landmass2dPlugin = LandmassPlugin<TwoD>;
pub type Landmass3dPlugin = LandmassPlugin<ThreeD>;

impl<CS: CoordinateSystem> Plugin for LandmassPlugin<CS> {
  fn build(&self, app: &mut bevy_app::App) {
    app.init_asset::<NavMesh<CS>>();
    app.configure_sets(
      self.schedule,
      (
        LandmassSystemSet::SyncExistence,
        LandmassSystemSet::SyncValues,
        LandmassSystemSet::Update,
        LandmassSystemSet::Output,
      )
        .chain()
        // Configure our systems to run before physics engines.
        .in_set(RunFixedMainLoopSystem::BeforeFixedMainLoop),
    );
    app.add_systems(
      self.schedule,
      (
        add_agents_to_archipelagos::<CS>,
        sync_islands_to_archipelago::<CS>,
        add_characters_to_archipelago::<CS>,
      )
        .in_set(LandmassSystemSet::SyncExistence),
    );
    app.add_systems(
      self.schedule,
      (sync_agent_input_state::<CS>, sync_character_state::<CS>)
        .in_set(LandmassSystemSet::SyncValues),
    );
    app.add_systems(
      self.schedule,
      update_archipelagos::<CS>.in_set(LandmassSystemSet::Update),
    );
    app.add_systems(
      self.schedule,
      (sync_agent_state::<CS>, sync_desired_velocity::<CS>)
        .in_set(LandmassSystemSet::Output),
    );
  }
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

/// An archipelago, holding the internal state of `landmass`.
#[derive(Component)]
pub struct Archipelago<CS: CoordinateSystem> {
  /// The `landmass` archipelago.
  archipelago: landmass::Archipelago<CS>,
  /// A map from the Bevy entity to its associated island ID in
  /// [`Self::archipelago`].
  islands: HashMap<Entity, IslandId>,
  /// A map from the island ID to its associated Bevy entity in
  /// [`Self::archipelago`]. This is just the reverse of [`Self::islands`].
  reverse_islands: HashMap<IslandId, Entity>,
  /// A map from the Bevy entity to its associated agent ID in
  /// [`Self::archipelago`].
  agents: HashMap<Entity, AgentId>,
  /// A map from the agent ID to its associated Bevy entity in
  /// [`Self::archipelago`]. This is just the reverse of [`Self::agents`].
  reverse_agents: HashMap<AgentId, Entity>,
  /// A map from the Bevy entity to its associated character ID in
  /// [`Self::archipelago`].
  characters: HashMap<Entity, CharacterId>,
}

pub type Archipelago2d = Archipelago<TwoD>;
pub type Archipelago3d = Archipelago<ThreeD>;

impl<CS: CoordinateSystem> Archipelago<CS> {
  /// Creates an empty archipelago.
  pub fn new(agent_options: AgentOptions<CS>) -> Self {
    Self {
      archipelago: landmass::Archipelago::new(agent_options),
      islands: HashMap::new(),
      reverse_islands: HashMap::new(),
      agents: HashMap::new(),
      reverse_agents: HashMap::new(),
      characters: HashMap::new(),
    }
  }

  /// Gets the agent options.
  pub fn get_agent_options(&self) -> &AgentOptions<CS> {
    &self.archipelago.agent_options
  }

  /// Gets a mutable borrow to the agent options.
  pub fn get_agent_options_mut(&mut self) -> &mut AgentOptions<CS> {
    &mut self.archipelago.agent_options
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
    self.archipelago.add_node_type(cost)
  }

  /// Sets the cost of `node_type` to `cost`. See
  /// [`Archipelago::add_node_type`] for the meaning of cost.
  pub fn set_node_type_cost(
    &mut self,
    node_type: NodeType,
    cost: f32,
  ) -> Result<(), SetNodeTypeCostError> {
    self.archipelago.set_node_type_cost(node_type, cost)
  }

  /// Gets the cost of `node_type`. Returns [`None`] if `node_type` is not in
  /// this archipelago.
  pub fn get_node_type_cost(&self, node_type: NodeType) -> Option<f32> {
    self.archipelago.get_node_type_cost(node_type)
  }

  /// Removes the node type from the archipelago. Returns false if this
  /// archipelago does not contain `node_type` or any islands still use this
  /// node type (so the node type cannot be removed). Otherwise, returns true.
  pub fn remove_node_type(&mut self, node_type: NodeType) -> bool {
    self.archipelago.remove_node_type(node_type)
  }

  /// Finds the nearest point on the navigation meshes to (and within
  /// `distance_to_node` of) `point`.
  pub fn sample_point(
    &self,
    point: CS::Coordinate,
    point_sample_distance: &CS::SampleDistance,
  ) -> Result<SampledPoint<'_, CS>, SamplePointError> {
    let sampled_point =
      self.archipelago.sample_point(point, point_sample_distance)?;
    Ok(SampledPoint {
      island: *self
        .reverse_islands
        .get(&sampled_point.island())
        .expect("The island hasn't been removed from the archipelago."),
      sampled_point,
    })
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
    self.archipelago.find_path(
      &start_point.sampled_point,
      &end_point.sampled_point,
      override_node_type_costs,
    )
  }

  /// Gets an agent.
  fn get_agent(&self, entity: Entity) -> Option<&landmass::Agent<CS>> {
    self
      .agents
      .get(&entity)
      .and_then(|&agent_id| self.archipelago.get_agent(agent_id))
  }

  /// Gets a mutable borrow to an agent.
  fn get_agent_mut(
    &mut self,
    entity: Entity,
  ) -> Option<&mut landmass::Agent<CS>> {
    self
      .agents
      .get(&entity)
      .and_then(|&agent_id| self.archipelago.get_agent_mut(agent_id))
  }

  /// Gets a borrow to a character.
  #[allow(unused)] // Used in tests.
  fn get_character(&self, entity: Entity) -> Option<&landmass::Character<CS>> {
    self
      .characters
      .get(&entity)
      .and_then(|&character_id| self.archipelago.get_character(character_id))
  }

  /// Gets a mutable borrow to a character.
  fn get_character_mut(
    &mut self,
    entity: Entity,
  ) -> Option<&mut landmass::Character<CS>> {
    self.characters.get(&entity).and_then(|&character_id| {
      self.archipelago.get_character_mut(character_id)
    })
  }

  /// Gets a borrow to a character.
  #[allow(unused)] // Used in tests.
  fn get_island(&self, entity: Entity) -> Option<&landmass::Island<CS>> {
    self
      .islands
      .get(&entity)
      .and_then(|&island_id| self.archipelago.get_island(island_id))
  }

  /// Gets a mutable borrow to an island (if present).
  fn get_island_mut(
    &mut self,
    entity: Entity,
  ) -> Option<landmass::IslandMut<'_, CS>> {
    self
      .islands
      .get(&entity)
      .and_then(|&island_id| self.archipelago.get_island_mut(island_id))
  }
}

/// Updates the archipelago.
fn update_archipelagos<CS: CoordinateSystem>(
  time: Res<Time>,
  mut archipelago_query: Query<&mut Archipelago<CS>>,
) {
  for mut archipelago in archipelago_query.iter_mut() {
    archipelago.archipelago.update(time.delta_secs());
  }
}

/// A reference to an archipelago.
#[derive(Component)]
pub struct ArchipelagoRef<CS: CoordinateSystem> {
  pub entity: Entity,
  pub marker: PhantomData<CS>,
}

pub type ArchipelagoRef2d = ArchipelagoRef<TwoD>;
pub type ArchipelagoRef3d = ArchipelagoRef<ThreeD>;

impl<CS: CoordinateSystem<Coordinate: std::fmt::Debug>> std::fmt::Debug
  for ArchipelagoRef<CS>
{
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.debug_struct("ArchipelagoRef")
      .field("entity", &self.entity)
      .field("marker", &self.marker)
      .finish()
  }
}

impl<CS: CoordinateSystem> ArchipelagoRef<CS> {
  pub fn new(entity: Entity) -> Self {
    Self { entity, marker: Default::default() }
  }
}

pub type NavigationMesh2d = NavigationMesh<TwoD>;
pub type NavigationMesh3d = NavigationMesh<ThreeD>;

pub type HeightNavigationMesh2d = HeightNavigationMesh<TwoD>;
pub type HeightNavigationMesh3d = HeightNavigationMesh<ThreeD>;

pub type ValidNavigationMesh2d = ValidNavigationMesh<TwoD>;
pub type ValidNavigationMesh3d = ValidNavigationMesh<ThreeD>;

/// An asset holding a `landmass` nav mesh.
#[derive(Asset, TypePath)]
pub struct NavMesh<CS: CoordinateSystem> {
  /// The nav mesh data.
  pub nav_mesh: Arc<ValidNavigationMesh<CS>>,
  /// A map from the type indices used by [`Self::nav_mesh`] to the
  /// [`NodeType`]s used in the [`crate::Archipelago`]. Type indices not
  /// present in this map are implicitly assigned the "default" node type,
  /// which always has a cost of 1.0.
  pub type_index_to_node_type: HashMap<usize, NodeType>,
}

pub type NavMesh2d = NavMesh<TwoD>;
pub type NavMesh3d = NavMesh<ThreeD>;

/// A handle to a navigation mesh for an [`Island`].
#[derive(Component, Clone, Debug)]
pub struct NavMeshHandle<CS: CoordinateSystem>(pub Handle<NavMesh<CS>>);

impl<CS: CoordinateSystem> Default for NavMeshHandle<CS> {
  fn default() -> Self {
    Self(Default::default())
  }
}

pub type NavMeshHandle2d = NavMeshHandle<TwoD>;
pub type NavMeshHandle3d = NavMeshHandle<ThreeD>;

/// A point on the navigation meshes.
pub struct SampledPoint<'archipelago, CS: CoordinateSystem> {
  /// The sampled point from landmass.
  sampled_point: landmass::SampledPoint<'archipelago, CS>,
  /// The island that the point is on.
  island: Entity,
}

// Manual Clone impl for `SampledPoint` to avoid the Clone bound on CS.
impl<CS: CoordinateSystem> Clone for SampledPoint<'_, CS> {
  fn clone(&self) -> Self {
    Self { sampled_point: self.sampled_point.clone(), island: self.island }
  }
}

impl<CS: CoordinateSystem> SampledPoint<'_, CS> {
  /// Gets the point on the navigation meshes.
  pub fn point(&self) -> CS::Coordinate {
    self.sampled_point.point()
  }

  /// Gets the island the sampled point is on.
  pub fn island(&self) -> Entity {
    self.island
  }

  /// Gets the node type of the sampled point. Returns None if the node type
  /// is the default node type.
  pub fn node_type(&self) -> Option<NodeType> {
    self.sampled_point.node_type()
  }
}

#[cfg(test)]
#[path = "lib_test.rs"]
mod test;

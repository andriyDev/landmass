#![doc = include_str!("../README.md")]

use std::{collections::HashMap, marker::PhantomData, sync::Arc};

use bevy::{
  asset::{Asset, AssetApp},
  prelude::{
    Component, Entity, IntoSystemConfigs, IntoSystemSetConfigs, Plugin, Query,
    Res, SystemSet, Update,
  },
  reflect::TypePath,
  time::Time,
};
use coords::{CoordinateSystem, ThreeD, TwoD};
use landmass::{AgentId, CharacterId, IslandId};

mod agent;
mod character;
mod island;
mod landmass_structs;

pub use landmass::AgentOptions;
pub use landmass::NavigationMesh;
pub use landmass::NewNodeTypeError;
pub use landmass::NodeType;
pub use landmass::SetNodeTypeCostError;
pub use landmass::ValidNavigationMesh;
pub use landmass::ValidationError;

pub use agent::*;
pub use character::*;
pub use island::*;
pub use landmass_structs::*;

pub mod coords;
pub mod debug;

#[cfg(feature = "mesh-utils")]
pub mod nav_mesh;

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
  pub use crate::Character;
  pub use crate::Character2dBundle;
  pub use crate::Character3dBundle;
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

pub struct LandmassPlugin<CS: CoordinateSystem>(PhantomData<CS>);

impl<CS: CoordinateSystem> Default for LandmassPlugin<CS> {
  fn default() -> Self {
    Self(Default::default())
  }
}

pub type Landmass2dPlugin = LandmassPlugin<TwoD>;
pub type Landmass3dPlugin = LandmassPlugin<ThreeD>;

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
        sync_islands_to_archipelago::<CS>,
        add_characters_to_archipelago::<CS>,
      )
        .in_set(LandmassSystemSet::SyncExistence),
    );
    app.add_systems(
      Update,
      (sync_agent_input_state::<CS>, sync_character_state::<CS>)
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
  /// A map from the Bevy entity to its associated character ID in
  /// [`Self::archipelago`].
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
      reverse_islands: HashMap::new(),
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

  /// Gets a mutable borrow to a character.
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

  /// Gets a mutable borrow to an island (if present).
  fn get_island_mut(
    &mut self,
    entity: Entity,
  ) -> Option<landmass::IslandMut<CS>> {
    self
      .islands
      .get(&entity)
      .and_then(|&island_id| self.archipelago.get_island_mut(island_id))
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

pub type NavigationMesh2d = NavigationMesh<TwoD>;
pub type NavigationMesh3d = NavigationMesh<ThreeD>;

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

#[cfg(test)]
#[path = "lib_test.rs"]
mod test;

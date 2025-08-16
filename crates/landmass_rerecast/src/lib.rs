use std::sync::{Arc, Weak};

use bevy_app::{Plugin, RunFixedMainLoop, RunFixedMainLoopSystem};
use bevy_asset::{AssetHandleProvider, AssetId, Assets, Handle, StrongHandle};
use bevy_ecs::{
  component::{Component, HookContext},
  intern::Interned,
  resource::Resource,
  schedule::{IntoScheduleConfigs, ScheduleLabel, SystemSet},
  world::DeferredWorld,
};
use bevy_landmass::LandmassSystemSet;
use bevy_platform::collections::{HashMap, hash_map::Entry};

mod raw_conversion;
pub use raw_conversion::convert_rerecast_navmesh_to_landmass_navmesh;

pub struct LandmassRerecastPlugin {
  schedule: Interned<dyn ScheduleLabel>,
}

impl Default for LandmassRerecastPlugin {
  fn default() -> Self {
    Self { schedule: RunFixedMainLoop.intern() }
  }
}

impl LandmassRerecastPlugin {
  /// Sets the schedule for running the plugin. Defaults to
  /// [`RunFixedMainLoop`].
  #[must_use]
  pub fn in_schedule(mut self, schedule: impl ScheduleLabel) -> Self {
    self.schedule = schedule.intern();
    self
  }
}

impl Plugin for LandmassRerecastPlugin {
  fn build(&self, app: &mut bevy_app::App) {
    app.init_resource::<RerecastToLandmassIds>().configure_sets(
      self.schedule,
      LandmassRerecastSystems
        .before(LandmassSystemSet::SyncExistence)
        .in_set(RunFixedMainLoopSystem::BeforeFixedMainLoop),
    );
  }
}

/// System set for systems converting between `landmass` and `rerecast`.
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct LandmassRerecastSystems;

/// A replacement for [`bevy_landmass::NavMeshHandle3d`] that stores a
/// [`bevy_rerecast_core::Navmesh`] handle.
#[derive(Component, Clone, Debug)]
#[component(immutable, on_insert=on_insert_rerecast_navmesh, on_replace=on_replace_rerecast_navmesh)]
pub struct NavMeshHandle3d(pub Handle<bevy_rerecast_core::Navmesh>);

// Due to https://github.com/rust-lang/rust/issues/73191, users could be using
// `bevy_landmass::NavMeshHandle` instead of `bevy_landmass::NavMeshHandle3d`.
// So we provide this type alias so it's easy to merge.
pub type NavMeshHandle = NavMeshHandle3d;

/// OnInsert hook for `NavMeshHandle3d` to insert an associated
/// `bevy_landmass::NavMeshHandle3d`.
fn on_insert_rerecast_navmesh(
  mut world: DeferredWorld,
  HookContext { entity, .. }: HookContext,
) {
  let rerecast_id =
    world.entity(entity).get::<crate::NavMeshHandle3d>().unwrap().0.id();
  let landmass_handles =
    world.resource::<Assets<bevy_landmass::NavMesh3d>>().get_handle_provider();

  // Keep track of the mapping from rerecast to landmass.
  let (landmass_handle, allocated) = world
    .resource_mut::<RerecastToLandmassIds>()
    .get_or_alloc(rerecast_id, landmass_handles);
  if allocated {
    // TODO: send an event to do the conversion in case the rerecast asset is
    // already loaded.
  }

  // Insert a landmass handle so that landmass can see this entity.
  world
    .commands()
    .entity(entity)
    .insert(bevy_landmass::NavMeshHandle(landmass_handle));
}

/// OnReplace hook for `NavMeshHandle3d` to remove the associated
/// `bevy_landmass::NavMeshHandle3d`.
fn on_replace_rerecast_navmesh(
  mut world: DeferredWorld,
  HookContext { entity, .. }: HookContext,
) {
  let rerecast_id =
    world.entity(entity).get::<crate::NavMeshHandle3d>().unwrap().0.id();
  // Stop tracking this mapping.
  world.resource_mut::<RerecastToLandmassIds>().remove(rerecast_id);

  let mut commands = world.commands();
  if let Ok(mut entity) = commands.get_entity(entity) {
    // Remove the landmass handle from this entity. It's ok if the user deleted
    // it though.
    entity.try_remove::<bevy_landmass::NavMeshHandle3d>();
  }
}

/// A resource to keep track of the mapping from rerecast nav meshes into their
/// landmass equivalents.
#[derive(Resource, Default)]
struct RerecastToLandmassIds(
  HashMap<AssetId<bevy_rerecast_core::Navmesh>, Weak<StrongHandle>>,
);

impl RerecastToLandmassIds {
  /// Looks up the landmass handle for `rerecast_id` or allocates one if one
  /// isn't present.
  ///
  /// Returns the handle and a bool indicating whether the handle was found
  /// (false) or was allocated (true).
  // Note: we pass the raw handle provider since we need mutable access to this
  // resource in the hook.
  fn get_or_alloc(
    &mut self,
    rerecast_id: AssetId<bevy_rerecast_core::Navmesh>,
    landmass_handle_provider: AssetHandleProvider,
  ) -> (Handle<bevy_landmass::NavMesh3d>, bool) {
    match self.0.entry(rerecast_id) {
      Entry::Occupied(entry) => {
        let handle = entry
          .get()
          .upgrade()
          .expect("mapping exists so the handle should still be alive");
        (Handle::Strong(handle), false)
      }
      Entry::Vacant(entry) => {
        let handle = landmass_handle_provider.reserve_handle().typed();
        let Handle::Strong(arc) = &handle else {
          unreachable!("reserve_handle should always return a Strong handle");
        };
        entry.insert(Arc::downgrade(arc));
        (handle, true)
      }
    }
  }

  /// Removes the mapping for `rerecast_id`.
  fn remove(&mut self, rerecast_id: AssetId<bevy_rerecast_core::Navmesh>) {
    self.0.remove(&rerecast_id);
  }
}

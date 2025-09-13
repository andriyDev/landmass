#![doc = include_str!("../README.md")]

use std::sync::Arc;

use bevy_app::{Plugin, RunFixedMainLoop, RunFixedMainLoopSystems};
use bevy_asset::{AssetEvent, AssetHandleProvider, AssetId, Assets, Handle};
use bevy_ecs::{
  bundle::Bundle,
  component::Component,
  intern::Interned,
  lifecycle::HookContext,
  message::{Message, MessageReader},
  resource::Resource,
  schedule::{
    IntoScheduleConfigs, ScheduleLabel, SystemCondition, SystemSet,
    common_conditions::on_message,
  },
  system::ResMut,
  world::DeferredWorld,
};
use bevy_landmass::{ArchipelagoRef3d, Island, LandmassSystems};
use bevy_platform::collections::{HashMap, HashSet, hash_map::Entry};

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
    app
      .init_resource::<RerecastToLandmassIds>()
      .add_message::<NewRerecastConversion>()
      .configure_sets(
        self.schedule,
        LandmassRerecastSystems
          .before(LandmassSystems::SyncExistence)
          .in_set(RunFixedMainLoopSystems::BeforeFixedMainLoop),
      )
      .add_systems(
        self.schedule,
        convert_changed_rerecast_meshes_to_landmass
          .in_set(LandmassRerecastSystems)
          .run_if(
            on_message::<AssetEvent<bevy_rerecast::Navmesh>>
              .or(on_message::<NewRerecastConversion>),
          ),
      );
  }
}

/// System set for systems converting between `landmass` and `rerecast`.
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct LandmassRerecastSystems;

/// A replacement for [`bevy_landmass::Island3dBundle`] that uses a rerecast
/// mesh instead.
#[derive(Bundle)]
pub struct Island3dBundle {
  /// An island marker component.
  pub island: Island,
  /// A reference pointing to the Archipelago to associate this entity with.
  pub archipelago_ref: ArchipelagoRef3d,
  /// A handle to the nav mesh that this island needs.
  pub nav_mesh: NavMeshHandle3d,
}

/// A replacement for [`bevy_landmass::NavMeshHandle3d`] that stores a
/// [`bevy_rerecast::Navmesh`] handle.
#[derive(Component, Clone, Debug)]
#[component(immutable, on_insert=on_insert_rerecast_navmesh, on_replace=on_replace_rerecast_navmesh)]
pub struct NavMeshHandle3d(pub Handle<bevy_rerecast::Navmesh>);

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
    // Send a message to do the conversion in case the rerecast asset is already
    // loaded.
    world.write_message(NewRerecastConversion(rerecast_id));
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
  HashMap<AssetId<bevy_rerecast::Navmesh>, Handle<bevy_landmass::NavMesh3d>>,
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
    rerecast_id: AssetId<bevy_rerecast::Navmesh>,
    landmass_handle_provider: AssetHandleProvider,
  ) -> (Handle<bevy_landmass::NavMesh3d>, bool) {
    match self.0.entry(rerecast_id) {
      Entry::Occupied(entry) => (entry.get().clone(), false),
      Entry::Vacant(entry) => {
        let handle = landmass_handle_provider.reserve_handle().typed();
        entry.insert(handle.clone());
        (handle, true)
      }
    }
  }

  /// Removes the mapping for `rerecast_id`.
  fn remove(&mut self, rerecast_id: AssetId<bevy_rerecast::Navmesh>) {
    self.0.remove(&rerecast_id);
  }

  /// Looks up the landmass mesh ID associated with the rerecast mesh ID.
  fn get(
    &self,
    rerecast_id: AssetId<bevy_rerecast::Navmesh>,
  ) -> Option<AssetId<bevy_landmass::NavMesh3d>> {
    self.0.get(&rerecast_id).map(|handle| handle.id())
  }
}

/// A message to indicate that a new conversion between rerecast and landmass
/// has been established.
///
/// This gives us an opportunity to do a conversion if the rerecast mesh is
/// already loaded.
#[derive(Message)]
struct NewRerecastConversion(AssetId<bevy_rerecast::Navmesh>);

/// A system to do the actual conversion between rerecast and landmass.
fn convert_changed_rerecast_meshes_to_landmass(
  mut rerecast_events: MessageReader<AssetEvent<bevy_rerecast::Navmesh>>,
  mut new_conversion_events: MessageReader<NewRerecastConversion>,
  mut mapping: ResMut<RerecastToLandmassIds>,
  mut rerecast_meshes: ResMut<Assets<bevy_rerecast::Navmesh>>,
  mut landmass_meshes: ResMut<Assets<bevy_landmass::NavMesh3d>>,
) {
  let mut changed_rerecast_ids = HashSet::new();
  for event in rerecast_events.read() {
    match event {
      AssetEvent::Added { id } | AssetEvent::Modified { id } => {
        changed_rerecast_ids.insert(*id);
      }
      AssetEvent::Unused { id } => {
        mapping.remove(*id);
      }
      _ => {}
    }
  }
  changed_rerecast_ids.extend(new_conversion_events.read().map(|e| e.0));

  for rerecast_id in changed_rerecast_ids {
    let Some(landmass_id) = mapping.get(rerecast_id) else {
      // The mapping has been cleaned up since we got the event OR this mesh
      // never had a mapping.
      continue;
    };

    let Some(rerecast_mesh) = rerecast_meshes.remove(rerecast_id) else {
      // We always send an event the first time the conversion is created in
      // case this asset exists. But it is also perfectly valid for this asset
      // to not exist and still be loading.
      continue;
    };
    let landmass_mesh =
      convert_rerecast_navmesh_to_landmass_navmesh(&rerecast_mesh);
    let landmass_mesh = match landmass_mesh.validate() {
      Ok(landmass_mesh) => landmass_mesh,
      Err(err) => {
        bevy_log::error!(
          "Failed to validate rerecast mesh when converting to landmass: {err}"
        );
        continue;
      }
    };
    landmass_meshes
      .insert(
        landmass_id,
        bevy_landmass::NavMesh { nav_mesh: Arc::new(landmass_mesh) },
      )
      .unwrap();
  }
}

#[cfg(test)]
#[path = "lib_test.rs"]
mod test;

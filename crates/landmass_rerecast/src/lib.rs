use std::sync::Arc;

use bevy_app::{Plugin, RunFixedMainLoop, RunFixedMainLoopSystem};
use bevy_asset::{AssetEvent, AssetId, Assets, Handle};
use bevy_ecs::{
  error::BevyError,
  event::EventReader,
  intern::Interned,
  resource::Resource,
  schedule::{IntoScheduleConfigs, ScheduleLabel, SystemSet},
  system::{Res, ResMut, SystemParam},
};
use bevy_platform::collections::{HashMap, hash_map::Entry};
use thiserror::Error;

use bevy_landmass::{LandmassSystemSet, NavMesh3d as LandmassNavMesh};
use bevy_rerecast_core::Navmesh as RerecastNavMesh;

pub use bevy_rerecast_core::NavmeshSettings;

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
    app.configure_sets(
      self.schedule,
      LandmassRerecastSystems
        .before(LandmassSystemSet::SyncExistence)
        .in_set(RunFixedMainLoopSystem::BeforeFixedMainLoop),
    );

    app.add_systems(
      self.schedule,
      update_navmesh_conversion.in_set(LandmassRerecastSystems),
    );
  }
}

/// System set for systems converting between `landmass` and `rerecast`.
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct LandmassRerecastSystems;

/// System param for generating navmeshes.
///
/// This mirrors [`bevy_rerecast_core::generator::NavmeshGenerator`].
#[derive(SystemParam)]
pub struct NavmeshGenerator<'w> {
  /// The landmass meshes so we can reserve handles from it.
  landmass_meshes: Res<'w, Assets<LandmassNavMesh>>,
  /// The conversions from landmass meshes to rerecast meshes so we can add to
  /// it.
  conversion: ResMut<'w, NavMeshConversion>,
  /// The rerecast generator so we can queue the actual nav mesh generation.
  rerecast_generator: bevy_rerecast_core::generator::NavmeshGenerator<'w>,
}

impl NavmeshGenerator<'_> {
  /// Queue a navmesh generation task.
  ///
  /// When you call this method, a new navmesh will be generated asynchronously.
  /// Calling it multiple times will queue multiple navmeshes to be generated.
  /// Obstacles existing this frame at [`PostUpdate`] will be used to generate
  /// the navmesh.
  pub fn generate(
    &mut self,
    settings: NavmeshSettings,
  ) -> Handle<LandmassNavMesh> {
    let landmass_handle = self.landmass_meshes.reserve_handle();
    let rerecast_handle = self.rerecast_generator.generate(settings);

    self
      .conversion
      .add_conversion(landmass_handle.id(), rerecast_handle)
      .expect("Both handles are new, so they cannot be mapped");

    landmass_handle
  }

  /// Queue a navmesh regeneration task.
  ///
  /// When you call this method, an existing navmesh will be regenerated
  /// asynchronously. Calling it multiple times will have no effect until the
  /// regeneration is complete. Obstacles existing this frame at [`PostUpdate`]
  /// will be used to generate the navmesh.
  ///
  /// Returns `true` if the regeneration was successfully queued now, `false` if
  /// it was already previously queued.
  pub fn regenerate(
    &mut self,
    id: &Handle<LandmassNavMesh>,
    settings: NavmeshSettings,
  ) -> Result<bool, MissingConversionError> {
    let Some(rerecast_id) = self.conversion.landmass_to_rerecast.get(&id.id())
    else {
      return Err(MissingConversionError(id.id()));
    };

    Ok(self.rerecast_generator.regenerate(rerecast_id, settings))
  }
}

#[derive(Debug, Error, Clone, Copy)]
#[error(
  "Landmass nav mesh {0} does not have an existing conversion to a rerecast nav mesh. Use `generate` to create the conversion."
)]
pub struct MissingConversionError(pub AssetId<LandmassNavMesh>);

/// A mapping of tracked conversions between landmass and rerecast nav meshes.
#[derive(Resource)]
struct NavMeshConversion {
  /// Maps from a landmass ID to the rerecast nav mesh handle. Note this keeps
  /// the rerecast nav mesh alive for as long as the mapping exists.
  landmass_to_rerecast:
    HashMap<AssetId<LandmassNavMesh>, Handle<RerecastNavMesh>>,
  /// Maps from a rerecast nav mesh handle back to the landmass ID. This
  /// mapping is always the reverse mapping of [`Self::landmass_to_recast`].
  rerecast_to_landmass:
    HashMap<AssetId<RerecastNavMesh>, AssetId<LandmassNavMesh>>,
}

impl NavMeshConversion {
  /// Add a new conversion between a rerecast mesh handle and a landmass asset
  /// ID. The rerecast asset will now be tracked and will automatically update
  /// the associated landmass asset.
  fn add_conversion(
    &mut self,
    landmass: AssetId<LandmassNavMesh>,
    rerecast: Handle<RerecastNavMesh>,
  ) -> Result<(), AddConversionError> {
    let landmass_to_rerecast_entry =
      match self.landmass_to_rerecast.entry(landmass) {
        Entry::Vacant(entry) => entry,
        Entry::Occupied(entry) => {
          return Err(AddConversionError::LandmassMapped(
            landmass,
            entry.get().id(),
          ));
        }
      };
    let rerecast_to_landmass_entry =
      match self.rerecast_to_landmass.entry(rerecast.id()) {
        Entry::Vacant(entry) => entry,
        Entry::Occupied(entry) => {
          return Err(AddConversionError::RerecastMapped(
            rerecast.id(),
            *entry.get(),
          ));
        }
      };

    landmass_to_rerecast_entry.insert(rerecast);
    rerecast_to_landmass_entry.insert(landmass);
    Ok(())
  }

  /// Removes the conversion between `landmass` and what it previously mapped
  /// to. Does nothing if `landmass` is not converted.
  fn remove_conversion(&mut self, landmass: AssetId<LandmassNavMesh>) {
    let Some(rerecast) = self.landmass_to_rerecast.remove(&landmass) else {
      // Silently do nothing if the landmass ID isn't mapped.
      return;
    };

    self.rerecast_to_landmass.remove(&rerecast.id()).expect(
      "The rerecast mapping exists since the landmass asset referenced it",
    );
  }
}

/// An error while adding a new conversion to [`NavMeshConversion`].
#[derive(Debug, Error, Clone, Copy)]
enum AddConversionError {
  #[error(
    "Landmass asset ID {0} is already mapped to another rerecast asset {1}"
  )]
  LandmassMapped(AssetId<LandmassNavMesh>, AssetId<RerecastNavMesh>),
  #[error(
    "Rerecast asset ID {0} is already mapped to another landmass asset {1}"
  )]
  RerecastMapped(AssetId<RerecastNavMesh>, AssetId<LandmassNavMesh>),
}

/// Updates the assets of landmass nav meshes based on the changes in their
/// rerecast assets.
fn update_navmesh_conversion(
  mut conversion: ResMut<NavMeshConversion>,
  mut landmass_assets: ResMut<Assets<LandmassNavMesh>>,
  mut rerecast_assets: ResMut<Assets<RerecastNavMesh>>,
  mut landmass_events: EventReader<AssetEvent<LandmassNavMesh>>,
  mut rerecast_events: EventReader<AssetEvent<RerecastNavMesh>>,
) -> Result<(), BevyError> {
  for event in landmass_events.read() {
    match event {
      AssetEvent::Unused { id } => {
        conversion.remove_conversion(*id);
      }
      _ => {}
    }
  }

  for event in rerecast_events.read() {
    match event {
      AssetEvent::Added { id } | AssetEvent::Modified { id } => {
        let Some(&landmass_id) = conversion.rerecast_to_landmass.get(id) else {
          continue;
        };

        let Some(rerecast_asset) = rerecast_assets.remove(*id) else {
          // This could fail if the asset was removed by the time we processed
          // these events, or if an add and modify happened on the same frame.
          continue;
        };

        let landmass_unvalidated_mesh = rerecast_to_landmass(&rerecast_asset);

        let landmass_mesh = match landmass_unvalidated_mesh.validate() {
          Ok(landmass_mesh) => landmass_mesh,
          Err(err) => todo!(),
        };

        landmass_assets.insert(
          landmass_id,
          LandmassNavMesh {
            nav_mesh: Arc::new(landmass_mesh),
            // TODO: Figure out how to set this appropriately.
            type_index_to_node_type: Default::default(),
          },
        );
      }
      _ => {}
    }
  }

  Ok(())
}

/// Converts a [`RerecastNavMesh`] into a raw, unvalidated landmass nav mesh.
fn rerecast_to_landmass(
  rerecast_navmesh: &RerecastNavMesh,
) -> bevy_landmass::NavigationMesh3d {
  todo!()
}

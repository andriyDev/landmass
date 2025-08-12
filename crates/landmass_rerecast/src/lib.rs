use bevy_app::{Plugin, RunFixedMainLoop, RunFixedMainLoopSystem};
use bevy_ecs::{
  intern::Interned,
  schedule::{IntoScheduleConfigs, ScheduleLabel, SystemSet},
};

use bevy_landmass::LandmassSystemSet;

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
  }
}

/// System set for systems converting between `landmass` and `rerecast`.
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct LandmassRerecastSystems;

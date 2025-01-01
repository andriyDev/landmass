use std::{collections::HashMap, time::Duration};

use bevy::prelude::*;
use bevy_landmass::prelude::*;
use bevy_rapier3d::prelude::*;
use oxidized_navigation::{
  ActiveGenerationTasks, NavMeshAffector, NavMeshSettings,
  OxidizedNavigationPlugin,
};

use crate::{LandmassOxidizedNavigationPlugin, OxidizedArchipelago};

const TIMEOUT_DURATION: Duration = Duration::new(15, 0);
const SLEEP_DURATION: Duration = Duration::from_millis(2);

fn wait_for_generation_to_finish(app: &mut App) {
  loop {
    app.update();
    if app.world().resource::<ActiveGenerationTasks>().is_empty() {
      break;
    }

    assert!(app.world().resource::<Time>().elapsed() < TIMEOUT_DURATION);

    std::thread::sleep(SLEEP_DURATION);
  }
}

#[test]
fn generates_nav_mesh() {
  let mut app = App::new();
  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default())
    .add_plugins(OxidizedNavigationPlugin::<Collider>::new(
      NavMeshSettings::from_agent_and_bounds(0.5, 2.0, 100.0, -50.0),
    ))
    .add_plugins(LandmassOxidizedNavigationPlugin);

  let archipelago_entity = app
    .world_mut()
    .spawn((
      Archipelago3d::new(AgentOptions::default_for_agent_radius(0.5)),
      OxidizedArchipelago,
    ))
    .id();

  let collider = Collider::cuboid(1.5, 0.25, 1.5);

  let spawn_tile = |app: &mut App, position| {
    app.world_mut().spawn((
      Transform::from_translation(position),
      Visibility::default(),
      collider.clone(),
      NavMeshAffector,
    ));
  };

  spawn_tile(&mut app, Vec3::ZERO);
  spawn_tile(&mut app, Vec3::new(3.0, 0.0, 0.0));
  spawn_tile(&mut app, Vec3::new(6.0, 0.0, 0.0));
  spawn_tile(&mut app, Vec3::new(6.0, 0.0, 3.0));
  spawn_tile(&mut app, Vec3::new(6.0, 0.0, 6.0));
  spawn_tile(&mut app, Vec3::new(3.0, 0.0, 6.0));

  app.update();
  wait_for_generation_to_finish(&mut app);

  let archipelago =
    app.world().get::<Archipelago3d>(archipelago_entity).unwrap();
  let start_point = archipelago
    .sample_point(Vec3::new(0.0, 0.0, 0.0), /* distance_to_node= */ 0.3)
    .unwrap();
  let end_point = archipelago
    .sample_point(Vec3::new(3.0, 0.0, 6.0), /* distance_to_node= */ 0.3)
    .unwrap();

  let path =
    archipelago.find_path(&start_point, &end_point, &HashMap::new()).unwrap();
  assert_eq!(
    path,
    [
      Vec3::new(0.0, 0.25, 0.0),
      Vec3::new(5.0, 0.25, 1.25),
      Vec3::new(5.0, 0.25, 4.75),
      Vec3::new(3.0, 0.25, 6.0)
    ]
  );

  // Spawn some more tiles to make a shorter path.

  spawn_tile(&mut app, Vec3::new(0.0, 0.0, 3.0));
  spawn_tile(&mut app, Vec3::new(0.0, 0.0, 6.0));

  app.update();
  wait_for_generation_to_finish(&mut app);

  let archipelago =
    app.world().get::<Archipelago3d>(archipelago_entity).unwrap();
  let start_point = archipelago
    .sample_point(Vec3::new(0.0, 0.0, 0.0), /* distance_to_node= */ 0.3)
    .unwrap();
  let end_point = archipelago
    .sample_point(Vec3::new(3.0, 0.0, 6.0), /* distance_to_node= */ 0.3)
    .unwrap();

  let path =
    archipelago.find_path(&start_point, &end_point, &HashMap::new()).unwrap();
  assert_eq!(
    path,
    [
      Vec3::new(0.0, 0.25, 0.0),
      Vec3::new(1.0, 0.25, 4.75),
      Vec3::new(3.0, 0.25, 6.0)
    ]
  );
}

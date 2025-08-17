use bevy::{prelude::*, scene::SceneInstanceReady};
use bevy_landmass::{debug::Landmass3dDebugPlugin, prelude::*};
use bevy_rerecast_core::{NavmeshSettings, prelude::NavmeshGenerator};
use landmass_rerecast::{
  Island3dBundle, LandmassRerecastPlugin, NavMeshHandle3d,
};

fn main() {
  App::new()
    .add_plugins(DefaultPlugins)
    .add_plugins((
      bevy_rerecast_core::RerecastPlugin::default(),
      bevy_rerecast_core::Mesh3dBackendPlugin::default(),
    ))
    .add_plugins((
      Landmass3dPlugin::default(),
      Landmass3dDebugPlugin::default(),
      LandmassRerecastPlugin::default(),
    ))
    .add_systems(Startup, setup)
    .run();
}

fn setup(
  mut commands: Commands,
  nav_meshes: Res<Assets<bevy_rerecast_core::Navmesh>>,
  asset_server: Res<AssetServer>,
) {
  commands.spawn((
    Camera3d::default(),
    Transform::from_xyz(-11.0, 39.0, 38.0) //
      .looking_at(Vec3::new(13.6, 7.5, -13.2), Vec3::Y),
  ));

  commands.spawn((
    DirectionalLight::default(),
    Transform::default().looking_to(Vec3::NEG_ONE, Vec3::Y),
  ));

  let archipelago = commands
    .spawn(Archipelago3d::new(AgentOptions::from_agent_radius(0.5)))
    .id();

  let handle = nav_meshes.reserve_handle();

  // Use a separate entity for the island to make sure the scene transform
  // doesn't apply to the island.
  commands.spawn(Island3dBundle {
    island: Island,
    archipelago_ref: ArchipelagoRef3d::new(archipelago),
    nav_mesh: NavMeshHandle3d(handle.clone()),
  });

  commands
    .spawn(SceneRoot(asset_server.load("dungeon.glb#Scene0")))
    // Generate the navmesh once the scene loads.
    .observe(
      move |_: Trigger<SceneInstanceReady>, mut generator: NavmeshGenerator| {
        generator.regenerate(
          &handle,
          NavmeshSettings { agent_radius: 0.5, ..Default::default() },
        );
      },
    );
}

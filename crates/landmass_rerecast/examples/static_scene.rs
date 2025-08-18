use bevy::color::palettes::tailwind;
use bevy::picking::events::Pressed;
use bevy::{color::palettes::css, prelude::*, scene::SceneInstanceReady};
use bevy_landmass::{Agent3d, PointSampleDistance3d};
use bevy_landmass::{debug::Landmass3dDebugPlugin, prelude::*};
use bevy_rerecast::{NavmeshSettings, prelude::NavmeshGenerator};
use landmass_rerecast::{
  Island3dBundle, LandmassRerecastPlugin, NavMeshHandle3d,
};

fn main() {
  App::new()
    .add_plugins((DefaultPlugins, MeshPickingPlugin))
    .add_plugins((
      bevy_rerecast::RerecastPlugin::default(),
      bevy_rerecast::Mesh3dBackendPlugin::default(),
    ))
    .add_plugins((
      Landmass3dPlugin::default(),
      Landmass3dDebugPlugin::default(),
      LandmassRerecastPlugin::default(),
    ))
    .add_systems(Startup, setup)
    .add_systems(
      Update,
      (update_agent_velocity, move_agent_by_velocity, snap_to_ground).chain(),
    )
    .add_observer(on_click)
    .run();
}

fn setup(
  mut commands: Commands,
  nav_meshes: Res<Assets<bevy_rerecast::Navmesh>>,
  asset_server: Res<AssetServer>,
  mut meshes: ResMut<Assets<Mesh>>,
  mut materials: ResMut<Assets<StandardMaterial>>,
) {
  commands.spawn((
    Camera3d::default(),
    Transform::from_xyz(-11.0, 39.0, 38.0) //
      .looking_at(Vec3::new(13.6, 7.5, -13.2), Vec3::Y),
  ));

  commands.spawn((
    DirectionalLight {
      color: tailwind::AMBER_100.into(),
      ..Default::default()
    },
    Transform::default().looking_to(Vec3::NEG_ONE, Vec3::Y),
  ));

  commands.spawn(Text("LMB - Spawn agent\nRMB - Set agent target".into()));

  let archipelago = commands
    .spawn(Archipelago3d::new(AgentOptions {
      point_sample_distance: PointSampleDistance3d {
        distance_above: 0.75,
        distance_below: 0.75,
        ..PointSampleDistance3d::from_agent_radius(0.5)
      },
      ..AgentOptions::from_agent_radius(0.5)
    }))
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

  // Spawn the target.
  let target_entity = commands
    .spawn((
      Mesh3d(meshes.add(Sphere { radius: 0.25 })),
      MeshMaterial3d(materials.add(StandardMaterial {
        base_color: css::PURPLE.into(),
        ..Default::default()
      })),
      Target,
    ))
    .id();

  commands.insert_resource(AgentSpawner {
    mesh: meshes.add(Sphere { radius: 0.5 }),
    material: materials.add(StandardMaterial {
      base_color: css::SEA_GREEN.into(),
      ..Default::default()
    }),
    archipelago_entity: archipelago,
    target_entity,
  });
}

#[derive(Resource)]
struct AgentSpawner {
  mesh: Handle<Mesh>,
  material: Handle<StandardMaterial>,
  archipelago_entity: Entity,
  target_entity: Entity,
}

impl AgentSpawner {
  fn spawn(&self, position: Vec3, commands: &mut Commands) {
    commands.spawn((
      Transform::from_translation(position),
      Mesh3d(self.mesh.clone()),
      MeshMaterial3d(self.material.clone()),
      Agent3dBundle {
        agent: Default::default(),
        settings: AgentSettings {
          radius: 0.5,
          desired_speed: 7.0,
          max_speed: 10.0,
        },
        archipelago_ref: ArchipelagoRef3d::new(self.archipelago_entity),
      },
      AgentTarget3d::Entity(self.target_entity),
    ));
  }
}

/// Use the desired velocity as the agent's velocity.
fn update_agent_velocity(
  mut agent_query: Query<(&mut Velocity3d, &AgentDesiredVelocity3d)>,
) {
  for (mut velocity, desired_velocity) in agent_query.iter_mut() {
    velocity.velocity = desired_velocity.velocity();
  }
}

/// Apply the agent's velocity to its position.
fn move_agent_by_velocity(
  time: Res<Time>,
  mut agent_query: Query<(&mut Transform, &GlobalTransform, &Velocity3d)>,
) {
  for (mut transform, global_transform, velocity) in agent_query.iter_mut() {
    let local_velocity =
      global_transform.affine().inverse().transform_vector3(velocity.velocity);
    transform.translation += local_velocity * time.delta_secs();
  }
}

/// Snap the agent to the ground so it's always on the nav mesh. Normally this
/// is done by your physics engine.
fn snap_to_ground(
  mut agent_transforms: Query<&mut Transform, With<Agent3d>>,
  mut mesh_ray_cast: MeshRayCast,
  agents: Query<(), With<Agent3d>>,
) {
  let filter = |entity| !agents.contains(entity);
  let settings = &MeshRayCastSettings { filter: &filter, ..Default::default() };
  for mut transform in agent_transforms.iter_mut() {
    let result = mesh_ray_cast.cast_ray(
      Ray3d {
        origin: transform.translation + Vec3::Y * 0.5,
        direction: Dir3::NEG_Y,
      },
      &settings,
    );
    let Some((_, hit)) = result.first() else {
      continue;
    };

    transform.translation.y = hit.point.y;
  }
}

/// Marker component for the target entity.
#[derive(Component)]
struct Target;

fn on_click(
  mut trigger: Trigger<Pointer<Pressed>>,
  mut commands: Commands,
  spawner: Res<AgentSpawner>,
  target: Single<Entity, With<Target>>,
) {
  let Some(position) = trigger.hit.position else {
    return;
  };
  let Some(normal) = trigger.hit.normal else {
    return;
  };
  if normal.dot(Vec3::Y) < 0.4 {
    return;
  }
  trigger.propagate(false);
  match trigger.button {
    PointerButton::Primary => {
      spawner.spawn(position, &mut commands);
    }
    PointerButton::Secondary => {
      commands.entity(*target).insert(Transform::from_translation(position));
    }
    PointerButton::Middle => {}
  }
}

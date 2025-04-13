use bevy::{
  color::palettes::css::{ANTIQUE_WHITE, PURPLE, SEA_GREEN},
  input::common_conditions::input_just_pressed,
  prelude::*,
  render::mesh::{CylinderAnchor, CylinderMeshBuilder},
};
use bevy_landmass::{
  debug::{EnableLandmassDebug, Landmass3dDebugPlugin},
  prelude::*,
};
use bevy_rapier3d::{
  geometry::{AsyncCollider, ComputedColliderShape},
  plugin::{NoUserData, RapierPhysicsPlugin},
  prelude::{Collider, TriMeshFlags},
};
use landmass_oxidized_navigation::{
  LandmassOxidizedNavigationPlugin, OxidizedArchipelago,
};
use oxidized_navigation::{
  NavMeshAffector, NavMeshSettings, OxidizedNavigationPlugin,
};

fn main() {
  App::new()
    .add_plugins(DefaultPlugins)
    .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
    .add_plugins(OxidizedNavigationPlugin::<Collider>::new(NavMeshSettings {
      tile_width: 40,
      ..NavMeshSettings::from_agent_and_bounds(0.5, 2.0, 10.0, -5.0)
    }))
    .add_plugins(Landmass3dPlugin::default())
    .add_plugins(Landmass3dDebugPlugin::default())
    .add_plugins(LandmassOxidizedNavigationPlugin::default())
    .add_systems(Startup, setup)
    .add_systems(Update, handle_clicks)
    .add_systems(Update, toggle_debug.run_if(input_just_pressed(KeyCode::F12)))
    .add_systems(
      Update,
      (update_agent_velocity, move_agent_by_velocity).chain(),
    )
    .run();
}

fn setup(
  mut commands: Commands,
  mut meshes: ResMut<Assets<Mesh>>,
  mut materials: ResMut<Assets<StandardMaterial>>,
  asset_server: Res<AssetServer>,
) {
  commands.spawn((
    Camera3d::default(),
    Projection::Orthographic(OrthographicProjection {
      scaling_mode: bevy::render::camera::ScalingMode::FixedVertical {
        viewport_height: 12.0,
      },
      ..OrthographicProjection::default_3d()
    }),
    Transform::from_xyz(0.0, 1.0, -1.0)
      .looking_to(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, -1.0)),
  ));

  commands.spawn((
    Text::new(
      "LMB - Spawn agent\nRMB - Change target point\nF12 - Toggle debug view",
    ),
    TextLayout::new_with_justify(JustifyText::Right),
    Node {
      position_type: PositionType::Absolute,
      right: Val::Px(0.0),
      bottom: Val::Px(0.0),
      ..Default::default()
    },
  ));

  let archipelago_entity = commands
    .spawn((
      Archipelago3d::new(AgentOptions::from_agent_radius(0.5)),
      OxidizedArchipelago,
    ))
    .id();

  // Spawn the floors.
  commands.spawn((
    Mesh3d(asset_server.load("floor.glb#Mesh0/Primitive0")),
    MeshMaterial3d(materials.add(StandardMaterial {
      unlit: true,
      base_color: ANTIQUE_WHITE.into(),
      ..Default::default()
    })),
    AsyncCollider(ComputedColliderShape::TriMesh(TriMeshFlags::default())),
    NavMeshAffector,
  ));

  commands.spawn((
    Mesh3d(asset_server.load("floor.glb#Mesh1/Primitive0")),
    MeshMaterial3d(materials.add(StandardMaterial {
      unlit: true,
      base_color: ANTIQUE_WHITE.into(),
      ..Default::default()
    })),
    AsyncCollider(ComputedColliderShape::TriMesh(TriMeshFlags::default())),
    NavMeshAffector,
  ));

  // Spawn the target.
  let target_entity = commands
    .spawn((
      Transform::from_translation(Vec3::new(0.0, 0.0, 3.0)),
      Mesh3d(
        meshes.add(
          CylinderMeshBuilder {
            cylinder: Cylinder { radius: 0.25, half_height: 0.1 },
            resolution: 20,
            segments: 1,
            anchor: CylinderAnchor::MidPoint,
            caps: true,
          }
          .build(),
        ),
      ),
      MeshMaterial3d(materials.add(StandardMaterial {
        unlit: true,
        base_color: PURPLE.into(),
        ..Default::default()
      })),
      Target,
    ))
    .id();

  commands.insert_resource(AgentSpawner {
    mesh: meshes.add(
      CylinderMeshBuilder {
        cylinder: Cylinder { radius: 0.5, half_height: 0.1 },
        resolution: 20,
        segments: 1,
        anchor: CylinderAnchor::MidPoint,
        caps: true,
      }
      .build(),
    ),
    material: materials.add(StandardMaterial {
      unlit: true,
      base_color: SEA_GREEN.into(),
      ..Default::default()
    }),
    archipelago_entity,
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
          desired_speed: 2.0,
          max_speed: 3.0,
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

/// Marker component for the target entity.
#[derive(Component)]
struct Target;

/// Handles clicks by spawning agents with LMB and moving the target with RMB.
fn handle_clicks(
  buttons: Res<ButtonInput<MouseButton>>,
  window_query: Query<&Window>,
  camera_query: Query<(&Camera, &GlobalTransform)>,
  agent_spawner: Res<AgentSpawner>,
  target: Query<Entity, With<Target>>,
  mut commands: Commands,
) {
  let left = buttons.just_pressed(MouseButton::Left);
  let right = buttons.just_pressed(MouseButton::Right);
  if !left && !right {
    return;
  }

  let window = window_query.get_single().unwrap();
  let (camera, camera_transform) = camera_query.get_single().unwrap();

  let Some(world_position) = window
    .cursor_position()
    .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor).ok())
    .and_then(|ray| {
      ray
        .intersect_plane(Vec3::ZERO, InfinitePlane3d { normal: Dir3::Y })
        .map(|d| ray.get_point(d))
    })
  else {
    return;
  };

  if left {
    agent_spawner.spawn(world_position, &mut commands);
  }
  if right {
    commands
      .entity(target.single())
      .insert(Transform::from_translation(world_position));
  }
}

/// System for toggling the `EnableLandmassDebug` resource.
fn toggle_debug(mut debug: ResMut<EnableLandmassDebug>) {
  **debug = !**debug;
}

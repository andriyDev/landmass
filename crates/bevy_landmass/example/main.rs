use std::sync::Arc;

use bevy::{
  input::common_conditions::input_just_pressed, prelude::*,
  render::mesh::CylinderMeshBuilder,
};
use bevy_landmass::{
  debug::{EnableLandmassDebug, LandmassDebugPlugin},
  nav_mesh::bevy_mesh_to_landmass_nav_mesh,
  prelude::*,
};

fn main() {
  App::new()
    .add_plugins(DefaultPlugins)
    .add_plugins(LandmassPlugin)
    .add_plugins(LandmassDebugPlugin::default())
    .add_systems(Startup, setup)
    .add_systems(Update, (convert_mesh, handle_clicks))
    .add_systems(Update, toggle_debug.run_if(input_just_pressed(KeyCode::F12)))
    .add_systems(
      Update,
      (update_agent_velocity, move_agent_by_velocity).chain(),
    )
    .run();
}

// A utility to wait for a mesh to be loaded and convert the mesh to a nav mesh.
#[derive(Component)]
struct ConvertMesh {
  mesh: Handle<Mesh>,
  nav_mesh: Handle<NavMesh>,
}

fn convert_mesh(
  converters: Query<(Entity, &ConvertMesh)>,
  meshes: Res<Assets<Mesh>>,
  mut nav_meshes: ResMut<Assets<NavMesh>>,
  mut commands: Commands,
) {
  for (entity, converter) in converters.iter() {
    let Some(mesh) = meshes.get(&converter.mesh) else {
      continue;
    };

    let nav_mesh = bevy_mesh_to_landmass_nav_mesh(mesh).unwrap();
    let valid_nav_mesh = nav_mesh.validate().unwrap();
    nav_meshes
      .insert(converter.nav_mesh.clone(), NavMesh(Arc::new(valid_nav_mesh)));
    commands.entity(entity).remove::<ConvertMesh>();
  }
}

fn setup(
  mut commands: Commands,
  mut meshes: ResMut<Assets<Mesh>>,
  mut materials: ResMut<Assets<StandardMaterial>>,
  nav_meshes: Res<Assets<NavMesh>>,
  asset_server: Res<AssetServer>,
) {
  commands.spawn(Camera3dBundle {
    transform: Transform::from_xyz(5.0, 1.0, 0.0)
      .looking_to(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, -1.0)),
    projection: Projection::Orthographic(OrthographicProjection {
      scaling_mode: bevy::render::camera::ScalingMode::FixedVertical(16.0),
      ..Default::default()
    }),
    ..Default::default()
  });

  let archipelago_entity = commands.spawn(Archipelago::new()).id();

  // Spawn the islands.
  let mesh_1: Handle<Mesh> = asset_server.load("nav_mesh.glb#Mesh0/Primitive0");
  let nav_mesh_1 = nav_meshes.reserve_handle();
  commands.spawn((
    MaterialMeshBundle {
      mesh: mesh_1.clone(),
      material: materials.add(StandardMaterial {
        unlit: true,
        base_color: Color::ANTIQUE_WHITE,
        ..Default::default()
      }),
      ..Default::default()
    },
    IslandBundle {
      archipelago_ref: ArchipelagoRef(archipelago_entity),
      island: Island,
      nav_mesh: nav_mesh_1.clone(),
    },
    ConvertMesh { mesh: mesh_1, nav_mesh: nav_mesh_1 },
  ));

  let mesh_2: Handle<Mesh> = asset_server.load("nav_mesh.glb#Mesh1/Primitive0");
  let nav_mesh_2 = nav_meshes.reserve_handle();
  commands.spawn((
    MaterialMeshBundle {
      mesh: mesh_2.clone(),
      material: materials.add(StandardMaterial {
        unlit: true,
        base_color: Color::ANTIQUE_WHITE,
        ..Default::default()
      }),
      transform: Transform::from_translation(Vec3::new(12.0, 0.0, 0.0)),
      ..Default::default()
    },
    IslandBundle {
      archipelago_ref: ArchipelagoRef(archipelago_entity),
      island: Island,
      nav_mesh: nav_mesh_2.clone(),
    },
    ConvertMesh { mesh: mesh_2, nav_mesh: nav_mesh_2 },
  ));

  // Spawn the target.
  let target_entity = commands
    .spawn((
      MaterialMeshBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, 6.0)),
        mesh: meshes.add(
          CylinderMeshBuilder {
            cylinder: Cylinder { radius: 0.25, half_height: 0.1 },
            resolution: 20,
            segments: 1,
          }
          .build(),
        ),
        material: materials.add(StandardMaterial {
          unlit: true,
          base_color: Color::PURPLE,
          ..Default::default()
        }),
        ..Default::default()
      },
      Target,
    ))
    .id();

  commands.insert_resource(AgentSpawner {
    mesh: meshes.add(
      CylinderMeshBuilder {
        cylinder: Cylinder { radius: 0.5, half_height: 0.1 },
        resolution: 20,
        segments: 1,
      }
      .build(),
    ),
    material: materials.add(StandardMaterial {
      unlit: true,
      base_color: Color::SEA_GREEN,
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
      MaterialMeshBundle {
        transform: Transform::from_translation(position),
        mesh: self.mesh.clone(),
        material: self.material.clone(),
        ..Default::default()
      },
      AgentBundle {
        agent: Agent { radius: 0.5, max_velocity: 2.0 },
        archipelago_ref: ArchipelagoRef(self.archipelago_entity),
        target: AgentTarget::Entity(self.target_entity),
        state: Default::default(),
        velocity: Default::default(),
        desired_velocity: Default::default(),
      },
    ));
  }
}

/// Use the desired velocity as the agent's velocity.
fn update_agent_velocity(
  mut agent_query: Query<(&mut Velocity, &AgentDesiredVelocity)>,
) {
  for (mut velocity, desired_velocity) in agent_query.iter_mut() {
    velocity.0 = desired_velocity.velocity();
  }
}

/// Apply the agent's velocity to its position.
fn move_agent_by_velocity(
  time: Res<Time>,
  mut agent_query: Query<(&mut Transform, &GlobalTransform, &Velocity)>,
) {
  for (mut transform, global_transform, velocity) in agent_query.iter_mut() {
    let local_velocity =
      global_transform.affine().inverse().transform_vector3(velocity.0);
    transform.translation += local_velocity * time.delta_seconds();
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
    .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor))
    .and_then(|ray| {
      ray
        .intersect_plane(Vec3::ZERO, Plane3d { normal: Direction3d::Y })
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

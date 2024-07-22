use std::{collections::HashMap, sync::Arc};

use bevy::{
  color::palettes::css,
  input::common_conditions::input_just_pressed,
  prelude::*,
  sprite::{MaterialMesh2dBundle, Mesh2dHandle},
};
use bevy_landmass::{
  debug::{EnableLandmassDebug, Landmass2dDebugPlugin},
  nav_mesh::bevy_mesh_to_landmass_nav_mesh,
  prelude::*,
  AgentNodeTypeCostOverrides,
};
use landmass::NodeType;

fn main() {
  App::new()
    .add_plugins(DefaultPlugins)
    .add_plugins(Landmass2dPlugin::default())
    .add_plugins(Landmass2dDebugPlugin::default())
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
  nav_mesh: Handle<NavMesh2d>,
  slow_area: Rect,
  slow_node_type: bevy_landmass::NodeType,
}

fn convert_mesh(
  converters: Query<(Entity, &ConvertMesh)>,
  meshes: Res<Assets<Mesh>>,
  mut nav_meshes: ResMut<Assets<NavMesh2d>>,
  mut commands: Commands,
) {
  for (entity, converter) in converters.iter() {
    let Some(mesh) = meshes.get(&converter.mesh) else {
      continue;
    };

    let mut nav_mesh = bevy_mesh_to_landmass_nav_mesh(mesh).unwrap();
    mark_slow_polygons(&mut nav_mesh, converter.slow_area);

    let valid_nav_mesh = nav_mesh.validate().unwrap();
    nav_meshes.insert(
      &converter.nav_mesh,
      NavMesh2d {
        nav_mesh: Arc::new(valid_nav_mesh),
        type_index_to_node_type: HashMap::from([(
          1usize,
          converter.slow_node_type,
        )]),
      },
    );
    commands.entity(entity).remove::<ConvertMesh>();
  }
}

fn mark_slow_polygons(nav_mesh: &mut NavigationMesh2d, slow_area: Rect) {
  for (index, polygon) in nav_mesh.polygons.iter().enumerate() {
    let center = polygon.iter().map(|&i| nav_mesh.vertices[i]).sum::<Vec2>()
      / polygon.len() as f32;

    if !slow_area.contains(center) {
      continue;
    }
    nav_mesh.polygon_type_indices[index] = 1;
  }
}

fn setup(
  mut commands: Commands,
  mut meshes: ResMut<Assets<Mesh>>,
  mut materials: ResMut<Assets<ColorMaterial>>,
  nav_meshes: Res<Assets<NavMesh2d>>,
  asset_server: Res<AssetServer>,
) {
  commands.spawn(Camera2dBundle {
    transform: Transform::from_xyz(5.0, 0.0, 0.0),
    projection: OrthographicProjection {
      scaling_mode: bevy::render::camera::ScalingMode::FixedVertical(16.0),
      near: -1000.0,
      ..Default::default()
    },
    ..Default::default()
  });

  commands.spawn(TextBundle {
    text: Text::from_section(
      "LMB - Spawn agent\nShift+LMB - Spawn agent (fast on mud)\nRMB - Change target point\nF12 - Toggle debug view",
      TextStyle::default()
    ).with_justify(JustifyText::Right),
    style: Style {
      position_type: PositionType::Absolute,
      right: Val::Px(0.0),
      bottom: Val::Px(0.0),
      ..Default::default()
    },
    ..Default::default()
  });

  let slow_area = Rect::from_corners(
    Vec2::new(-3.99582, -2.89418),
    Vec2::new(3.30418, 4.00582),
  );
  commands.spawn(MaterialMesh2dBundle {
    transform: Transform::from_translation(slow_area.center().extend(1.0)),
    mesh: Mesh2dHandle(
      meshes.add(Rectangle { half_size: slow_area.size() * 0.5 }),
    ),
    material: materials.add(ColorMaterial {
      color: css::BROWN.with_alpha(0.5).into(),
      ..Default::default()
    }),
    ..Default::default()
  });

  let mut archipelago = Archipelago2d::new();
  let slow_node_type = archipelago.add_node_type(1000.0).unwrap();
  let archipelago_entity = commands.spawn(archipelago).id();

  // Spawn the islands.
  let mesh_1: Handle<Mesh> = asset_server.load("nav_mesh.glb#Mesh0/Primitive0");
  let nav_mesh_1 = nav_meshes.reserve_handle();
  commands.spawn((
    MaterialMesh2dBundle {
      mesh: Mesh2dHandle(mesh_1.clone()),
      material: materials.add(ColorMaterial {
        color: css::ANTIQUE_WHITE.into(),
        ..Default::default()
      }),
      ..Default::default()
    },
    Island2dBundle {
      archipelago_ref: ArchipelagoRef2d::new(archipelago_entity),
      island: Island,
      nav_mesh: nav_mesh_1.clone(),
    },
    ConvertMesh {
      mesh: mesh_1,
      nav_mesh: nav_mesh_1,
      slow_node_type,
      slow_area,
    },
  ));

  let mesh_2: Handle<Mesh> = asset_server.load("nav_mesh.glb#Mesh1/Primitive0");
  let nav_mesh_2 = nav_meshes.reserve_handle();
  commands.spawn((
    MaterialMesh2dBundle {
      mesh: Mesh2dHandle(mesh_2.clone()),
      material: materials.add(ColorMaterial {
        color: css::ANTIQUE_WHITE.into(),
        ..Default::default()
      }),
      transform: Transform::from_translation(Vec3::new(12.0, 0.0, 0.0)),
      ..Default::default()
    },
    Island2dBundle {
      archipelago_ref: ArchipelagoRef2d::new(archipelago_entity),
      island: Island,
      nav_mesh: nav_mesh_2.clone(),
    },
    ConvertMesh {
      mesh: mesh_2,
      nav_mesh: nav_mesh_2,
      slow_node_type,
      slow_area: Rect::EMPTY,
    },
  ));

  // Spawn the target.
  let target_entity = commands
    .spawn((
      MaterialMesh2dBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 6.0, 0.11)),
        mesh: Mesh2dHandle(meshes.add(Circle { radius: 0.25 })),
        material: materials.add(ColorMaterial {
          color: css::PURPLE.into(),
          ..Default::default()
        }),
        ..Default::default()
      },
      Target,
    ))
    .id();

  commands.insert_resource(AgentSpawner {
    mesh: meshes.add(Circle { radius: 0.5 }),
    material: materials.add(ColorMaterial {
      color: css::SEA_GREEN.into(),
      ..Default::default()
    }),
    archipelago_entity,
    target_entity,
    fast_material: materials.add(ColorMaterial {
      color: css::BLUE_VIOLET.into(),
      ..Default::default()
    }),
    slow_node_type,
  });
}

#[derive(Resource)]
struct AgentSpawner {
  mesh: Handle<Mesh>,
  material: Handle<ColorMaterial>,
  archipelago_entity: Entity,
  target_entity: Entity,
  fast_material: Handle<ColorMaterial>,
  slow_node_type: NodeType,
}

impl AgentSpawner {
  fn spawn(&self, position: Vec2, commands: &mut Commands, fast_agent: bool) {
    let entity = commands
      .spawn((
        MaterialMesh2dBundle {
          transform: Transform::from_translation(position.extend(0.1)),
          mesh: Mesh2dHandle(self.mesh.clone()),
          material: self.material.clone(),
          ..Default::default()
        },
        Agent2dBundle {
          agent: Agent { radius: 0.5, max_velocity: 2.0 },
          archipelago_ref: ArchipelagoRef2d::new(self.archipelago_entity),
          target: AgentTarget2d::Entity(self.target_entity),
          state: Default::default(),
          velocity: Default::default(),
          desired_velocity: Default::default(),
        },
      ))
      .id();

    if fast_agent {
      commands.entity(entity).insert((self.fast_material.clone(), {
        let mut node_cost_overrides = AgentNodeTypeCostOverrides::default();
        assert!(
          node_cost_overrides.set_node_type_cost(self.slow_node_type, 1.0)
        );
        node_cost_overrides
      }));
    }
  }
}

/// Use the desired velocity as the agent's velocity.
fn update_agent_velocity(
  mut agent_query: Query<(&mut Velocity2d, &AgentDesiredVelocity2d)>,
) {
  for (mut velocity, desired_velocity) in agent_query.iter_mut() {
    velocity.velocity = desired_velocity.velocity();
  }
}

/// Apply the agent's velocity to its position.
fn move_agent_by_velocity(
  time: Res<Time>,
  mut agent_query: Query<(&mut Transform, &GlobalTransform, &Velocity2d)>,
) {
  for (mut transform, global_transform, velocity) in agent_query.iter_mut() {
    let local_velocity = global_transform
      .affine()
      .inverse()
      .transform_vector3(velocity.velocity.extend(0.0));
    transform.translation += local_velocity * time.delta_seconds();
  }
}

/// Marker component for the target entity.
#[derive(Component)]
struct Target;

/// Handles clicks by spawning agents with LMB and moving the target with RMB.
fn handle_clicks(
  keys: Res<ButtonInput<KeyCode>>,
  buttons: Res<ButtonInput<MouseButton>>,
  window_query: Query<&Window>,
  camera_query: Query<(&Camera, &GlobalTransform)>,
  agent_spawner: Res<AgentSpawner>,
  target: Query<Entity, With<Target>>,
  mut commands: Commands,
) {
  let left = buttons.just_pressed(MouseButton::Left);
  let right = buttons.just_pressed(MouseButton::Right);
  let shift = keys.pressed(KeyCode::ShiftLeft);
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
        .intersect_plane(Vec3::ZERO, InfinitePlane3d { normal: Dir3::Z })
        .map(|d| ray.get_point(d))
    })
  else {
    return;
  };

  if left {
    agent_spawner.spawn(world_position.xy(), &mut commands, shift);
  }
  if right {
    commands
      .entity(target.single())
      .insert(Transform::from_translation(world_position.xy().extend(0.11)));
  }
}

/// System for toggling the `EnableLandmassDebug` resource.
fn toggle_debug(mut debug: ResMut<EnableLandmassDebug>) {
  **debug = !**debug;
}

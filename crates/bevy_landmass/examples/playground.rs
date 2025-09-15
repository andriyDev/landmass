use std::{f32::consts::TAU, sync::Arc};

use bevy::{
  color::palettes::css, input::common_conditions::input_just_pressed,
  prelude::*, scene::SceneInstanceReady,
};
use bevy_landmass::{
  Agent3d, AnimationLink, AnimationLinkReachedDistance, FromAgentRadius,
  NavMeshHandle, UsingAnimationLink,
  debug::{EnableLandmassDebug, Landmass3dDebugPlugin},
  nav_mesh::bevy_mesh_to_landmass_nav_mesh,
  prelude::*,
};

fn main() {
  App::new()
    .add_plugins((DefaultPlugins, MeshPickingPlugin))
    .add_plugins(Landmass3dPlugin::default())
    .add_plugins(Landmass3dDebugPlugin::default())
    .add_systems(Startup, setup)
    .add_systems(Update, convert_mesh)
    .add_systems(Update, toggle_debug.run_if(input_just_pressed(KeyCode::F12)))
    .add_systems(Update, rotate_by_keyboard)
    .add_systems(
      Update,
      (
        start_animation_link_for_agents,
        update_agent_jump,
        update_agent_velocity,
        move_agent_by_velocity,
        snap_agent_to_floor,
      )
        .chain(),
    )
    .add_observer(on_remove_agent_jumping)
    .add_observer(handle_clicks)
    .run();
}

// A utility to wait for a mesh to be loaded and convert the mesh to a nav mesh.
#[derive(Component)]
struct ConvertMesh {
  mesh: Handle<Mesh>,
  nav_mesh: Handle<NavMesh3d>,
}

fn convert_mesh(
  converters: Query<(Entity, &ConvertMesh)>,
  meshes: Res<Assets<Mesh>>,
  mut nav_meshes: ResMut<Assets<NavMesh3d>>,
  mut commands: Commands,
) {
  for (entity, converter) in converters.iter() {
    let Some(mesh) = meshes.get(&converter.mesh) else {
      continue;
    };

    let nav_mesh = bevy_mesh_to_landmass_nav_mesh(mesh).unwrap();
    let valid_nav_mesh = nav_mesh.validate().unwrap();
    nav_meshes.insert(
      &converter.nav_mesh,
      NavMesh3d { nav_mesh: Arc::new(valid_nav_mesh) },
    );
    commands.entity(entity).remove::<ConvertMesh>();
  }
}

fn setup(
  mut commands: Commands,
  mut meshes: ResMut<Assets<Mesh>>,
  mut materials: ResMut<Assets<StandardMaterial>>,
  nav_meshes: Res<Assets<NavMesh3d>>,
  asset_server: Res<AssetServer>,
) {
  commands.spawn((
    Rotate,
    Transform::default(),
    Visibility::default(),
    children![(
      Transform::from_xyz(10.0, 10.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
      Camera3d::default(),
    )],
  ));
  commands.spawn((
    Transform::from_xyz(10.0, 10.0, -10.0).looking_at(Vec3::ZERO, Vec3::Y),
    DirectionalLight { shadows_enabled: true, ..Default::default() },
  ));

  let message = "LMB - Spawn agent\nRMB - Change target point\nA/D - Look left/right\nF12 - Toggle debug view";
  commands.spawn((
    Text(message.into()),
    TextLayout { justify: JustifyText::Right, ..Default::default() },
    Node {
      position_type: PositionType::Absolute,
      right: Val::Px(0.0),
      bottom: Val::Px(0.0),
      ..Default::default()
    },
  ));

  let archipelago = Archipelago3d::new(AgentOptions::from_agent_radius(0.5));
  let archipelago_entity = commands.spawn(archipelago).id();

  commands
    .spawn(SceneRoot(asset_server.load("playground.glb#Scene1")))
    .observe(
      move |trigger: Trigger<SceneInstanceReady>,
            children: Query<&Children>,
            name: Query<&Name>,
            transforms: TransformHelper,
            mut commands: Commands| {
        // First find the relevant nodes that mark the animation links.
        let mut link_1_start = None;
        let mut link_1_end = None;
        let mut link_2_start = None;
        let mut link_2_end = None;
        for child in children.iter_descendants(trigger.target()) {
          let Ok(name) = name.get(child) else {
            continue;
          };
          if name.as_str() == "Link1Start" {
            link_1_start = Some(child);
          }
          if name.as_str() == "Link1End" {
            link_1_end = Some(child);
          }
          if name.as_str() == "Link2Start" {
            link_2_start = Some(child);
          }
          if name.as_str() == "Link2End" {
            link_2_end = Some(child);
          }
        }

        let link_1_start = link_1_start.unwrap();
        let link_1_end = link_1_end.unwrap();
        let link_2_start = link_2_start.unwrap();
        let link_2_end = link_2_end.unwrap();

        let entity_to_point = |entity: Entity| {
          transforms.compute_global_transform(entity).unwrap().translation()
        };
        let link_1_start_point = entity_to_point(link_1_start);
        let link_1_end_point = entity_to_point(link_1_end);
        let link_2_start_point = entity_to_point(link_2_start);
        let link_2_end_point = entity_to_point(link_2_end);

        // Create the animation links themselves.
        let edge_half = Vec3::Z * 0.75;
        commands.entity(link_1_start).insert((
          AnimationLink::<ThreeD> {
            start_edge: (
              link_1_start_point + edge_half,
              link_1_start_point - edge_half,
            ),
            end_edge: (
              link_1_end_point + edge_half,
              link_1_end_point - edge_half,
            ),
            cost: 4.0,
            kind: 0,
            bidirectional: false,
          },
          ArchipelagoRef3d::new(archipelago_entity),
        ));

        commands.entity(link_2_start).insert((
          AnimationLink::<ThreeD> {
            start_edge: (
              link_2_start_point + edge_half,
              link_2_start_point - edge_half,
            ),
            end_edge: (
              link_2_end_point + edge_half,
              link_2_end_point - edge_half,
            ),
            cost: 2.0,
            kind: 0,
            bidirectional: true,
          },
          ArchipelagoRef3d::new(archipelago_entity),
        ));
      },
    );

  // Spawn the islands.
  let mesh: Handle<Mesh> = asset_server.load("playground.glb#Mesh0/Primitive0");
  let nav_mesh = nav_meshes.reserve_handle();
  commands.spawn((
    Island3dBundle {
      archipelago_ref: ArchipelagoRef3d::new(archipelago_entity),
      island: Island,
      nav_mesh: NavMeshHandle(nav_mesh.clone()),
    },
    ConvertMesh { mesh, nav_mesh },
  ));

  // Spawn the target.
  let target_entity = commands
    .spawn((
      Mesh3d(meshes.add(Sphere { radius: 0.25 })),
      MeshMaterial3d(materials.add(StandardMaterial {
        base_color: css::PURPLE.into(),
        ..Default::default()
      })),
      Pickable::IGNORE,
      Target,
    ))
    .id();

  commands.insert_resource(AgentSpawner {
    mesh: meshes.add(Capsule3d { radius: 0.35, half_length: 0.5 }),
    material: materials.add(StandardMaterial {
      base_color: css::WHITE_SMOKE.into(),
      ..Default::default()
    }),
    archipelago_entity,
    target_entity,
  });
}

#[derive(Component)]
struct Rotate;

/// Rotate on object around using the A/D or left/right arrows.
fn rotate_by_keyboard(
  keys: Res<ButtonInput<KeyCode>>,
  time: Res<Time>,
  mut rotaters: Query<&mut Transform, With<Rotate>>,
) {
  let mut rotate_dir = 0.0;
  if keys.pressed(KeyCode::KeyA) || keys.pressed(KeyCode::ArrowLeft) {
    rotate_dir -= 1.0;
  }
  if keys.pressed(KeyCode::KeyD) || keys.pressed(KeyCode::ArrowRight) {
    rotate_dir += 1.0;
  }

  for mut transform in rotaters.iter_mut() {
    transform.rotation *=
      Quat::from_rotation_y(rotate_dir * TAU * 0.25 * time.delta_secs());
  }
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
      Visibility::default(),
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
      AnimationLinkReachedDistance(0.1),
      children![(
        Transform::from_xyz(0.0, 0.5, 0.0),
        Mesh3d(self.mesh.clone()),
        MeshMaterial3d(self.material.clone()),
        Pickable::IGNORE,
      )],
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
  mut agent_query: Query<
    (&mut Transform, &GlobalTransform, &Velocity3d),
    Without<AgentJumping>,
  >,
) {
  for (mut transform, global_transform, velocity) in agent_query.iter_mut() {
    let local_velocity =
      global_transform.affine().inverse().transform_vector3(velocity.velocity);
    transform.translation += local_velocity * time.delta_secs();
  }
}

/// Snap the agent to the floor so that they can go up and down ramps.
fn snap_agent_to_floor(
  mut ray_cast: MeshRayCast,
  mut agents: Query<&mut Transform, (With<Agent3d>, Without<AgentJumping>)>,
  pickable: Query<&Pickable>,
) {
  let filter = |entity| {
    pickable.get(entity).map(|pickable| pickable.is_hoverable).unwrap_or(true)
  };
  for mut transform in agents.iter_mut() {
    let Some((_, hit)) = ray_cast
      .cast_ray(
        Ray3d::new(transform.translation + Vec3::Y * 0.5, Dir3::NEG_Y),
        &MeshRayCastSettings::default()
          .always_early_exit()
          .with_filter(&filter),
      )
      .first()
    else {
      continue;
    };
    if hit.distance > 2.0 {
      continue;
    }
    transform.translation.y = hit.point.y;
  }
}

fn start_animation_link_for_agents(
  agents: Query<(Entity, &ReachedAnimationLink3d), Without<AgentJumping>>,
  mut commands: Commands,
) {
  for (agent, reached_animation_link) in agents.iter() {
    commands.entity(agent).insert(AgentJumping {
      start: reached_animation_link.start_point,
      end: reached_animation_link.end_point,
      timer: Timer::from_seconds(1.0, TimerMode::Once),
    });
  }
}

#[derive(Component)]
#[require(UsingAnimationLink)]
struct AgentJumping {
  start: Vec3,
  end: Vec3,
  timer: Timer,
}

fn on_remove_agent_jumping(
  trigger: Trigger<OnRemove, AgentJumping>,
  mut commands: Commands,
) {
  commands.entity(trigger.target()).remove::<UsingAnimationLink>();
}

fn update_agent_jump(
  time: Res<Time>,
  mut agent_jumps: Query<(Entity, &mut AgentJumping, &mut Transform)>,
  mut commands: Commands,
) {
  for (agent, mut jump, mut transform) in agent_jumps.iter_mut() {
    jump.timer.tick(time.delta());

    let alpha = jump.timer.fraction();
    let delta = jump.end - jump.start;
    let delta_flat = Vec3::new(delta.x, 0.0, delta.z);

    // The jump will peak 1 unit above the highest point.
    let max_height = 1.0 + delta.z.max(0.0);
    // This is a quadratic which passes through (0,0), (1,delta.y), and where
    // the vertex reaches max_height.
    let a = delta.y
      - 2.0 * max_height
      - 2.0 * (max_height * max_height - max_height * delta.y).sqrt();
    let b = delta.y - a;
    // c = 0 because we pass through (0,0).
    let delta_height = a * alpha * alpha + b * alpha;

    transform.translation =
      delta_flat * alpha + Vec3::Y * delta_height + jump.start;

    if jump.timer.finished() {
      commands.entity(agent).remove::<AgentJumping>();
    }
  }
}

/// Marker component for the target entity.
#[derive(Component)]
struct Target;

/// Handles clicks by spawning agents with LMB and moving the target with RMB.
fn handle_clicks(
  mut trigger: Trigger<Pointer<Pressed>>,
  agent_spawner: Res<AgentSpawner>,
  mut target: Single<&mut Transform, With<Target>>,
  mut commands: Commands,
) {
  let Some(world_position) = trigger.hit.position else {
    return;
  };
  trigger.propagate(false);
  match trigger.button {
    PointerButton::Primary => {
      agent_spawner.spawn(world_position, &mut commands);
    }
    PointerButton::Secondary => {
      target.translation = world_position;
    }
    PointerButton::Middle => {}
  }
}

/// System for toggling the `EnableLandmassDebug` resource.
fn toggle_debug(mut debug: ResMut<EnableLandmassDebug>) {
  **debug = !**debug;
}

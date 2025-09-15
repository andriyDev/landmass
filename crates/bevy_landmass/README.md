# bevy_landmass

A plugin for [Bevy](https://bevyengine.org) to allow using
[landmass](https://github.com/andriyDev/landmass) conveniently.

## Overview

`bevy_landmass` allows using a navigation mesh to determine the desired move
direction for characters using pathfinding.

To use `bevy_landmass`:

1. Add `LandmassPlugin` to your app.
2. Spawn an entity with an `Archipelago` component.
3. Spawn an entity with an `IslandBundle`, a `TransformBundle` (or any other
   bundle which includes a `Transform` and `GlobalTransform`), and an
   `IslandNavMesh` component.
4. Spawn entities with the `AgentBundle` and a `TransformBundle` (or any other
   bundle which includes a `Transform` and `GlobalTransform`).

Note the `Archipelago` can be created later, even if the agents/islands already
have an `ArchipelagoRef` to it. Agents/islands will be added once the
`Archipelago` exists.

## Example

```rust
use std::sync::Arc;

use bevy::{app::AppExit, prelude::*};
use bevy_landmass::{prelude::*, NavMeshHandle};

fn main() {
  App::new()
    .add_plugins(MinimalPlugins)
    .add_plugins(AssetPlugin::default())
    .add_plugins(TransformPlugin)
    .add_plugins(Landmass2dPlugin::default())
    .add_systems(Startup, set_up_scene)
    .add_systems(Update, print_desired_velocity.after(LandmassSystemSet::Output))
    .add_systems(Update, quit.after(print_desired_velocity))
    .run();
}

fn set_up_scene(
  mut commands: Commands,
  mut nav_meshes: ResMut<Assets<NavMesh2d>>,
) {
  let archipelago_id = commands
    .spawn(Archipelago2d::new(ArchipelagoOptions::from_agent_radius(0.5)))
    .id();

  let nav_mesh_handle = nav_meshes.reserve_handle();

  commands
    .spawn((
      Island2dBundle {
        island: Island,
        archipelago_ref: ArchipelagoRef2d::new(archipelago_id),
        nav_mesh: NavMeshHandle(nav_mesh_handle.clone()),
      },
    ));

  // The nav mesh can be populated in another system, or even several frames
  // later.
  let nav_mesh = Arc::new(NavigationMesh2d {
      vertices: vec![
        Vec2::new(1.0, 1.0),
        Vec2::new(2.0, 1.0),
        Vec2::new(2.0, 2.0),
        Vec2::new(1.0, 2.0),
        Vec2::new(2.0, 3.0),
        Vec2::new(1.0, 3.0),
        Vec2::new(2.0, 4.0),
        Vec2::new(1.0, 4.0),
      ],
      polygons: vec![
        vec![0, 1, 2, 3],
        vec![3, 2, 4, 5],
        vec![5, 4, 6, 7],
      ],
      polygon_type_indices: vec![0, 0, 0],
      height_mesh: None,
    }.validate().expect("is valid"));
  nav_meshes.insert(&nav_mesh_handle, NavMesh2d { nav_mesh });

  commands.spawn((
    Transform::from_translation(Vec3::new(1.5, 1.5, 0.0)),
    Agent2dBundle {
      agent: Default::default(),
      settings: AgentSettings {
        radius: 0.5,
        desired_speed: 1.0,
        max_speed: 2.0,
      },
      archipelago_ref: ArchipelagoRef2d::new(archipelago_id),
    },
    AgentTarget2d::Point(Vec2::new(1.5, 3.5)),
  ));
}

fn print_desired_velocity(query: Query<(Entity, &AgentDesiredVelocity2d)>) {
  for (entity, desired_velocity) in query.iter() {
    println!(
      "entity={:?}, desired_velocity={}",
      entity,
      desired_velocity.velocity());
  }
}

fn quit(mut exit: EventWriter<AppExit>) {
  // Quit so doctests pass.
  exit.send(AppExit::Success);
}
```

## License

License under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

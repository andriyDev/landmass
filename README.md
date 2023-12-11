# bevy_landmass

A plugin for [Bevy](https://bevyengine.org) to allow using
[landmass](https://github.com/andriyDev/landmass) conveniently.

## Overview

`bevy_landmass` allows using a navigation mesh to determine the desired move
direction for characters using pathfinding.

To use `bevy_landmass`:
1) Add `LandmassPlugin` to your app.
2) Spawn an entity with an `Archipelago` component.
3) Spawn an entity with an `IslandBundle`, and an `IslandNavMesh` component.
3) Spawn entities with the `AgentBundle` and a `TransformBundle` (or any other
bundle which includes a `Transform` and `GlobalTransform`).

Note the `Archipelago` can be created later, even if the agents/islands already
have an `ArchipelagoRef` to it. Agents/islands will be added once the
`Archipelago` exists.

## Example

```rust
use std::sync::Arc;

use bevy::{app::AppExit, prelude::*};
use bevy_landmass::prelude::*;

fn main() {
  App::new()
    .add_plugins(MinimalPlugins)
    .add_plugins(AssetPlugin::default())
    .add_plugins(TransformPlugin)
    .add_plugins(LandmassPlugin)
    .add_systems(Startup, set_up_scene)
    .add_systems(Update, print_desired_velocity.after(LandmassSystemSet::Output))
    .add_systems(Update, quit.after(print_desired_velocity))
    .run();
}

fn set_up_scene(
  mut commands: Commands,
  mut nav_meshes: ResMut<Assets<NavMesh>>,
) {
  let archipelago_id = commands.spawn(Archipelago::new()).id();

  let nav_mesh_handle = nav_meshes
    .get_handle_provider()
    .reserve_handle()
    .typed::<NavMesh>();

  commands
    .spawn(TransformBundle::default())
    .insert(IslandBundle {
      island: Island,
      archipelago_ref: ArchipelagoRef(archipelago_id),
      nav_mesh: Default::default(),
    })
    .insert(nav_mesh_handle.clone());
  
  // The nav mesh can be populated in another system, or even several frames
  // later.
  let nav_mesh = Arc::new(landmass::NavigationMesh {
      mesh_bounds: None,
      vertices: vec![
        glam::Vec3::new(1.0, 0.0, 1.0),
        glam::Vec3::new(2.0, 0.0, 1.0),
        glam::Vec3::new(2.0, 0.0, 2.0),
        glam::Vec3::new(1.0, 0.0, 2.0),
        glam::Vec3::new(2.0, 0.0, 3.0),
        glam::Vec3::new(1.0, 0.0, 3.0),
        glam::Vec3::new(2.0, 0.0, 4.0),
        glam::Vec3::new(1.0, 0.0, 4.0),
      ],
      polygons: vec![
        vec![0, 1, 2, 3],
        vec![3, 2, 4, 5],
        vec![5, 4, 6, 7],
      ],
    }.validate().expect("is valid"));
  nav_meshes.insert(nav_mesh_handle, NavMesh(nav_mesh));

  commands.spawn(TransformBundle {
    local: Transform::from_translation(Vec3::new(1.5, 0.0, 1.5)),
    ..Default::default()
  }).insert(AgentBundle {
    agent: Agent {
      radius: 0.5,
      max_velocity: 1.0,
    },
    archipelago_ref: ArchipelagoRef(archipelago_id),
    target: AgentTarget::Point(Vec3::new(1.5, 0.0, 3.5)),
    velocity: Default::default(),
    state: Default::default(),
    desired_velocity: Default::default(),
  });
}

fn print_desired_velocity(query: Query<(Entity, &AgentDesiredVelocity)>) {
  for (entity, desired_velocity) in query.iter() {
    println!(
      "entity={:?}, desired_velocity={}",
      entity,
      desired_velocity.velocity());
  }
}

fn quit(mut exit: EventWriter<AppExit>) {
  // Quit so doctests pass.
  exit.send(AppExit);
}
```

## License

License under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

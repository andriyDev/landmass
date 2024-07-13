# landmass

A Rust crate to provide a navigation system for video game characters to walk
around levels.

## What is a navigation system?

A navigation system is essentially the collection of tools needed for robust
agent movement in video games. This generally involves 4 things:

- Path finding (e.g. A-star)
- Path simplification (e.g. SSFA)
- Steering (e.g. boids)
- Local collision avoidance

In addition, managing agents and the navigation meshes they walk on can be
cumbersome, so a navigation system ideally will handle that for you.

Generally it is difficult to find a full, free system to handle all of these for
you, and the goal is for `landmass` to work relatively easily with other
languages so it can be used anywhere.

## Overview

`landmass` has four major components: `Archipelago`s, `Island`s, `Agent`s, and
`Character`s. An `Archipelago` is composed of several `Island`s, as well as the
`Agent`s and `Character`s that travel across those `Island`s. Each `Island`
holds a single [navigation mesh](https://en.wikipedia.org/wiki/Navigation_mesh).
Each game character (controlled by AI) should correspond to one `Agent`. Player
characters or other characters **not** controlled by AI should correspond to one
`Character`. To start using `landmass`:

1. Create an `Archipelago`.
2. Create an `Island`.
3. Assign a `ValidNavigationMesh` to the `Island`.
2. Add `Agent`s and `Character`s to the `Archipelago`.

Each frame of the game:

1. Set the position and velocity of each game character to its corresponding
`Agent` or `Character`.
2. Call `update` on the `Archipelago`.
3. Use the desired move from each `Agent` to inform the corresponding game
character where it should move.

Note: `landmass` intentionally does not update the `Agent`s position itself.
Generally, characters are moved using some other method (like a physics
simulation) rather than just moving the character, so moving the `Agent` would
be confusing.

## Example

```rust
use glam::Vec3;
use landmass::*;
use std::sync::Arc;

let mut archipelago = Archipelago::<XYZ>::new();

let nav_mesh = NavigationMesh {
  vertices: vec![
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(15.0, 0.0, 0.0),
    Vec3::new(15.0, 15.0, 0.0),
    Vec3::new(0.0, 15.0, 0.0),
  ],
  polygons: vec![vec![0, 1, 2, 3]],
};

let valid_nav_mesh = Arc::new(
  nav_mesh.validate().expect("Validation succeeds")
);

let island_id = archipelago
  .add_island()
  .set_nav_mesh(
    Transform { translation: Vec3::ZERO, rotation: 0.0 },
    valid_nav_mesh,
  )
  .id();

let agent_1 = archipelago.add_agent({
  let mut agent = Agent::create(
    /* position= */ Vec3::new(1.0, 1.0, 0.0),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 1.0,
    /* max_velocity= */ 1.0,
  );
  agent.current_target = Some(Vec3::new(11.0, 1.1, 0.0));
  agent.target_reached_condition = TargetReachedCondition::Distance(Some(0.01));
  agent
});
let agent_2 = archipelago.add_agent({
  let mut agent = Agent::create(
    /* position= */ Vec3::new(11.0, 1.1, 0.0),
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 1.0,
    /* max_velocity= */ 1.0,
  );
  agent.current_target = Some(Vec3::new(1.0, 1.0, 0.0));
  agent.target_reached_condition = TargetReachedCondition::Distance(Some(0.01));
  agent
});

for i in 0..200 {
  let delta_time = 1.0 / 10.0;
  archipelago.update(delta_time);

  for agent_id in archipelago.get_agent_ids().collect::<Vec<_>>() {
    let agent = archipelago.get_agent_mut(agent_id).unwrap();
    agent.velocity = *agent.get_desired_velocity();
    agent.position += agent.velocity * delta_time;
  }
}

assert!(archipelago
  .get_agent(agent_1)
  .unwrap()
  .position
  .abs_diff_eq(Vec3::new(11.0, 1.1, 0.0), 0.1));
assert!(archipelago
  .get_agent(agent_2)
  .unwrap()
  .position
  .abs_diff_eq(Vec3::new(1.0, 1.0, 0.0), 0.1));
```

## License

License under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

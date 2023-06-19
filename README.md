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

`landmass` has two major components: `Archipelago`s and `Agent`s. An
`Archipelago` is essentially one
[navigation mesh](https://en.wikipedia.org/wiki/Navigation_mesh) and the
`Agent`s on that navigation mesh. Each game character (controlled by AI) should
correspond to one `Agent`. To start using `landmass`:

1. Create an `Archipelago` from a `ValidNavigationMesh`.
2. Add `Agent`s to the `Archipelago`.

Each frame of the game:

1. Set the position and velocity of each game character to its corresponding
`Agent`.
2. Call `update` on the `Archipelago`.
3. Use the desired move from each `Agent` to inform the corresponding game
character where it should move.

Note: `landmass` intentionally does not update the `Agent`s position itself.
Generally, characters are moved using some other method (like a physics
simulation) rather than just moving the character, so moving the `Agent` would
be confusing.

## License

Licensed under the [MIT license](LICENSE).

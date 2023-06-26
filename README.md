# bevy_landmass

A plugin for [Bevy](https://bevyengine.org) to allow using
[landmass](https://github.com/andriyDev/landmass) conveniently.

## Overview

`bevy_landmass` allows using a navigation mesh to determine the desired move
direction for characters using pathfinding.

To use `bevy_landmass`:
1) Add `LandmassPlugin` to your app.
2) Spawn an entity with an `Archipelago` component.
3) Spawn entities with the `AgentBundle` and a `TransformBundle` (or any other
bundle which includes a `Transform` and `GlobalTransform`).

Note the `Archipelago` can be created later, even if the agents already have an
`ArchipelagoRef` to it. Agents will be added once the `Archipelago` exists.

See the docs for an example.

## License

Licensed under the [MIT license](LICENSE).

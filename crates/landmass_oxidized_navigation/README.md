# landmass_oxidized_navigation

An integration between `bevy_landmass` and
[`oxidized_navigation`](https://github.com/TheGrimsey/oxidized_navigation).

## Overview

`landmass` is the core navigation system with `bevy_landmass` as the Bevy plugin
for it. With both of these crates, providing the navigation meshes to `landmass`
is still the responsibility of the user.

Thankfully, `oxidized_navigation` provides navigation mesh generation based on
your world's colliders. `landmass_oxidized_navigation` exists to automatically
convert your `oxidized_navigation` meshes into `bevy_landmass` navigation
meshes, so navigation meshes are created automatically!

To use `landmass_oxidized_navigation`:
1) Add your physics plugin of choice (compatible with `oxidized_navigation`).
2) Add the `OxidizedNavigationPlugin`.
3) Add the `Landmass3dPlugin`.
4) Add the `LandmassOxidizedNavigationPlugin`.
5) Create an `Archipelago3d` with the `OxidizedArchipelago` component.
6) Add agents and start using them!

## License

License under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

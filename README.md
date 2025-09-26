# landmass

A Rust crate to provide a navigation system for video game characters to walk
around levels.

## What is a navigation system?

A navigation system is essentially the collection of tools needed for robust
agent movement in video games. This generally involves 4 things:

- Path finding (e.g., A-star)
- Path simplification (e.g., SSFA)
- Steering (e.g., boids)
- Local collision avoidance

In addition, managing agents and the navigation meshes they walk on can be
cumbersome, so a navigation system ideally will handle that for you.

Generally it is difficult to find a full, free system to handle all of these for
you, and the goal is for `landmass` to work relatively easily with other
languages so it can be used anywhere.

## Crates

- [`landmass`](crates/landmass/README.md): The core crate that other crates
  build off of.
- [`bevy_landmass`](crates/bevy_landmass/README.md): A plugin for Bevy to
  provide a convenient API to `landmass`.
- [`landmass_rerecast`](crates/landmass_rerecast/README.md): A plugin to
  integrate `bevy_landmass` with `bevy_rerecast`.

## Frequently Asked Questions

See the [FAQ](FAQ.md).

## License

License under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

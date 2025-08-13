# Change Log

## `landmass` [Unreleased] / `bevy_landmass` [Unreleased] - yyyy-mm-dd

### Features

- Better paths!
  - Previously, the pathfinding algorithm would compute paths assuming that agents walk through the
    centers of polygons. However, after applying path simplification, agents would travel directly
    from edge to edge. This caused a stark difference between what the pathfinding did and what the
    agent actually does, resulting in **very** confusing paths in some cases.
  - Now, pathfinding assumes agents walk through the centers of edges! This is not strictly correct
    (for that we'd need to use something like [polyanya](https://github.com/vleue/polyanya)), but it
    is **much** closer.
- Support for "height meshes".
  - When constructing a navigation mesh, it is common to be very "coarse". For example, if there is
    a staircase, you probably don't need the agent to navigate up each step individually. So rather
    than making each step a polygon, we can just represent the entire staircase as one big polygon.
    This results in a problem though when the polygon no longer accurately represents the height of
    the stairs! The agent is likely to be walking along the actual geometry, so it is possible that
    their position is too far below or too high above the polygon, resulting in the agent
    erroneously being marked "off the nav mesh".
  - To handle this case, we now support height meshes. While pathfinding continues to occur on the
    regular navigation mesh, the height mesh is used for sampling positions. This means we keep the
    performance benefits of a coarse navigation mesh, but avoid unfortunate sampling situations
    where agents on the geometry won't pathfind.
- Automatic flipping for clockwise coordinate systems.
  - When implementing a coordinate system, it is possible to "flip" the axes. This means that
    counter-clockwise polygons will become clockwise polygons once converted to `landmass`
    coordinates (which is invalid according to `landmass`). This can be confusing to users (e.g.,
    when switching from `bevy_landmass::TwoD` to `bevy_landmass::ThreeD`), or at least to me!
  - `CoordinateSystem`s can now specify whether to flip polygons after converting to `landmass` or
    not. This allows you to define navigation meshes counter-clockwise in your coordinate system,
    and have that same navigation mesh properly converted to `landmass`.
  - Note: counter-clockwise is defined as having a cross-product that points "up" (we love the
    right-hand rule!). If converting to `landmass` coordinates would result in a cross product
    pointing towards negative Z, you should enable flipping!
- No more `type_index_to_node_type`!
  - In previous versions, nav meshes stored a type index, but this type index needed to be
    "translated" into an actual node type, which you could then assign a cost to. This was a lot of
    steps! The more pressing concern was that it was difficult to apply that "translation" to the
    node type when loading a nav mesh. Node types needed to be created on the archipelago itself,
    and then when loading up a nav mesh, you would need to set the node type mappings on it (which
    could be difficult from `bevy_landmass` for example).
  - Now, node types are gone, and you can just set the cost of a type index! Note this means that
    all your nav meshes must share the same meaning for each type index (e.g., 1 means road in every
    nav mesh).
- `bevy_landmass`: More configurable debug rendering.
  - While the `bevy_landmass` API allows you to create your own `DebugDrawer`, the default one is
    good enough for a **lot** of cases. To further expand its usefulness, we have replaced
    `LandmassGizmoConfigGroup` with `LandmassGizmos`, which contains many more settings! This allows
    you to change the colors of the various features we render, as well as disabling them entirely!
    To change it, you can use `App::insert_gizmo_config`.

### Migration Guide

- `NavigationMesh` now has a `height_mesh` field. To maintain the existing behaviour, set it to
  `None`.

  ```rust
  NavigationMesh {
    polygons,
    vertices,
    polygon_type_indices,
    height_mesh: None, # NEW
  }
  ```

- `CoordinateSystem` now has a `const FLIP_POLYGONS: bool`. To maintain existing behaviour, set it
  to `false`.
- `NodeType` has been removed. Replace it with the corresponding type indices. This could require
  mutating any serialized nav meshes to ensure their type indices are "globally-unique" relative to
  other nav meshes.
- All node type functions (e.g., `add_node_type`, `get_node_type_cost`) have been replaced by
  versions directly relevant to type indices.
- `bevy_landmass`: The `ThreeD` coordinate system is now flipped! In previous versions, navigation
  meshes in 3D were expected to have clockwise-oriented polygons. Now, they are expected to be
  counter-clockwise oriented. This puts it in sync with every other coordinate system in `landmass`/
  `bevy_landmass`. To maintain exsting behaviour, reverse the order of the vertex indices in your
  polygons.
- `bevy_landmass`: `LandmassGizmoConfigGroup` has been replaced by `LandmassGizmos`. If you were
  configuring the `LandmassGizmoConfigGroup`, replace it with `LandmassGizmos::default()` (since it
  is no longer a ZST).
- `bevy_landmass`: `NavMesh` no longer contains a `type_index_to_node_type` field. Just remove it!

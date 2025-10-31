use std::time::Duration;

use bevy::{
  MinimalPlugins,
  time::TimeUpdateStrategy,
  transform::{TransformPlugin, components::Transform},
};
use bevy_app::App;
use bevy_asset::{AssetApp, AssetEvent, AssetPlugin, Assets};
use bevy_ecs::message::Messages;
use bevy_landmass::{
  Agent, Agent3dBundle, AgentSettings, AgentState, AgentTarget3d,
  Archipelago3d, ArchipelagoOptions, ArchipelagoRef, FromAgentRadius, Island,
  Landmass3dPlugin, NavMesh3d,
};
use bevy_math::{U16Vec3, Vec3};
use bevy_rerecast::{
  RerecastPlugin,
  rerecast::{Aabb3d, AreaType, DetailNavmesh, PolygonNavmesh, SubMesh},
};
use googletest::{
  expect_eq, expect_false, expect_that, expect_true, matchers::*,
};

use crate::{LandmassRerecastPlugin, NavMeshHandle3d};

/// Creates a rerecast nav mesh that is just a unit square.
fn square_rerecast_nav_mesh() -> bevy_rerecast::Navmesh {
  bevy_rerecast::Navmesh {
    polygon: PolygonNavmesh {
      aabb: Aabb3d { min: Vec3::ZERO, max: Vec3::ONE * 100.0 },
      cell_size: 1.0,
      cell_height: 1.0,
      max_vertices_per_polygon: 4,
      vertices: vec![
        U16Vec3::new(0, 0, 0),
        U16Vec3::new(1, 0, 0),
        U16Vec3::new(1, 0, 1),
        U16Vec3::new(0, 0, 1),
      ],
      polygons: vec![3, 2, 1, 0],
      areas: vec![AreaType(0)],
      polygon_neighbors: vec![],
      regions: vec![],
      border_size: 0,
      flags: vec![],
      max_edge_error: 0.0,
    },
    detail: DetailNavmesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, 1.0),
      ],
      triangles: vec![[0, 2, 1], [2, 0, 3]],
      meshes: vec![SubMesh {
        base_vertex_index: 0,
        vertex_count: 4,
        base_triangle_index: 0,
        triangle_count: 2,
      }],
      triangle_flags: vec![],
    },
    settings: Default::default(),
  }
}

#[googletest::test]
fn landmass_mesh_created_once_rerecast_mesh_is_added_and_updated() {
  let mut app = App::new();
  app
    .add_plugins((
      MinimalPlugins,
      TransformPlugin,
      AssetPlugin::default(),
      Landmass3dPlugin::default(),
      RerecastPlugin::default(),
      LandmassRerecastPlugin::default(),
    ))
    .insert_resource(TimeUpdateStrategy::ManualDuration(
      Duration::from_micros(
        // Bevy's default fixed timestep
        15625,
      ),
    ));
  app.finish();
  app.update();

  let rerecast_handle =
    app.world().resource::<Assets<bevy_rerecast::Navmesh>>().reserve_handle();

  // Once the nav meshes are converted, we need to know whether the mesh is
  // actually correct, so set up a situation that will show us that.
  let archipelago = app
    .world_mut()
    .spawn(Archipelago3d::new(ArchipelagoOptions::from_agent_radius(1.0)))
    .id();
  let island = app
    .world_mut()
    .spawn(crate::Island3dBundle {
      island: Island,
      archipelago_ref: ArchipelagoRef::new(archipelago),
      nav_mesh: NavMeshHandle3d(rerecast_handle.clone()),
    })
    .id();
  let agent = app
    .world_mut()
    .spawn((
      Agent3dBundle {
        agent: Agent::default(),
        archipelago_ref: ArchipelagoRef::new(archipelago),
        settings: AgentSettings {
          desired_speed: 1.0,
          max_speed: 1.0,
          radius: 1.0,
        },
      },
      Transform::from_xyz(0.5, 0.0, 0.5),
      AgentTarget3d::Point(Vec3::new(1.5, 0.0, 0.5)),
    ))
    .id();

  let landmass_handle = app
    .world()
    .entity(island)
    .get::<bevy_landmass::NavMeshHandle3d>()
    .expect("landmass handle should be added by the rerecast version")
    .0
    .clone();

  // Asset events are sent in PostUpdate, so we need two frames for landmass
  // systems to see any new assets.
  app.update();
  app.update();

  // The landmass mesh is still not populated.
  expect_false!(
    app
      .world()
      .resource::<Assets<bevy_landmass::NavMesh3d>>()
      .contains(&landmass_handle)
  );
  // The agent is not on the nav mesh.
  expect_that!(
    app.world().entity(agent).get::<AgentState>(),
    some(eq(&AgentState::AgentNotOnNavMesh))
  );

  let rerecast_mesh = square_rerecast_nav_mesh();
  app
    .world_mut()
    .resource_mut::<Assets<bevy_rerecast::Navmesh>>()
    .insert(&rerecast_handle, rerecast_mesh)
    .unwrap();

  app.update();
  app.update();

  // We now have a landmass mesh!
  expect_true!(
    app
      .world()
      .resource::<Assets<bevy_landmass::NavMesh3d>>()
      .get(&landmass_handle)
      .is_some(),
    "landmass mesh is set"
  );
  // The rerecast mesh was consumed (so we don't waste RAM).
  expect_false!(
    app
      .world()
      .resource::<Assets<bevy_rerecast::Navmesh>>()
      .get(&rerecast_handle)
      .is_some(),
    "landmass mesh is set"
  );
  // The agent is now on the landmass mesh! But their target is still not on the
  // nav mesh.
  expect_that!(
    app.world().entity(agent).get::<AgentState>(),
    some(eq(&AgentState::TargetNotOnNavMesh))
  );

  // We now mutate the rerecast nav mesh. This should result in the nav mesh
  // being modified.
  let rerecast_mesh = bevy_rerecast::Navmesh {
    polygon: PolygonNavmesh {
      aabb: Aabb3d { min: Vec3::ZERO, max: Vec3::ONE * 100.0 },
      cell_size: 1.0,
      cell_height: 1.0,
      max_vertices_per_polygon: 4,
      vertices: vec![
        U16Vec3::new(0, 0, 0),
        U16Vec3::new(1, 0, 0),
        U16Vec3::new(1, 0, 1),
        U16Vec3::new(0, 0, 1),
        U16Vec3::new(2, 0, 0),
        U16Vec3::new(2, 0, 1),
      ],
      polygons: vec![3, 2, 1, 0, 5, 4, 1, 2],
      areas: vec![AreaType(0), AreaType(0)],
      polygon_neighbors: vec![],
      regions: vec![],
      border_size: 0,
      flags: vec![],
      max_edge_error: 0.0,
    },
    detail: DetailNavmesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 1.0),
      ],
      triangles: vec![[0, 2, 1], [2, 0, 3], [0, 2, 1], [2, 0, 3]],
      meshes: vec![
        SubMesh {
          base_vertex_index: 0,
          vertex_count: 4,
          base_triangle_index: 0,
          triangle_count: 2,
        },
        SubMesh {
          base_vertex_index: 4,
          vertex_count: 4,
          base_triangle_index: 2,
          triangle_count: 2,
        },
      ],
      triangle_flags: vec![],
    },
    settings: Default::default(),
  };
  app
    .world_mut()
    .resource_mut::<Assets<bevy_rerecast::Navmesh>>()
    .insert(&rerecast_handle, rerecast_mesh)
    .unwrap();

  app.update();
  app.update();

  // We still have a landmass mesh.
  expect_true!(
    app
      .world()
      .resource::<Assets<bevy_landmass::NavMesh3d>>()
      .get(&landmass_handle)
      .is_some(),
    "landmass mesh is set"
  );
  // The rerecast mesh was consumed again.
  expect_false!(
    app
      .world()
      .resource::<Assets<bevy_rerecast::Navmesh>>()
      .get(&rerecast_handle)
      .is_some(),
    "landmass mesh is set"
  );
  // The agent is now on the mesh and its target is on the mesh, since the new
  // nav mesh is big enough.
  expect_that!(
    app.world().entity(agent).get::<AgentState>(),
    some(eq(&AgentState::Moving))
  );
}

#[googletest::test]
fn existing_rerecast_mesh_is_converted() {
  let mut app = create_test_app();

  let rerecast_mesh = square_rerecast_nav_mesh();
  let rerecast_handle = app
    .world_mut()
    .resource_mut::<Assets<bevy_rerecast::Navmesh>>()
    .add(rerecast_mesh);

  // Asset events are sent in PostUpdate, so we need two frames for landmass
  // systems to see any new assets. In this case, nothing should happen, so just
  // let the asset event go away.
  app.update();
  app.update();

  // Clear out the asset events in the pipe so we are sure we're testing the
  // case of handling an existing asset.
  app
    .world_mut()
    .resource_mut::<Messages<AssetEvent<bevy_rerecast::Navmesh>>>()
    .clear();

  let entity =
    app.world_mut().spawn(NavMeshHandle3d(rerecast_handle.clone())).id();

  let landmass_handle = app
    .world()
    .entity(entity)
    .get::<bevy_landmass::NavMeshHandle3d>()
    .expect("landmass handle should be added by the rerecast version")
    .0
    .clone();

  app.update();

  // We now have a landmass mesh!
  expect_true!(
    app
      .world()
      .resource::<Assets<bevy_landmass::NavMesh3d>>()
      .get(&landmass_handle)
      .is_some(),
    "landmass mesh is set"
  );
  // The rerecast mesh was consumed (so we don't waste RAM).
  expect_false!(
    app
      .world()
      .resource::<Assets<bevy_rerecast::Navmesh>>()
      .get(&rerecast_handle)
      .is_some(),
    "landmass mesh is set"
  );
}

#[googletest::test]
fn shared_rerecast_mesh_maps_to_same_handle() {
  let mut app = create_test_app();

  let rerecast_handle = app
    .world_mut()
    .resource_mut::<Assets<bevy_rerecast::Navmesh>>()
    .reserve_handle();

  let entity_1 =
    app.world_mut().spawn(NavMeshHandle3d(rerecast_handle.clone())).id();
  let entity_2 =
    app.world_mut().spawn(NavMeshHandle3d(rerecast_handle.clone())).id();

  let landmass_handle_1 = app
    .world()
    .entity(entity_1)
    .get::<bevy_landmass::NavMeshHandle3d>()
    .unwrap()
    .0
    .clone();
  let landmass_handle_2 = app
    .world()
    .entity(entity_2)
    .get::<bevy_landmass::NavMeshHandle3d>()
    .unwrap()
    .0
    .clone();
  expect_eq!(landmass_handle_1, landmass_handle_2);

  // Despawn one of the entities to ensure respawning it will give us the same
  // handle.
  app.world_mut().entity_mut(entity_2).despawn();
  let entity_2 = app.world_mut().spawn(NavMeshHandle3d(rerecast_handle)).id();

  let landmass_handle_2 = app
    .world()
    .entity(entity_2)
    .get::<bevy_landmass::NavMeshHandle3d>()
    .unwrap()
    .0
    .clone();
  expect_eq!(landmass_handle_1, landmass_handle_2);
}

#[googletest::test]
fn added_then_removed_mesh_does_not_convert() {
  let mut app = create_test_app();

  let rerecast_mesh = square_rerecast_nav_mesh();
  let rerecast_handle = app
    .world_mut()
    .resource_mut::<Assets<bevy_rerecast::Navmesh>>()
    .add(rerecast_mesh);

  // Asset events are sent in PostUpdate, so we need two frames for landmass
  // systems to see any new assets. In this case, nothing should happen, so just
  // let the asset event go away.
  app.update();
  app.update();

  // Clear out the asset events in the pipe so we are sure we're testing the
  // case of handling an existing asset.
  app
    .world_mut()
    .resource_mut::<Messages<AssetEvent<bevy_rerecast::Navmesh>>>()
    .clear();

  let entity = app.world_mut().spawn(NavMeshHandle3d(rerecast_handle)).id();
  let landmass_handle = app
    .world()
    .entity(entity)
    .get::<bevy_landmass::NavMeshHandle3d>()
    .unwrap()
    .0
    .clone();

  // Despawn the entity, so the handles are dropped.
  app.world_mut().entity_mut(entity).despawn();

  app.update();

  // The landmass mesh wasn't converted.
  expect_false!(
    app
      .world()
      .resource::<Assets<bevy_landmass::NavMesh3d>>()
      .contains(&landmass_handle)
  );
  // We never converted it!
  expect_that!(
    app
      .world_mut()
      .resource_mut::<Messages<AssetEvent<bevy_landmass::NavMesh3d>>>()
      .drain()
      .collect::<Vec<_>>(),
    is_empty()
  );
}

#[googletest::test]
fn converted_mesh_retained_until_original_mesh_dropped() {
  // Since we consume the rerecast nav mesh during conversion, we should also
  // keep the landmass nav mesh alive as long as the rerecast handle is alive.
  // Otherwise, we could consume the rerecast mesh, then drop the landmass mesh
  // and create a new handle - which would never be populated since the rerecast
  // mesh was consumed.

  let mut app = create_test_app();

  let rerecast_handle = app
    .world_mut()
    .resource_mut::<Assets<bevy_rerecast::Navmesh>>()
    .add(square_rerecast_nav_mesh());

  // Spawn the entity but retain our rerecast handle.
  let entity =
    app.world_mut().spawn(NavMeshHandle3d(rerecast_handle.clone())).id();

  // Use IDs instead of handles so we aren't keeping the landmass asset alive.
  let landmass_id = app
    .world()
    .entity(entity)
    .get::<bevy_landmass::NavMeshHandle3d>()
    .unwrap()
    .0
    .id();

  app.update();
  app.update();

  // Despawn, and update the app to make sure everything propagates through.
  // Note, we are still holding the rerecast handle.
  app.world_mut().entity_mut(entity).despawn();
  app.update();

  // Respawn the entity.
  let entity =
    app.world_mut().spawn(NavMeshHandle3d(rerecast_handle.clone())).id();

  // Use IDs instead of handles so we aren't keeping the landmass asset alive.
  let new_landmass_id = app
    .world()
    .entity(entity)
    .get::<bevy_landmass::NavMeshHandle3d>()
    .unwrap()
    .0
    .id();
  expect_eq!(new_landmass_id, landmass_id);
  // Make sure the landmass handle is actually populated still.
  expect_true!(
    app
      .world()
      .resource::<Assets<bevy_landmass::NavMesh3d>>()
      .contains(landmass_id)
  );
}

#[googletest::test]
fn handles_have_same_lifetime_even_without_mesh() {
  // Same as `converted_mesh_retained_until_original_mesh_dropped`, except no
  // real mesh is involved - it's all handles.
  let mut app = create_test_app();

  let rerecast_handle = app
    .world_mut()
    .resource_mut::<Assets<bevy_rerecast::Navmesh>>()
    .reserve_handle();

  // Spawn the entity but retain our rerecast handle.
  let entity =
    app.world_mut().spawn(NavMeshHandle3d(rerecast_handle.clone())).id();

  // Use IDs instead of handles so we aren't keeping the landmass asset alive.
  let landmass_id = app
    .world()
    .entity(entity)
    .get::<bevy_landmass::NavMeshHandle3d>()
    .unwrap()
    .0
    .id();

  app.update();
  app.update();

  // Despawn, and update the app to make sure everything propagates through.
  // Note, we are still holding the rerecast handle.
  app.world_mut().entity_mut(entity).despawn();
  app.update();

  // Respawn the entity.
  let entity =
    app.world_mut().spawn(NavMeshHandle3d(rerecast_handle.clone())).id();

  // Use IDs instead of handles so we aren't keeping the landmass asset alive.
  let new_landmass_id = app
    .world()
    .entity(entity)
    .get::<bevy_landmass::NavMeshHandle3d>()
    .unwrap()
    .0
    .id();
  expect_eq!(new_landmass_id, landmass_id);
}

fn create_test_app() -> App {
  let mut app = App::new();

  app
    .add_plugins((MinimalPlugins, AssetPlugin::default()))
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_micros(
      // Bevy's default fixed timestep
      15625,
    )))
    .add_plugins((RerecastPlugin::default(), LandmassRerecastPlugin::default()))
    .init_asset::<NavMesh3d>();
  app.finish();

  // init time
  app.update();
  app
}

use std::{collections::HashMap, sync::Arc};

use bevy::prelude::*;
use landmass::{NavigationMesh, SamplePointError};

use crate::{
  Agent2dBundle, Agent3dBundle, AgentDesiredVelocity2d, AgentDesiredVelocity3d,
  AgentNodeTypeCostOverrides, AgentSettings, AgentState, AgentTarget2d,
  AgentTarget3d, Archipelago2d, Archipelago3d, ArchipelagoRef2d,
  ArchipelagoRef3d, Character, Character3dBundle, Island, Island2dBundle,
  Island3dBundle, Landmass2dPlugin, Landmass3dPlugin, NavMesh2d, NavMesh3d,
  NavMeshHandle, NavigationMesh3d, Velocity3d,
};

#[test]
fn computes_path_for_agent_and_updates_desired_velocity() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default());

  let archipelago_id = app.world_mut().spawn(Archipelago3d::new()).id();

  let nav_mesh = Arc::new(
    NavigationMesh3d {
      vertices: vec![
        Vec3::new(1.0, 0.0, 1.0),
        Vec3::new(4.0, 0.0, 1.0),
        Vec3::new(4.0, 0.0, 4.0),
        Vec3::new(3.0, 0.0, 4.0),
        Vec3::new(3.0, 0.0, 2.0),
        Vec3::new(1.0, 0.0, 2.0),
      ],
      polygons: vec![vec![5, 4, 1, 0], vec![4, 3, 2, 1]],
      polygon_type_indices: vec![0, 0],
    }
    .validate()
    .expect("is valid"),
  );

  let nav_mesh_handle = app
    .world()
    .resource::<Assets<NavMesh3d>>()
    .get_handle_provider()
    .reserve_handle()
    .typed::<NavMesh3d>();

  app.world_mut().spawn((
    Transform::from_translation(Vec3::new(1.0, 1.0, 1.0)),
    Island3dBundle {
      island: Island,
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
      nav_mesh: NavMeshHandle(nav_mesh_handle.clone()),
    },
  ));

  app.world_mut().resource_mut::<Assets<NavMesh3d>>().insert(
    &nav_mesh_handle,
    NavMesh3d { nav_mesh, type_index_to_node_type: Default::default() },
  );

  let agent_id = app
    .world_mut()
    .spawn((
      Transform::from_translation(Vec3::new(2.5, 1.0, 2.5)),
      Agent3dBundle {
        agent: Default::default(),
        settings: AgentSettings {
          radius: 0.5,
          desired_speed: 1.0,
          max_speed: 2.0,
        },
        archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
      },
      AgentTarget3d::Point(Vec3::new(4.5, 1.0, 4.5)),
    ))
    .id();

  // The first update propagates the global transform, and sets the start of
  // the delta time (in this update, delta time is 0).
  app.update();
  // The second update allows landmass to update properly.
  app.update();

  assert_eq!(
    *app.world().get::<AgentState>(agent_id).expect("current state was added"),
    AgentState::Moving,
  );
  assert_eq!(
    app
      .world()
      .get::<AgentDesiredVelocity3d>(agent_id)
      .expect("desired velocity was added")
      .velocity(),
    Vec3::new(1.5, 0.0, 0.5).normalize(),
  );
}

#[test]
fn adds_and_removes_agents() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default());

  let archipelago_id = app.world_mut().spawn(Archipelago3d::new()).id();

  let agent_id_1 = app
    .world_mut()
    .spawn(Agent3dBundle {
      agent: Default::default(),
      settings: AgentSettings {
        radius: 0.5,
        desired_speed: 1.0,
        max_speed: 2.0,
      },
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
    })
    .id();

  let agent_id_2 = app
    .world_mut()
    .spawn(Agent3dBundle {
      agent: Default::default(),
      settings: AgentSettings {
        radius: 0.5,
        desired_speed: 1.0,
        max_speed: 2.0,
      },
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
    })
    .id();

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  fn sorted(mut v: Vec<Entity>) -> Vec<Entity> {
    v.sort();
    v
  }

  assert_eq!(
    sorted(archipelago.agents.keys().copied().collect()),
    sorted(vec![agent_id_1, agent_id_2]),
  );
  assert_eq!(archipelago.archipelago.get_agent_ids().len(), 2);

  let agent_id_3 = app
    .world_mut()
    .spawn(Agent3dBundle {
      agent: Default::default(),
      settings: AgentSettings {
        radius: 0.5,
        desired_speed: 1.0,
        max_speed: 2.0,
      },
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
    })
    .id();

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  assert_eq!(
    sorted(archipelago.agents.keys().copied().collect()),
    sorted(vec![agent_id_1, agent_id_2, agent_id_3]),
  );
  assert_eq!(archipelago.archipelago.get_agent_ids().len(), 3);

  app.world_mut().despawn(agent_id_2);

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  assert_eq!(
    sorted(archipelago.agents.keys().copied().collect()),
    sorted(vec![agent_id_1, agent_id_3]),
  );
  assert_eq!(archipelago.archipelago.get_agent_ids().len(), 2);

  app.world_mut().despawn(agent_id_1);
  app.world_mut().despawn(agent_id_3);

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  assert_eq!(archipelago.agents.keys().copied().collect::<Vec<_>>(), []);
  assert_eq!(archipelago.archipelago.get_agent_ids().len(), 0);
}

#[test]
fn adds_and_removes_characters() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default());

  let archipelago_id = app.world_mut().spawn(Archipelago3d::new()).id();

  let character_id_1 = app
    .world_mut()
    .spawn((Character3dBundle {
      character: Character { radius: 0.5 },
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
      velocity: Default::default(),
    },))
    .id();

  let character_id_2 = app
    .world_mut()
    .spawn((Character3dBundle {
      character: Character { radius: 0.5 },
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
      velocity: Default::default(),
    },))
    .id();

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  fn sorted(mut v: Vec<Entity>) -> Vec<Entity> {
    v.sort();
    v
  }

  assert_eq!(
    sorted(archipelago.characters.keys().copied().collect()),
    sorted(vec![character_id_1, character_id_2]),
  );
  assert_eq!(archipelago.archipelago.get_character_ids().len(), 2);

  let character_id_3 = app
    .world_mut()
    .spawn((Character3dBundle {
      character: Character { radius: 0.5 },
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
      velocity: Default::default(),
    },))
    .id();

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  assert_eq!(
    sorted(archipelago.characters.keys().copied().collect()),
    sorted(vec![character_id_1, character_id_2, character_id_3]),
  );
  assert_eq!(archipelago.archipelago.get_character_ids().len(), 3);

  app.world_mut().despawn(character_id_2);

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  assert_eq!(
    sorted(archipelago.characters.keys().copied().collect()),
    sorted(vec![character_id_1, character_id_3]),
  );
  assert_eq!(archipelago.archipelago.get_character_ids().len(), 2);

  app.world_mut().despawn(character_id_1);
  app.world_mut().despawn(character_id_3);

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  assert_eq!(archipelago.characters.keys().copied().collect::<Vec<_>>(), []);
  assert_eq!(archipelago.archipelago.get_character_ids().len(), 0);
}

#[test]
fn adds_and_removes_islands() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default());

  let archipelago_id = app.world_mut().spawn(Archipelago3d::new()).id();

  let nav_mesh =
    app.world_mut().resource_mut::<Assets<NavMesh3d>>().add(NavMesh3d {
      nav_mesh: Arc::new(
        NavigationMesh {
          vertices: vec![],
          polygons: vec![],
          polygon_type_indices: vec![],
        }
        .validate()
        .unwrap(),
      ),
      type_index_to_node_type: HashMap::new(),
    });

  let island_id_1 = app
    .world_mut()
    .spawn(Island3dBundle {
      island: Island,
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
      nav_mesh: NavMeshHandle(nav_mesh.clone()),
    })
    .id();

  let island_id_2 = app
    .world_mut()
    .spawn(Island3dBundle {
      island: Island,
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
      nav_mesh: NavMeshHandle(nav_mesh.clone()),
    })
    .id();

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  fn sorted(mut v: Vec<Entity>) -> Vec<Entity> {
    v.sort();
    v
  }

  assert_eq!(
    sorted(archipelago.islands.keys().copied().collect()),
    sorted(vec![island_id_1, island_id_2]),
  );
  assert_eq!(
    sorted(archipelago.reverse_islands.values().copied().collect()),
    sorted(vec![island_id_1, island_id_2]),
  );
  assert_eq!(archipelago.archipelago.get_island_ids().len(), 2);

  let island_id_3 = app
    .world_mut()
    .spawn(Island3dBundle {
      island: Island,
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
      nav_mesh: NavMeshHandle(nav_mesh.clone()),
    })
    .id();

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  assert_eq!(
    sorted(archipelago.islands.keys().copied().collect()),
    sorted(vec![island_id_1, island_id_2, island_id_3])
  );
  assert_eq!(
    sorted(archipelago.reverse_islands.values().copied().collect()),
    sorted(vec![island_id_1, island_id_2, island_id_3])
  );
  assert_eq!(archipelago.archipelago.get_island_ids().len(), 3);

  app.world_mut().despawn(island_id_2);

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  assert_eq!(
    sorted(archipelago.islands.keys().copied().collect()),
    sorted(vec![island_id_1, island_id_3])
  );
  assert_eq!(
    sorted(archipelago.reverse_islands.values().copied().collect()),
    sorted(vec![island_id_1, island_id_3])
  );
  assert_eq!(archipelago.archipelago.get_island_ids().len(), 2);

  app.world_mut().despawn(island_id_1);
  app.world_mut().despawn(island_id_3);

  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  assert_eq!(archipelago.islands.keys().copied().collect::<Vec<_>>(), []);
  assert_eq!(
    archipelago.reverse_islands.values().copied().collect::<Vec<_>>(),
    []
  );
  assert_eq!(archipelago.archipelago.get_island_ids().len(), 0);
}

#[test]
fn changing_agent_fields_changes_landmass_agent() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default());

  let archipelago = app.world_mut().spawn(Archipelago3d::new()).id();

  let agent = app
    .world_mut()
    .spawn((
      Transform::from_translation(Vec3::new(1.0, 2.0, 3.0)),
      Agent3dBundle {
        agent: Default::default(),
        settings: AgentSettings {
          radius: 1.0,
          desired_speed: 1.0,
          max_speed: 2.0,
        },
        archipelago_ref: ArchipelagoRef3d::new(archipelago),
      },
      Velocity3d { velocity: Vec3::new(4.0, 5.0, 6.0) },
      crate::TargetReachedCondition::Distance(Some(1.0)),
    ))
    .id();

  app.update();
  // Update a second time so the global transform propagates correctly.
  app.update();

  let agent_ref = app
    .world()
    .get::<Archipelago3d>(archipelago)
    .unwrap()
    .get_agent(agent)
    .unwrap();
  assert_eq!(agent_ref.position, Vec3::new(1.0, 2.0, 3.0));
  assert_eq!(agent_ref.velocity, Vec3::new(4.0, 5.0, 6.0));
  assert_eq!(agent_ref.radius, 1.0);
  assert_eq!(agent_ref.desired_speed, 1.0);
  assert_eq!(agent_ref.max_speed, 2.0);
  assert_eq!(agent_ref.current_target, None);
  let landmass::TargetReachedCondition::Distance(dist) =
    agent_ref.target_reached_condition
  else {
    panic!("Expected distance reached condition");
  };
  assert_eq!(dist, Some(1.0));

  app.world_mut().entity_mut(agent).insert((
    Transform::from_translation(Vec3::new(7.0, 8.0, 9.0)),
    AgentSettings { radius: 2.0, desired_speed: 1.5, max_speed: 2.0 },
    Velocity3d { velocity: Vec3::new(10.0, 11.0, 12.0) },
    AgentTarget3d::Point(Vec3::new(13.0, 14.0, 15.0)),
    crate::TargetReachedCondition::VisibleAtDistance(Some(2.0)),
  ));

  app.update();
  // Update a second time so the global transform propagates correctly.
  app.update();

  let agent_ref = app
    .world()
    .get::<Archipelago3d>(archipelago)
    .unwrap()
    .get_agent(agent)
    .unwrap();
  assert_eq!(agent_ref.position, Vec3::new(7.0, 8.0, 9.0));
  assert_eq!(agent_ref.velocity, Vec3::new(10.0, 11.0, 12.0));
  assert_eq!(agent_ref.radius, 2.0);
  assert_eq!(agent_ref.desired_speed, 1.5);
  assert_eq!(agent_ref.max_speed, 2.0);
  assert_eq!(agent_ref.current_target, Some(Vec3::new(13.0, 14.0, 15.0)));
  let landmass::TargetReachedCondition::VisibleAtDistance(dist) =
    agent_ref.target_reached_condition
  else {
    panic!("Expected distance reached condition");
  };
  assert_eq!(dist, Some(2.0));
}

#[test]
fn changing_character_fields_changes_landmass_character() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default());

  let archipelago = app.world_mut().spawn(Archipelago3d::new()).id();

  let character = app
    .world_mut()
    .spawn((
      Transform::from_translation(Vec3::new(1.0, 2.0, 3.0)),
      Character3dBundle {
        character: Character { radius: 1.0 },
        archipelago_ref: ArchipelagoRef3d::new(archipelago),
        velocity: Velocity3d { velocity: Vec3::new(4.0, 5.0, 6.0) },
      },
    ))
    .id();

  app.update();
  // Update a second time so the global transform propagates correctly.
  app.update();

  let character_ref = app
    .world()
    .get::<Archipelago3d>(archipelago)
    .unwrap()
    .get_character(character)
    .unwrap();
  assert_eq!(character_ref.position, Vec3::new(1.0, 2.0, 3.0));
  assert_eq!(character_ref.velocity, Vec3::new(4.0, 5.0, 6.0));
  assert_eq!(character_ref.radius, 1.0);

  app.world_mut().entity_mut(character).insert((
    Transform::from_translation(Vec3::new(7.0, 8.0, 9.0)),
    Character { radius: 2.0 },
    Velocity3d { velocity: Vec3::new(10.0, 11.0, 12.0) },
  ));

  app.update();
  // Update a second time so the global transform propagates correctly.
  app.update();

  let agent_ref = app
    .world()
    .get::<Archipelago3d>(archipelago)
    .unwrap()
    .get_character(character)
    .unwrap();
  assert_eq!(agent_ref.position, Vec3::new(7.0, 8.0, 9.0));
  assert_eq!(agent_ref.velocity, Vec3::new(10.0, 11.0, 12.0));
  assert_eq!(agent_ref.radius, 2.0);
}

#[test]
fn node_type_costs_are_used() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass2dPlugin::default());

  let mut archipelago = Archipelago2d::new();
  let slow_node_type = archipelago.add_node_type(10.0).unwrap();

  let archipelago_id = app.world_mut().spawn(archipelago).id();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
        //
        Vec2::new(2.0, 0.0),
        Vec2::new(2.0, 1.0),
        //
        Vec2::new(3.0, 0.0),
        Vec2::new(3.0, 1.0),
        //
        Vec2::new(2.0, 11.0),
        Vec2::new(3.0, 11.0),
        //
        Vec2::new(2.0, 12.0),
        Vec2::new(3.0, 12.0),
        //
        Vec2::new(1.0, 12.0),
        Vec2::new(1.0, 11.0),
        //
        Vec2::new(0.0, 12.0),
        Vec2::new(0.0, 11.0),
      ],
      polygons: vec![
        vec![0, 1, 2, 3],
        vec![2, 1, 4, 5],
        vec![5, 4, 6, 7],
        //
        vec![5, 7, 9, 8],
        vec![8, 9, 11, 10],
        //
        vec![8, 10, 12, 13],
        vec![13, 12, 14, 15],
        //
        vec![3, 2, 13, 15],
      ],
      polygon_type_indices: vec![0, 0, 0, 0, 0, 0, 0, 1],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let nav_mesh_handle = app
    .world()
    .resource::<Assets<NavMesh2d>>()
    .get_handle_provider()
    .reserve_handle()
    .typed::<NavMesh2d>();

  app.world_mut().spawn(Island2dBundle {
    island: Island,
    archipelago_ref: ArchipelagoRef2d::new(archipelago_id),
    nav_mesh: NavMeshHandle(nav_mesh_handle.clone()),
  });

  app.world_mut().resource_mut::<Assets<NavMesh2d>>().insert(
    &nav_mesh_handle,
    NavMesh2d {
      nav_mesh,
      type_index_to_node_type: HashMap::from([(1, slow_node_type)]),
    },
  );

  let agent_id = app
    .world_mut()
    .spawn((
      Transform::from_translation(Vec3::new(0.5, 0.5, 1.0)),
      Agent2dBundle {
        agent: Default::default(),
        settings: AgentSettings {
          radius: 0.5,
          desired_speed: 1.0,
          max_speed: 2.0,
        },
        archipelago_ref: ArchipelagoRef2d::new(archipelago_id),
      },
      AgentTarget2d::Point(Vec2::new(0.5, 11.5)),
    ))
    .id();

  // The first update propagates the global transform, and sets the start of
  // the delta time (in this update, delta time is 0).
  app.update();
  // The second update allows landmass to update properly.
  app.update();

  assert_eq!(
    *app.world().get::<AgentState>(agent_id).expect("current state was added"),
    AgentState::Moving,
  );
  assert_eq!(
    app
      .world()
      .get::<AgentDesiredVelocity2d>(agent_id)
      .expect("desired velocity was added")
      .velocity(),
    Vec2::new(1.5, 0.5).normalize(),
  );
}

#[test]
fn overridden_node_type_costs_are_used() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass2dPlugin::default());

  let mut archipelago = Archipelago2d::new();
  let slow_node_type = archipelago.add_node_type(1.0).unwrap();

  let archipelago_id = app.world_mut().spawn(archipelago).id();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
        //
        Vec2::new(2.0, 0.0),
        Vec2::new(2.0, 1.0),
        //
        Vec2::new(3.0, 0.0),
        Vec2::new(3.0, 1.0),
        //
        Vec2::new(2.0, 11.0),
        Vec2::new(3.0, 11.0),
        //
        Vec2::new(2.0, 12.0),
        Vec2::new(3.0, 12.0),
        //
        Vec2::new(1.0, 12.0),
        Vec2::new(1.0, 11.0),
        //
        Vec2::new(0.0, 12.0),
        Vec2::new(0.0, 11.0),
      ],
      polygons: vec![
        vec![0, 1, 2, 3],
        vec![2, 1, 4, 5],
        vec![5, 4, 6, 7],
        //
        vec![5, 7, 9, 8],
        vec![8, 9, 11, 10],
        //
        vec![8, 10, 12, 13],
        vec![13, 12, 14, 15],
        //
        vec![3, 2, 13, 15],
      ],
      polygon_type_indices: vec![0, 0, 0, 0, 0, 0, 0, 1],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let nav_mesh_handle = app
    .world()
    .resource::<Assets<NavMesh2d>>()
    .get_handle_provider()
    .reserve_handle()
    .typed::<NavMesh2d>();

  app.world_mut().spawn(Island2dBundle {
    island: Island,
    archipelago_ref: ArchipelagoRef2d::new(archipelago_id),
    nav_mesh: NavMeshHandle(nav_mesh_handle.clone()),
  });

  app.world_mut().resource_mut::<Assets<NavMesh2d>>().insert(
    &nav_mesh_handle,
    NavMesh2d {
      nav_mesh,
      type_index_to_node_type: HashMap::from([(1, slow_node_type)]),
    },
  );

  let agent_id = app
    .world_mut()
    .spawn((
      Transform::from_translation(Vec3::new(0.5, 0.5, 1.0)),
      Agent2dBundle {
        agent: Default::default(),
        settings: AgentSettings {
          radius: 0.5,
          desired_speed: 1.0,
          max_speed: 2.0,
        },
        archipelago_ref: ArchipelagoRef2d::new(archipelago_id),
      },
      AgentTarget2d::Point(Vec2::new(0.5, 11.5)),
      {
        let mut overrides = AgentNodeTypeCostOverrides::default();
        overrides.set_node_type_cost(slow_node_type, 10.0);
        overrides
      },
    ))
    .id();

  // The first update propagates the global transform, and sets the start of
  // the delta time (in this update, delta time is 0).
  app.update();
  // The second update allows landmass to update properly.
  app.update();

  assert_eq!(
    *app.world().get::<AgentState>(agent_id).expect("current state was added"),
    AgentState::Moving,
  );
  assert_eq!(
    app
      .world()
      .get::<AgentDesiredVelocity2d>(agent_id)
      .expect("desired velocity was added")
      .velocity(),
    Vec2::new(1.5, 0.5).normalize(),
  );
}

#[test]
fn sample_point_error_on_out_of_range() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass2dPlugin::default());

  let archipelago_entity = app.world_mut().spawn(Archipelago2d::new()).id();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );
  let nav_mesh_handle = app
    .world_mut()
    .resource_mut::<Assets<NavMesh2d>>()
    .add(NavMesh2d { nav_mesh, type_index_to_node_type: HashMap::new() });

  app.world_mut().spawn((Island2dBundle {
    island: Island,
    archipelago_ref: ArchipelagoRef2d::new(archipelago_entity),
    nav_mesh: NavMeshHandle(nav_mesh_handle),
  },));

  // The first update propagates the global transform, and sets the start of
  // the delta time (in this update, delta time is 0).
  app.update();
  // The second update allows landmass to update properly.
  app.update();

  let archipelago =
    app.world().get::<Archipelago2d>(archipelago_entity).unwrap();

  assert_eq!(
    archipelago.sample_point(Vec2::new(-0.5, 0.5), 0.1).map(|p| p.point()),
    Err(SamplePointError::OutOfRange)
  );
}

#[test]
fn samples_point_on_nav_mesh_or_near_nav_mesh() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass2dPlugin::default());

  let archipelago_entity = app.world_mut().spawn(Archipelago2d::new()).id();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );
  let nav_mesh_handle = app
    .world_mut()
    .resource_mut::<Assets<NavMesh2d>>()
    .add(NavMesh2d { nav_mesh, type_index_to_node_type: HashMap::new() });

  let offset = Vec2::new(10.0, 10.0);
  let island_id = app
    .world_mut()
    .spawn((
      Transform::from_translation(offset.extend(0.0)),
      Island2dBundle {
        island: Island,
        archipelago_ref: ArchipelagoRef2d::new(archipelago_entity),
        nav_mesh: NavMeshHandle(nav_mesh_handle),
      },
    ))
    .id();

  // The first update propagates the global transform, and sets the start of
  // the delta time (in this update, delta time is 0).
  app.update();
  // The second update allows landmass to update properly.
  app.update();

  let archipelago =
    app.world().get::<Archipelago2d>(archipelago_entity).unwrap();

  assert_eq!(
    archipelago
      .sample_point(
        /* point= */ offset + Vec2::new(-0.5, 0.5),
        /* distance_to_node= */ 0.6
      )
      .map(|p| (p.island(), p.point())),
    Ok((island_id, offset + Vec2::new(0.0, 0.5)))
  );
  assert_eq!(
    archipelago
      .sample_point(
        /* point= */ offset + Vec2::new(0.5, 0.5),
        /* distance_to_node= */ 0.6
      )
      .map(|p| (p.island(), p.point())),
    Ok((island_id, offset + Vec2::new(0.5, 0.5)))
  );
  assert_eq!(
    archipelago
      .sample_point(
        /* point= */ offset + Vec2::new(1.2, 1.2),
        /* distance_to_node= */ 0.6
      )
      .map(|p| (p.island(), p.point())),
    Ok((island_id, offset + Vec2::new(1.0, 1.0)))
  );
}

#[test]
fn samples_node_types() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass2dPlugin::default());

  let mut archipelago = Archipelago2d::new();
  let node_type = archipelago.add_node_type(2.0).unwrap();
  let archipelago_entity = app.world_mut().spawn(archipelago).id();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
        Vec2::new(2.0, 0.0),
        Vec2::new(2.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3], vec![2, 1, 4, 5]],
      polygon_type_indices: vec![0, 1],
    }
    .validate()
    .expect("nav mesh is valid"),
  );
  let nav_mesh_handle =
    app.world_mut().resource_mut::<Assets<NavMesh2d>>().add(NavMesh2d {
      nav_mesh,
      type_index_to_node_type: HashMap::from([(1, node_type)]),
    });

  let offset = Vec2::new(10.0, 10.0);
  app.world_mut().spawn((
    Transform::from_translation(offset.extend(0.0)),
    Island2dBundle {
      island: Island,
      archipelago_ref: ArchipelagoRef2d::new(archipelago_entity),
      nav_mesh: NavMeshHandle(nav_mesh_handle),
    },
  ));

  // The first update propagates the global transform, and sets the start of
  // the delta time (in this update, delta time is 0).
  app.update();
  // The second update allows landmass to update properly.
  app.update();

  let archipelago =
    app.world().get::<Archipelago2d>(archipelago_entity).unwrap();

  assert_eq!(
    archipelago
      .sample_point(
        /* point= */ offset + Vec2::new(0.5, 0.5),
        /* distance_to_node= */ 0.1
      )
      .map(|p| p.node_type()),
    Ok(None)
  );
  assert_eq!(
    archipelago
      .sample_point(
        /* point= */ offset + Vec2::new(1.5, 0.5),
        /* distance_to_node= */ 0.1
      )
      .map(|p| p.node_type()),
    Ok(Some(node_type))
  );
}

#[test]
fn finds_path() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass2dPlugin::default());

  let archipelago_entity = app.world_mut().spawn(Archipelago2d::new()).id();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let nav_mesh = app
    .world_mut()
    .resource_mut::<Assets<NavMesh2d>>()
    .add(NavMesh2d { nav_mesh, type_index_to_node_type: HashMap::new() });

  app.world_mut().spawn((Island2dBundle {
    island: Island,
    archipelago_ref: ArchipelagoRef2d::new(archipelago_entity),
    nav_mesh: NavMeshHandle(nav_mesh.clone()),
  },));

  app.world_mut().spawn((
    Transform::from_translation(Vec3::new(1.0, 0.0, 0.0)),
    Island2dBundle {
      island: Island,
      archipelago_ref: ArchipelagoRef2d::new(archipelago_entity),
      nav_mesh: NavMeshHandle(nav_mesh.clone()),
    },
  ));

  app.world_mut().spawn((
    Transform::from_translation(Vec3::new(2.0, 0.5, 0.0)),
    Island2dBundle {
      island: Island,
      archipelago_ref: ArchipelagoRef2d::new(archipelago_entity),
      nav_mesh: NavMeshHandle(nav_mesh.clone()),
    },
  ));

  // The first update propagates the global transform, and sets the start of
  // the delta time (in this update, delta time is 0).
  app.update();
  // The second update allows landmass to update properly.
  app.update();

  let archipelago =
    app.world().get::<Archipelago2d>(archipelago_entity).unwrap();
  let start_point = archipelago
    .sample_point(Vec2::new(0.5, 0.5), 1e-5)
    .expect("point is on nav mesh.");
  let end_point = archipelago
    .sample_point(Vec2::new(2.5, 1.25), 1e-5)
    .expect("point is on nav mesh.");
  assert_eq!(
    archipelago.find_path(&start_point, &end_point, &HashMap::new()),
    Ok(vec![Vec2::new(0.5, 0.5), Vec2::new(2.0, 1.0), Vec2::new(2.5, 1.25)])
  );
}

#[test]
fn island_matches_rotation_3d() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default());

  let archipelago_entity = app.world_mut().spawn(Archipelago3d::new()).id();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, -1.0),
        Vec3::new(0.0, 0.0, -1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let nav_mesh = app
    .world_mut()
    .resource_mut::<Assets<NavMesh3d>>()
    .add(NavMesh3d { nav_mesh, type_index_to_node_type: HashMap::new() });

  let island = app
    .world_mut()
    .spawn((
      Transform::from_rotation(Quat::from_rotation_y(2.0)),
      Island3dBundle {
        island: Island,
        archipelago_ref: ArchipelagoRef3d::new(archipelago_entity),
        nav_mesh: NavMeshHandle(nav_mesh.clone()),
      },
    ))
    .id();

  app.update();
  app.update();

  let rotation = app
    .world()
    .get::<Archipelago3d>(archipelago_entity)
    .unwrap()
    .get_island(island)
    .expect("The island is present.")
    .get_transform()
    .rotation;
  assert!((rotation - 2.0).abs() < 1e-6, "left={rotation} right=2.0");
}

#[test]
fn island_matches_rotation_2d() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass2dPlugin::default());

  let archipelago_entity = app.world_mut().spawn(Archipelago2d::new()).id();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(0.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("nav mesh is valid"),
  );

  let nav_mesh = app
    .world_mut()
    .resource_mut::<Assets<NavMesh2d>>()
    .add(NavMesh2d { nav_mesh, type_index_to_node_type: HashMap::new() });

  let island = app
    .world_mut()
    .spawn((
      Transform::from_rotation(Quat::from_rotation_z(2.0)),
      Island2dBundle {
        island: Island,
        archipelago_ref: ArchipelagoRef2d::new(archipelago_entity),
        nav_mesh: NavMeshHandle(nav_mesh.clone()),
      },
    ))
    .id();

  app.update();
  app.update();

  let rotation = app
    .world()
    .get::<Archipelago2d>(archipelago_entity)
    .unwrap()
    .get_island(island)
    .expect("The island is present.")
    .get_transform()
    .rotation;
  assert!((rotation - 2.0).abs() < 1e-6, "left={rotation} right=2.0");
}

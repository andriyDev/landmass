use std::{cmp::Ordering, sync::Arc};

use bevy::{
  app::App,
  asset::{AssetPlugin, Assets},
  math::Vec3,
  prelude::{Transform, TransformPlugin},
  MinimalPlugins,
};
use landmass::AgentOptions;

use crate::{
  coords::ThreeD, Agent, Agent3dBundle, AgentSettings, Archipelago3d,
  ArchipelagoRef3d, Island, Island3dBundle, Landmass3dPlugin, NavMesh3d,
  NavMeshHandle, NavigationMesh3d,
};

use super::{
  draw_archipelago_debug, DebugDrawer, LineType, PointType, TriangleType,
};

struct FakeDrawer {
  points: Vec<(PointType, Vec3)>,
  lines: Vec<(LineType, [Vec3; 2])>,
  triangles: Vec<(TriangleType, [Vec3; 3])>,
}
impl DebugDrawer<ThreeD> for FakeDrawer {
  fn add_point(&mut self, point_type: crate::debug::PointType, point: Vec3) {
    self.points.push((point_type, point));
  }

  fn add_line(&mut self, line_type: crate::debug::LineType, line: [Vec3; 2]) {
    self.lines.push((line_type, line));
  }

  fn add_triangle(
    &mut self,
    triangle_type: crate::debug::TriangleType,
    triangle: [Vec3; 3],
  ) {
    self.triangles.push((triangle_type, triangle));
  }
}

impl FakeDrawer {
  fn new() -> Self {
    Self { points: vec![], lines: vec![], triangles: vec![] }
  }

  fn sort(&mut self) {
    fn lex_order_points(a: Vec3, b: Vec3) -> Ordering {
      a.x
        .partial_cmp(&b.x)
        .unwrap()
        .then(a.y.partial_cmp(&b.y).unwrap())
        .then(a.z.partial_cmp(&b.z).unwrap())
    }
    self.points.sort_by(|a, b| a.0.cmp(&b.0).then(lex_order_points(a.1, b.1)));
    self.lines.sort_by(|a, b| {
      a.0
        .cmp(&b.0)
        .then(lex_order_points(a.1[0], b.1[0]))
        .then(lex_order_points(a.1[1], b.1[1]))
    });
    self.triangles.sort_by(|a, b| {
      a.0
        .cmp(&b.0)
        .then(lex_order_points(a.1[0], b.1[0]))
        .then(lex_order_points(a.1[1], b.1[1]))
        .then(lex_order_points(a.1[2], b.1[2]))
    });
  }
}

#[test]
fn draws_archipelago_debug() {
  let mut app = App::new();

  app
    .add_plugins(MinimalPlugins)
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default());
  // Update early to allow the time to not be 0.0.
  app.update();

  let archipelago_id = app
    .world_mut()
    .spawn(Archipelago3d::new(AgentOptions::default_for_agent_radius(0.5)))
    .id();

  let nav_mesh = Arc::new(
    NavigationMesh3d {
      vertices: vec![
        Vec3::new(1.0, 0.0, 1.0),
        Vec3::new(4.0, 0.0, 1.0),
        Vec3::new(4.0, 0.0, 4.0),
        Vec3::new(1.0, 0.0, 4.0),
      ],
      polygons: vec![vec![3, 2, 1, 0]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("is valid"),
  );

  let nav_mesh_handle = app
    .world_mut()
    .resource_mut::<Assets<NavMesh3d>>()
    .add(NavMesh3d { nav_mesh, type_index_to_node_type: Default::default() });

  app.world_mut().spawn((
    Transform::from_translation(Vec3::new(1.0, 1.0, 1.0)),
    Island3dBundle {
      island: Island,
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
      nav_mesh: NavMeshHandle(nav_mesh_handle.clone()),
    },
  ));

  let agent = app
    .world_mut()
    .spawn((
      Transform::from_translation(Vec3::new(3.0, 1.0, 3.0)),
      Agent3dBundle {
        agent: Agent::default(),
        archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
        settings: AgentSettings {
          radius: 0.5,
          max_speed: 1.0,
          desired_speed: 1.0,
        },
      },
    ))
    .id();

  // Sync the islands with landmass.
  app.update();

  let archipelago = app
    .world()
    .get::<Archipelago3d>(archipelago_id)
    .expect("archipelago exists");

  let mut fake_drawer = FakeDrawer::new();
  draw_archipelago_debug(archipelago, &mut fake_drawer).unwrap();

  fake_drawer.sort();

  assert_eq!(
    fake_drawer.points,
    [(PointType::AgentPosition(agent), Vec3::new(3.0, 1.0, 3.0))]
  );

  assert_eq!(
    fake_drawer.lines,
    [
      (
        LineType::BoundaryEdge,
        [Vec3::new(2.0, 1.0, 2.0), Vec3::new(2.0, 1.0, 5.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(2.0, 1.0, 5.0), Vec3::new(5.0, 1.0, 5.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(5.0, 1.0, 2.0), Vec3::new(2.0, 1.0, 2.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(5.0, 1.0, 5.0), Vec3::new(5.0, 1.0, 2.0)]
      ),
    ]
  );

  assert_eq!(
    fake_drawer.triangles,
    [
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 1.0, 2.0),
          Vec3::new(2.0, 1.0, 5.0),
          Vec3::new(3.5, 1.0, 3.5),
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 1.0, 5.0),
          Vec3::new(5.0, 1.0, 5.0),
          Vec3::new(3.5, 1.0, 3.5),
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(5.0, 1.0, 2.0),
          Vec3::new(2.0, 1.0, 2.0),
          Vec3::new(3.5, 1.0, 3.5),
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(5.0, 1.0, 5.0),
          Vec3::new(5.0, 1.0, 2.0),
          Vec3::new(3.5, 1.0, 3.5),
        ]
      ),
    ]
  );
}

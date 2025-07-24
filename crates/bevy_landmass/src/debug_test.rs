use std::{cmp::Ordering, sync::Arc};

use bevy_app::App;
use bevy_asset::{AssetPlugin, Assets};
use bevy_math::Vec3;
use bevy_transform::{TransformPlugin, components::Transform};

use crate::{
  Agent, Agent3dBundle, AgentOptions, AgentSettings, Archipelago3d,
  ArchipelagoRef3d, FromAgentRadius, Island, Island3dBundle, Landmass3dPlugin,
  NavMesh3d, NavMeshHandle, NavigationMesh3d, coords::ThreeD,
};

use super::{
  DebugDrawer, LineType, PointType, TriangleType, draw_archipelago_debug,
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
    .add_plugins((
      bevy_app::TaskPoolPlugin::default(),
      bevy_time::TimePlugin,
      bevy_app::ScheduleRunnerPlugin::default(),
    ))
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default());
  // Update early to allow the time to not be 0.0.
  app.update();

  let archipelago_id = app
    .world_mut()
    .spawn(Archipelago3d::new(AgentOptions::from_agent_radius(0.5)))
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

#[cfg(feature = "debug-avoidance")]
#[googletest::gtest]
fn draws_avoidance_data_when_requested() {
  use std::collections::HashMap;

  use crate::{
    AgentTarget3d, KeepAvoidanceData, NavigationMesh, Velocity3d,
    debug::{
      AvoidanceDrawer, ConstraintKind, ConstraintLine, draw_avoidance_data,
    },
  };

  use bevy::{math::Vec2, prelude::Entity};
  use googletest::{matcher::MatcherResult, prelude::*};

  let mut app = App::new();

  app
    .add_plugins((
      bevy_app::TaskPoolPlugin::default(),
      bevy_time::TimePlugin,
      bevy_app::ScheduleRunnerPlugin::default(),
    ))
    .add_plugins(TransformPlugin)
    .add_plugins(AssetPlugin::default())
    .add_plugins(Landmass3dPlugin::default());
  // Update early to allow the time to not be 0.0.
  app.update();

  let archipelago_id = app
    .world_mut()
    .spawn(Archipelago3d::new(AgentOptions {
      neighbourhood: 100.0,
      avoidance_time_horizon: 100.0,
      obstacle_avoidance_time_horizon: 100.0,
      ..AgentOptions::from_agent_radius(0.5)
    }))
    .id();

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(11.0, 1.0, 1.0),
        Vec3::new(11.0, 1.0, 11.0),
        Vec3::new(1.0, 1.0, 11.0),
      ],
      polygons: vec![vec![3, 2, 1, 0]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("The mesh is valid."),
  );

  let nav_mesh_handle = app
    .world_mut()
    .resource_mut::<Assets<NavMesh3d>>()
    .add(NavMesh3d { nav_mesh, type_index_to_node_type: Default::default() });

  app.world_mut().spawn((Island3dBundle {
    island: Island,
    archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
    nav_mesh: NavMeshHandle(nav_mesh_handle.clone()),
  },));

  let agent_1 = app
    .world_mut()
    .spawn((
      Transform::from_translation(Vec3::new(6.0, 1.0, 2.0)),
      Agent3dBundle {
        agent: Agent::default(),
        archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
        settings: AgentSettings {
          radius: 0.5,
          max_speed: 1.0,
          desired_speed: 1.0,
        },
      },
      Velocity3d {
        // Use a velocity that allows both agents to agree on their "passing
        // side".
        velocity: Vec3::new(1.0, 0.0, 1.0),
      },
      AgentTarget3d::Point(Vec3::new(6.0, 1.0, 10.0)),
      KeepAvoidanceData,
    ))
    .id();

  app.world_mut().spawn((
    Transform::from_translation(Vec3::new(6.0, 1.0, 10.0)),
    Agent3dBundle {
      agent: Agent::default(),
      archipelago_ref: ArchipelagoRef3d::new(archipelago_id),
      settings: AgentSettings {
        radius: 0.5,
        max_speed: 1.0,
        desired_speed: 1.0,
      },
    },
    AgentTarget3d::Point(Vec3::new(6.0, 1.0, 2.0)),
  ));

  // We now have avoidance data for agent_1.
  app.update();

  struct FakeAvoidanceDrawer(
    HashMap<Entity, HashMap<ConstraintKind, Vec<ConstraintLine>>>,
  );

  impl AvoidanceDrawer for FakeAvoidanceDrawer {
    fn add_constraint(
      &mut self,
      agent: Entity,
      constraint: ConstraintLine,
      kind: ConstraintKind,
    ) {
      self
        .0
        .entry(agent)
        .or_default()
        .entry(kind)
        .or_default()
        .push(constraint);
    }
  }

  let mut drawer = FakeAvoidanceDrawer(Default::default());

  let archipelago =
    app.world().entity(archipelago_id).get::<Archipelago3d>().unwrap();

  draw_avoidance_data(archipelago, &mut drawer);

  #[derive(MatcherBase)]
  struct EquivLineMatcher(ConstraintLine);

  impl Matcher<&ConstraintLine> for EquivLineMatcher {
    fn matches(&self, actual: &ConstraintLine) -> MatcherResult {
      if self.0.normal.angle_to(actual.normal).abs() >= 1e-3 {
        // The lines don't point in the same direction.
        return MatcherResult::NoMatch;
      }
      if (self.0.point - actual.point).dot(actual.normal).abs() >= 1e-3 {
        // The expected line point is not on the actual line.
        return MatcherResult::NoMatch;
      }
      MatcherResult::Match
    }

    fn describe(
      &self,
      matcher_result: MatcherResult,
    ) -> googletest::description::Description {
      match matcher_result {
        MatcherResult::Match => {
          format!("is equivalent to the line {:?}", self.0).into()
        }
        MatcherResult::NoMatch => {
          format!("isn't equivalent to the line {:?}", self.0).into()
        }
      }
    }
  }

  fn equiv_line<'a>(line: ConstraintLine) -> impl Matcher<&'a ConstraintLine> {
    EquivLineMatcher(line)
  }

  // I would ideally use three levels of unordered_elements_are here instead,
  // but due to https://github.com/rust-lang/rust/issues/134719
  expect_eq!(drawer.0.len(), 1);
  let agent_constraints = drawer.0.get(&agent_1).unwrap();
  expect_eq!(agent_constraints.len(), 1);
  let agent_constraints =
    agent_constraints.get(&ConstraintKind::Original).unwrap();

  // Only one of the agents was rendered.
  expect_that!(
    agent_constraints,
    unordered_elements_are!(
      // Lines for the edges of the nav mesh.
      equiv_line(ConstraintLine {
        normal: Vec2::new(-1.0, 0.0),
        point: Vec2::new(0.05, 0.0),
      }),
      equiv_line(ConstraintLine {
        normal: Vec2::new(0.0, 1.0),
        point: Vec2::new(0.0, -0.09),
      }),
      equiv_line(ConstraintLine {
        normal: Vec2::new(1.0, 0.0),
        point: Vec2::new(-0.05, 0.0),
      }),
      equiv_line(ConstraintLine {
        normal: Vec2::new(0.0, -1.0),
        point: Vec2::new(0.0, 0.01),
      }),
      // Line for the other agent.
      equiv_line(ConstraintLine {
        normal: Vec2::new(1.007905, -8.0).normalize().perp(),
        point: Vec2::new(0.0, 0.0),
      }),
    )
  );
}

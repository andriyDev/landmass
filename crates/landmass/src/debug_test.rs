use std::{cmp::Ordering, collections::HashMap, sync::Arc};

use glam::Vec3;

use crate::{
  coords::XYZ,
  debug::{DebugDrawError, DebugDrawer, LineType, PointType, TriangleType},
  Agent, AgentOptions, Archipelago, FromAgentRadius, Island, NavigationMesh,
  Transform,
};

use super::draw_archipelago_debug;

struct FakeDrawer {
  points: Vec<(PointType, Vec3)>,
  lines: Vec<(LineType, [Vec3; 2])>,
  triangles: Vec<(TriangleType, [Vec3; 3])>,
}
impl DebugDrawer<XYZ> for FakeDrawer {
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
fn draws_island_meshes_and_agents() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(2.0, 0.0, 0.0),
      Vec3::new(4.0, 1.0, 0.0),
      Vec3::new(4.0, 2.0, 0.0),
      Vec3::new(2.0, 3.0, 0.0),
      Vec3::new(1.0, 3.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 5.0, 0.0),
      Vec3::new(2.0, 5.0, 0.0),
      Vec3::new(2.0, 4.0, 0.0),
      Vec3::new(3.0, 5.0, 1.0),
      Vec3::new(3.0, 4.0, 1.0),
      Vec3::new(3.0, 4.0, -2.0),
      Vec3::new(3.0, 3.0, -2.0),
    ],
    polygons: vec![
      vec![0, 1, 2, 3, 4, 5, 6, 7],
      vec![5, 4, 10, 9, 8],
      vec![9, 10, 12, 11],
      vec![10, 4, 14, 13],
    ],
    polygon_type_indices: vec![0, 0, 0, 0],
  }
  .validate()
  .expect("Mesh is valid.");

  let mut archipelago =
    Archipelago::<XYZ>::new(AgentOptions::from_agent_radius(0.5));
  const TRANSLATION: Vec3 = Vec3::ONE;
  archipelago.add_island(Island::new(
    Transform { translation: TRANSLATION, rotation: 0.0 },
    Arc::new(nav_mesh),
    HashMap::new(),
  ));

  let agent_id = archipelago.add_agent(Agent::create(
    /* position= */ Vec3::new(3.9, 1.5, 0.0) + TRANSLATION,
    /* velocity= */ Vec3::ZERO,
    /* radius= */ 1.0,
    /* desired_speed= */ 1.0,
    /* max_speed= */ 1.0,
  ));
  archipelago.get_agent_mut(agent_id).unwrap().current_target =
    Some(Vec3::new(1.5, 4.5, 0.0) + TRANSLATION);

  // Update so everything is in sync.
  archipelago.update(1.0);

  let mut fake_drawer = FakeDrawer::new();
  draw_archipelago_debug(&archipelago, &mut fake_drawer)
    .expect("the archipelago can be debug-drawed");

  fake_drawer.sort();

  assert_eq!(
    fake_drawer.points,
    [
      (PointType::AgentPosition(agent_id), Vec3::new(3.9, 1.5, 0.0)),
      (PointType::TargetPosition(agent_id), Vec3::new(1.5, 4.5, 0.0)),
      (PointType::Waypoint(agent_id), Vec3::new(2.0, 3.0, 0.0)),
    ]
    .iter()
    .copied()
    .map(|(t, p)| (t, p + TRANSLATION))
    .collect::<Vec<_>>()
  );
  assert_eq!(
    fake_drawer.lines,
    [
      (
        LineType::BoundaryEdge,
        [Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, 1.0, 0.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(1.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(1.0, 3.0, 0.0), Vec3::new(0.0, 2.0, 0.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(1.0, 5.0, 0.0), Vec3::new(1.0, 3.0, 0.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(2.0, 0.0, 0.0), Vec3::new(4.0, 1.0, 0.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(2.0, 3.0, 0.0), Vec3::new(3.0, 3.0, -2.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(2.0, 4.0, 0.0), Vec3::new(3.0, 4.0, 1.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(2.0, 5.0, 0.0), Vec3::new(1.0, 5.0, 0.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(3.0, 3.0, -2.0), Vec3::new(3.0, 4.0, -2.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(3.0, 4.0, -2.0), Vec3::new(2.0, 4.0, 0.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(3.0, 4.0, 1.0), Vec3::new(3.0, 5.0, 1.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(3.0, 5.0, 1.0), Vec3::new(2.0, 5.0, 0.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(4.0, 1.0, 0.0), Vec3::new(4.0, 2.0, 0.0)]
      ),
      (
        LineType::BoundaryEdge,
        [Vec3::new(4.0, 2.0, 0.0), Vec3::new(2.0, 3.0, 0.0)]
      ),
      //
      (
        LineType::ConnectivityEdge,
        [Vec3::new(2.0, 3.0, 0.0), Vec3::new(1.0, 3.0, 0.0)]
      ),
      (
        LineType::ConnectivityEdge,
        [Vec3::new(2.0, 3.0, 0.0), Vec3::new(2.0, 4.0, 0.0)]
      ),
      (
        LineType::ConnectivityEdge,
        [Vec3::new(2.0, 4.0, 0.0), Vec3::new(2.0, 5.0, 0.0)]
      ),
      //
      (
        LineType::AgentCorridor(agent_id),
        [Vec3::new(1.75, 1.5, 0.0), Vec3::new(1.6, 4.0, 0.0)]
      ),
      //
      (
        LineType::Target(agent_id),
        [Vec3::new(3.9, 1.5, 0.0), Vec3::new(1.5, 4.5, 0.0)]
      ),
      //
      (
        LineType::Waypoint(agent_id),
        [Vec3::new(3.9, 1.5, 0.0), Vec3::new(2.0, 3.0, 0.0)]
      ),
    ]
    .iter()
    .copied()
    .map(|(t, [p0, p1])| (t, [p0 + TRANSLATION, p1 + TRANSLATION]))
    .collect::<Vec<_>>()
  );
  assert_eq!(
    fake_drawer.triangles,
    [
      (
        TriangleType::Node,
        [
          Vec3::new(0.0, 1.0, 0.0),
          Vec3::new(1.0, 0.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(0.0, 2.0, 0.0),
          Vec3::new(0.0, 1.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(1.0, 0.0, 0.0),
          Vec3::new(2.0, 0.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(1.0, 3.0, 0.0),
          Vec3::new(0.0, 2.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(1.0, 3.0, 0.0),
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(1.6, 4.0, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(1.0, 5.0, 0.0),
          Vec3::new(1.0, 3.0, 0.0),
          Vec3::new(1.6, 4.0, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 0.0, 0.0),
          Vec3::new(4.0, 1.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(1.0, 3.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(1.6, 4.0, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(3.0, 3.0, -2.0),
          Vec3::new(2.5, 3.5, -1.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(2.5, 3.5, -1.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(2.0, 5.0, 0.0),
          Vec3::new(1.6, 4.0, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(3.0, 4.0, 1.0),
          Vec3::new(2.5, 4.5, 0.5)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 5.0, 0.0),
          Vec3::new(1.0, 5.0, 0.0),
          Vec3::new(1.6, 4., 0.00)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(2.0, 5.0, 0.0),
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(2.5, 4.5, 0.5)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(3.0, 3.0, -2.0),
          Vec3::new(3.0, 4.0, -2.0),
          Vec3::new(2.5, 3.5, -1.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(3.0, 4.0, -2.0),
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(2.5, 3.5, -1.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(3.0, 4.0, 1.0),
          Vec3::new(3.0, 5.0, 1.0),
          Vec3::new(2.5, 4.5, 0.5)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(3.0, 5.0, 1.0),
          Vec3::new(2.0, 5.0, 0.0),
          Vec3::new(2.5, 4.5, 0.5)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(4.0, 1.0, 0.0),
          Vec3::new(4.0, 2.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      (
        TriangleType::Node,
        [
          Vec3::new(4.0, 2.0, 0.0),
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
    ]
    .iter()
    .copied()
    .map(|(t, [p0, p1, p2])| (
      t,
      [p0 + TRANSLATION, p1 + TRANSLATION, p2 + TRANSLATION]
    ))
    .collect::<Vec<_>>()
  );
}

#[test]
fn draws_boundary_links() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 1.0),
        Vec3::new(2.0, 2.0, 1.0),
        Vec3::new(1.0, 2.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("The mesh is valid."),
  );

  let mut archipelago =
    Archipelago::<XYZ>::new(AgentOptions::from_agent_radius(0.5));
  archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh.clone(),
    HashMap::new(),
  ));
  archipelago.add_island(Island::new(
    Transform { translation: Vec3::new(1.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
    HashMap::new(),
  ));

  // Update so everything is in sync.
  archipelago.update(1.0);

  let mut fake_drawer = FakeDrawer::new();
  draw_archipelago_debug(&archipelago, &mut fake_drawer)
    .expect("the archipelago can be debug-drawed");
  fake_drawer.sort();

  let lines = fake_drawer
    .lines
    .iter()
    .filter(|(line_type, _)| *line_type == LineType::BoundaryLink)
    .map(|(_, edge)| *edge)
    .collect::<Vec<_>>();

  assert_eq!(lines, [[Vec3::new(2.0, 1.0, 1.0), Vec3::new(2.0, 2.0, 1.0)]]);
}

#[test]
fn fails_to_draw_dirty_archipelago() {
  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(2.0, 1.0, 1.0),
        Vec3::new(2.0, 2.0, 1.0),
        Vec3::new(1.0, 2.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("The mesh is valid."),
  );

  let mut fake_drawer = FakeDrawer::new();

  // A brand new archipelago is considered clean.
  let mut archipelago =
    Archipelago::<XYZ>::new(AgentOptions::from_agent_radius(0.5));
  assert_eq!(draw_archipelago_debug(&archipelago, &mut fake_drawer), Ok(()));

  // Creating an island marks the nav data as dirty.
  let island_id = archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh.clone(),
    HashMap::new(),
  ));
  assert_eq!(
    draw_archipelago_debug(&archipelago, &mut fake_drawer),
    Err(DebugDrawError::NavDataDirty)
  );

  archipelago.update(1.0);
  // Nav data is clean again.
  assert_eq!(draw_archipelago_debug(&archipelago, &mut fake_drawer), Ok(()));

  // Setting a nav mesh marks the nav data as dirty.
  archipelago.get_island_mut(island_id).unwrap().set_nav_mesh(nav_mesh.clone());
  assert_eq!(
    draw_archipelago_debug(&archipelago, &mut fake_drawer),
    Err(DebugDrawError::NavDataDirty)
  );
}

#[cfg(feature = "debug-avoidance")]
#[googletest::gtest]
fn draws_avoidance_data_when_requested() {
  use glam::Vec2;
  use googletest::{matcher::MatcherResult, prelude::*};

  use crate::{
    debug::{
      draw_avoidance_data, AvoidanceDrawer, ConstraintKind, ConstraintLine,
    },
    AgentId,
  };

  let nav_mesh = Arc::new(
    NavigationMesh {
      vertices: vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(11.0, 1.0, 1.0),
        Vec3::new(11.0, 11.0, 1.0),
        Vec3::new(1.0, 11.0, 1.0),
      ],
      polygons: vec![vec![0, 1, 2, 3]],
      polygon_type_indices: vec![0],
    }
    .validate()
    .expect("The mesh is valid."),
  );

  let mut archipelago = Archipelago::<XYZ>::new(AgentOptions {
    neighbourhood: 100.0,
    avoidance_time_horizon: 100.0,
    obstacle_avoidance_time_horizon: 100.0,
    ..AgentOptions::from_agent_radius(0.5)
  });
  archipelago.add_island(Island::new(
    Transform::default(),
    nav_mesh.clone(),
    HashMap::new(),
  ));

  let agent_1 = archipelago.add_agent({
    let mut agent = Agent::create(
      Vec3::new(6.0, 2.0, 1.0),
      // Use a velocity that allows both agents to agree on their "passing
      // side".
      Vec3::new(1.0, 1.0, 0.0),
      0.5,
      1.0,
      1.0,
    );
    agent.current_target = Some(Vec3::new(6.0, 10.0, 1.0));
    // This agent we want to see the avoidance data for.
    agent.keep_avoidance_data = true;
    agent
  });

  archipelago.add_agent({
    let mut agent =
      Agent::create(Vec3::new(6.0, 10.0, 1.0), Vec3::ZERO, 0.5, 1.0, 1.0);
    agent.current_target = Some(Vec3::new(6.0, 2.0, 1.0));
    agent
  });

  // We now have avoidance data for agent_1.
  archipelago.update(/* delta_time= */ 1.0);

  struct FakeAvoidanceDrawer(
    HashMap<AgentId, HashMap<ConstraintKind, Vec<ConstraintLine>>>,
  );

  impl AvoidanceDrawer for FakeAvoidanceDrawer {
    fn add_constraint(
      &mut self,
      agent: AgentId,
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

  draw_avoidance_data(&archipelago, &mut drawer);

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

  // Only one of the agents was rendered.
  expect_that!(
    &drawer.0,
    unordered_elements_are!((
      eq(&agent_1),
      unordered_elements_are!((
        eq(&ConstraintKind::Original),
        unordered_elements_are!(
          // Lines for the edges of the nav mesh.
          equiv_line(ConstraintLine {
            normal: Vec2::new(-1.0, 0.0),
            point: Vec2::new(0.05, 0.0),
          }),
          equiv_line(ConstraintLine {
            normal: Vec2::new(0.0, 1.0),
            point: Vec2::new(0.0, -0.01),
          }),
          equiv_line(ConstraintLine {
            normal: Vec2::new(1.0, 0.0),
            point: Vec2::new(-0.05, 0.0),
          }),
          equiv_line(ConstraintLine {
            normal: Vec2::new(0.0, -1.0),
            point: Vec2::new(0.0, 0.09),
          }),
          // Line for the other agent.
          equiv_line(ConstraintLine {
            normal: -Vec2::new(1.007905, 8.0).normalize().perp(),
            point: Vec2::new(0.0, 0.0),
          }),
        ),
      ))
    ))
  );
}

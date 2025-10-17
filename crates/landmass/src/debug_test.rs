use glam::Vec3;
use googletest::{expect_that, matchers::*};

use crate::{
  Agent, AnimationLink, Archipelago, ArchipelagoOptions, FromAgentRadius,
  HeightNavigationMesh, HeightPolygon, NavigationMesh, Transform,
  coords::XYZ,
  debug::{DebugDrawError, DebugDrawer, LineType, PointType, TriangleType},
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
}

#[googletest::test]
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
    height_mesh: None,
  }
  .validate()
  .expect("Mesh is valid.");

  let mut archipelago =
    Archipelago::<XYZ>::new(ArchipelagoOptions::from_agent_radius(0.5));
  const TRANSLATION: Vec3 = Vec3::ONE;
  archipelago.add_island(
    Transform { translation: TRANSLATION, rotation: 0.0 },
    nav_mesh,
  );

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

  let move_point = |v: &mut Vec3| {
    *v = ((*v - TRANSLATION) * 100.0).round() / 100.0;
  };
  fake_drawer.points.iter_mut().for_each(|(_, point)| move_point(point));
  fake_drawer.lines.iter_mut().for_each(|(_, line)| {
    move_point(&mut line[0]);
    move_point(&mut line[1]);
  });
  fake_drawer.triangles.iter_mut().for_each(|(_, tri)| {
    move_point(&mut tri[0]);
    move_point(&mut tri[1]);
    move_point(&mut tri[2]);
  });

  expect_that!(
    fake_drawer.points,
    unordered_elements_are!(
      &(PointType::AgentPosition(agent_id), Vec3::new(3.9, 1.5, 0.0)),
      &(PointType::TargetPosition(agent_id), Vec3::new(1.5, 4.5, 0.0)),
      &(PointType::Waypoint(agent_id), Vec3::new(2.0, 3.0, 0.0)),
    )
  );
  expect_that!(
    fake_drawer.lines,
    unordered_elements_are!(
      &(
        LineType::BoundaryEdge,
        [Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, 1.0, 0.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(1.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(1.0, 3.0, 0.0), Vec3::new(0.0, 2.0, 0.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(1.0, 5.0, 0.0), Vec3::new(1.0, 3.0, 0.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(2.0, 0.0, 0.0), Vec3::new(4.0, 1.0, 0.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(2.0, 3.0, 0.0), Vec3::new(3.0, 3.0, -2.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(2.0, 4.0, 0.0), Vec3::new(3.0, 4.0, 1.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(2.0, 5.0, 0.0), Vec3::new(1.0, 5.0, 0.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(3.0, 3.0, -2.0), Vec3::new(3.0, 4.0, -2.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(3.0, 4.0, -2.0), Vec3::new(2.0, 4.0, 0.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(3.0, 4.0, 1.0), Vec3::new(3.0, 5.0, 1.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(3.0, 5.0, 1.0), Vec3::new(2.0, 5.0, 0.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(4.0, 1.0, 0.0), Vec3::new(4.0, 2.0, 0.0)]
      ),
      &(
        LineType::BoundaryEdge,
        [Vec3::new(4.0, 2.0, 0.0), Vec3::new(2.0, 3.0, 0.0)]
      ),
      //
      &(
        LineType::ConnectivityEdge,
        [Vec3::new(2.0, 3.0, 0.0), Vec3::new(1.0, 3.0, 0.0)]
      ),
      &(
        LineType::ConnectivityEdge,
        [Vec3::new(2.0, 3.0, 0.0), Vec3::new(2.0, 4.0, 0.0)]
      ),
      &(
        LineType::ConnectivityEdge,
        [Vec3::new(2.0, 4.0, 0.0), Vec3::new(2.0, 5.0, 0.0)]
      ),
      //
      &(
        LineType::AgentCorridor(agent_id),
        [Vec3::new(3.9, 1.5, 0.0), Vec3::new(1.5, 3.0, 0.0)]
      ),
      &(
        LineType::AgentCorridor(agent_id),
        [Vec3::new(1.5, 3.0, 0.0), Vec3::new(1.5, 4.5, 0.0)]
      ),
      //
      &(
        LineType::Target(agent_id),
        [Vec3::new(3.9, 1.5, 0.0), Vec3::new(1.5, 4.5, 0.0)]
      ),
      //
      &(
        LineType::Waypoint(agent_id),
        [Vec3::new(3.9, 1.5, 0.0), Vec3::new(2.0, 3.0, 0.0)]
      ),
    )
  );
  expect_that!(
    fake_drawer.triangles,
    unordered_elements_are!(
      &(
        TriangleType::Node,
        [
          Vec3::new(0.0, 1.0, 0.0),
          Vec3::new(1.0, 0.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(0.0, 2.0, 0.0),
          Vec3::new(0.0, 1.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(1.0, 0.0, 0.0),
          Vec3::new(2.0, 0.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(1.0, 3.0, 0.0),
          Vec3::new(0.0, 2.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(1.0, 3.0, 0.0),
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(1.6, 4.0, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(1.0, 5.0, 0.0),
          Vec3::new(1.0, 3.0, 0.0),
          Vec3::new(1.6, 4.0, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(2.0, 0.0, 0.0),
          Vec3::new(4.0, 1.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(1.0, 3.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(1.6, 4.0, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(3.0, 3.0, -2.0),
          Vec3::new(2.5, 3.5, -1.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(2.5, 3.5, -1.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(2.0, 5.0, 0.0),
          Vec3::new(1.6, 4.0, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(3.0, 4.0, 1.0),
          Vec3::new(2.5, 4.5, 0.5)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(2.0, 5.0, 0.0),
          Vec3::new(1.0, 5.0, 0.0),
          Vec3::new(1.6, 4., 0.00)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(2.0, 5.0, 0.0),
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(2.5, 4.5, 0.5)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(3.0, 3.0, -2.0),
          Vec3::new(3.0, 4.0, -2.0),
          Vec3::new(2.5, 3.5, -1.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(3.0, 4.0, -2.0),
          Vec3::new(2.0, 4.0, 0.0),
          Vec3::new(2.5, 3.5, -1.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(3.0, 4.0, 1.0),
          Vec3::new(3.0, 5.0, 1.0),
          Vec3::new(2.5, 4.5, 0.5)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(3.0, 5.0, 1.0),
          Vec3::new(2.0, 5.0, 0.0),
          Vec3::new(2.5, 4.5, 0.5)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(4.0, 1.0, 0.0),
          Vec3::new(4.0, 2.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
      &(
        TriangleType::Node,
        [
          Vec3::new(4.0, 2.0, 0.0),
          Vec3::new(2.0, 3.0, 0.0),
          Vec3::new(1.75, 1.5, 0.0)
        ]
      ),
    )
  );
}

#[googletest::test]
fn draws_boundary_links() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 1.0, 1.0),
      Vec3::new(2.0, 1.0, 1.0),
      Vec3::new(2.0, 2.0, 1.0),
      Vec3::new(1.0, 2.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0],
    height_mesh: None,
  }
  .validate()
  .expect("The mesh is valid.");

  let mut archipelago =
    Archipelago::<XYZ>::new(ArchipelagoOptions::from_agent_radius(0.5));
  archipelago.add_island(Transform::default(), nav_mesh.clone());
  archipelago.add_island(
    Transform { translation: Vec3::new(1.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  );

  let agent_id = archipelago.add_agent({
    let mut agent = Agent::create(
      /* position= */ Vec3::new(1.5, 1.25, 1.0),
      /* velocity= */ Vec3::ZERO,
      /* radius= */ 0.5,
      /* desired_speed= */ 1.0,
      /* max_speed= */ 2.0,
    );
    agent.current_target = Some(Vec3::new(2.5, 1.25, 1.0));
    agent
  });

  // Update so everything is in sync.
  archipelago.update(1.0);

  let mut fake_drawer = FakeDrawer::new();
  draw_archipelago_debug(&archipelago, &mut fake_drawer)
    .expect("the archipelago can be debug-drawed");

  expect_that!(
    fake_drawer.lines,
    contains_each!(
      eq(&(
        LineType::BoundaryLink,
        [Vec3::new(2.0, 2.0, 1.0), Vec3::new(2.0, 1.0, 1.0)]
      )),
      // The agent's waypoint goes directly to the end.
      eq(&(
        LineType::Waypoint(agent_id),
        [Vec3::new(1.5, 1.25, 1.0), Vec3::new(2.5, 1.25, 1.0)]
      )),
      eq(&(
        LineType::Target(agent_id),
        [Vec3::new(1.5, 1.25, 1.0), Vec3::new(2.5, 1.25, 1.0)]
      )),
      // The agent corridor includes the point on the boundary link.
      eq(&(
        LineType::AgentCorridor(agent_id),
        [Vec3::new(1.5, 1.25, 1.0), Vec3::new(2.0, 1.5, 1.0)]
      )),
      eq(&(
        LineType::AgentCorridor(agent_id),
        [Vec3::new(2.0, 1.5, 1.0), Vec3::new(2.5, 1.25, 1.0)]
      )),
    )
  );
}

#[googletest::test]
fn draws_animation_links() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0],
    height_mesh: None,
  }
  .validate()
  .expect("The mesh is valid.");

  let mut archipelago =
    Archipelago::<XYZ>::new(ArchipelagoOptions::from_agent_radius(0.5));
  archipelago.add_island(Transform::default(), nav_mesh.clone());
  archipelago.add_island(
    Transform { translation: Vec3::new(2.0, 0.0, 0.0), rotation: 0.0 },
    nav_mesh.clone(),
  );
  let link_id = archipelago.add_animation_link(AnimationLink {
    start_edge: (Vec3::new(0.9, 0.0, 0.0), Vec3::new(0.9, 1.0, 0.0)),
    end_edge: (Vec3::new(2.1, 0.0, 0.0), Vec3::new(2.1, 1.0, 0.0)),
    kind: 0,
    cost: 1.0,
    bidirectional: false,
  });
  let agent_id = archipelago.add_agent({
    let mut agent =
      Agent::create(Vec3::new(0.5, 0.25, 0.0), Vec3::ZERO, 0.5, 1.0, 2.0);
    agent.current_target = Some(Vec3::new(2.5, 0.25, 0.0));
    agent
  });

  // Update so everything is in sync.
  archipelago.update(1.0);

  let mut fake_drawer = FakeDrawer::new();
  draw_archipelago_debug(&archipelago, &mut fake_drawer)
    .expect("the archipelago can be debug-drawed");

  expect_that!(
    fake_drawer.lines,
    contains_each!(
      eq(&(
        LineType::AnimationLinkStart(link_id),
        [Vec3::new(0.9, 0.0, 0.0), Vec3::new(0.9, 1.0, 0.0)]
      )),
      eq(&(
        LineType::AnimationLinkEnd(link_id),
        [Vec3::new(2.1, 0.0, 0.0), Vec3::new(2.1, 1.0, 0.0)]
      )),
      eq(&(
        LineType::AnimationLinkConnection(link_id),
        [Vec3::new(0.9, 0.5, 0.0), Vec3::new(2.1, 0.5, 0.0)]
      )),
      // The corridor includes the animation link.
      eq(&(
        LineType::CorridorAnimationLink(agent_id, link_id),
        [Vec3::new(0.9, 0.5, 0.0), Vec3::new(2.1, 0.5, 0.0)]
      )),
      // The waypoint goes to the animation link start point.
      eq(&(
        LineType::Waypoint(agent_id),
        [Vec3::new(0.5, 0.25, 0.0), Vec3::new(0.9, 0.25, 0.0)],
      )),
      // Include the estimated path to travel.
      eq(&(
        LineType::PathAnimationLink(agent_id, link_id),
        [Vec3::new(0.9, 0.25, 0.0), Vec3::new(2.1, 0.25, 0.0)],
      )),
    )
  );
}

#[googletest::test]
fn draws_height_mesh() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(0.0, 0.0, 0.0),
      Vec3::new(1.0, 0.0, 0.0),
      Vec3::new(1.0, 1.0, 0.0),
      Vec3::new(0.0, 1.0, 0.0),
      Vec3::new(1.0, 2.0, 0.0),
      Vec3::new(0.0, 2.0, 0.0),
    ],
    polygons: vec![vec![0, 1, 2, 3], vec![3, 2, 4, 5]],
    polygon_type_indices: vec![0; 2],
    height_mesh: Some(HeightNavigationMesh {
      vertices: vec![
        Vec3::new(0.0, 0.0, -2.0),
        Vec3::new(1.0, 0.0, -2.0),
        Vec3::new(1.0, 1.0, -2.0),
        Vec3::new(0.0, 1.0, -2.0),
        Vec3::new(0.0, 0.5, -2.0),
        Vec3::new(0.0, 1.0, 3.0),
        Vec3::new(1.0, 1.0, 3.0),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(0.0, 2.0, 3.0),
      ],
      triangles: vec![[0, 1, 2], [2, 3, 4], [2, 4, 0], [0, 1, 2], [2, 3, 0]],
      polygons: vec![
        HeightPolygon {
          base_vertex_index: 0,
          vertex_count: 5,
          base_triangle_index: 0,
          triangle_count: 3,
        },
        HeightPolygon {
          base_vertex_index: 5,
          vertex_count: 4,
          base_triangle_index: 3,
          triangle_count: 2,
        },
      ],
    }),
  }
  .validate()
  .expect("The mesh is valid.");

  let mut archipelago =
    Archipelago::<XYZ>::new(ArchipelagoOptions::from_agent_radius(0.5));
  archipelago.add_island(Transform::default(), nav_mesh.clone());

  // Update so everything is in sync.
  archipelago.update(1.0);

  let mut fake_drawer = FakeDrawer::new();
  draw_archipelago_debug(&archipelago, &mut fake_drawer)
    .expect("the archipelago can be debug-drawed");

  expect_that!(
    fake_drawer.lines,
    contains_each!(
      // The nav mesh should still be rendered as normal.
      eq(&(
        LineType::BoundaryEdge,
        [Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)]
      )),
      eq(&(
        LineType::BoundaryEdge,
        [Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0)]
      )),
      eq(&(
        LineType::BoundaryEdge,
        [Vec3::new(1.0, 1.0, 0.0), Vec3::new(1.0, 2.0, 0.0)]
      )),
      eq(&(
        LineType::BoundaryEdge,
        [Vec3::new(1.0, 2.0, 0.0), Vec3::new(0.0, 2.0, 0.0)]
      )),
      eq(&(
        LineType::BoundaryEdge,
        [Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, 1.0, 0.0)]
      )),
      eq(&(
        LineType::BoundaryEdge,
        [Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 0.0)]
      )),
      eq(&(
        LineType::ConnectivityEdge,
        [Vec3::new(1.0, 1.0, 0.0), Vec3::new(0.0, 1.0, 0.0)]
      )),
      // The height mesh edges are also drawn.

      // The edges for the first node.
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(0.0, 0.0, -2.0), Vec3::new(1.0, 0.0, -2.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(1.0, 0.0, -2.0), Vec3::new(1.0, 1.0, -2.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(1.0, 1.0, -2.0), Vec3::new(0.0, 0.0, -2.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(1.0, 1.0, -2.0), Vec3::new(0.0, 1.0, -2.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(0.0, 1.0, -2.0), Vec3::new(0.0, 0.5, -2.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(0.0, 0.5, -2.0), Vec3::new(1.0, 1.0, -2.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(1.0, 1.0, -2.0), Vec3::new(0.0, 0.5, -2.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(0.0, 0.5, -2.0), Vec3::new(0.0, 0.0, -2.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(0.0, 0.0, -2.0), Vec3::new(1.0, 1.0, -2.0)]
      )),
      // The edges for the second node.
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(0.0, 1.0, 3.0), Vec3::new(1.0, 1.0, 3.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(1.0, 1.0, 3.0), Vec3::new(1.0, 2.0, 3.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(1.0, 2.0, 3.0), Vec3::new(0.0, 1.0, 3.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(1.0, 2.0, 3.0), Vec3::new(0.0, 2.0, 3.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(0.0, 2.0, 3.0), Vec3::new(0.0, 1.0, 3.0)]
      )),
      eq(&(
        LineType::HeightEdge,
        [Vec3::new(0.0, 1.0, 3.0), Vec3::new(1.0, 2.0, 3.0)]
      )),
    )
  );
}

#[test]
fn fails_to_draw_dirty_archipelago() {
  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 1.0, 1.0),
      Vec3::new(2.0, 1.0, 1.0),
      Vec3::new(2.0, 2.0, 1.0),
      Vec3::new(1.0, 2.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0],
    height_mesh: None,
  }
  .validate()
  .expect("The mesh is valid.");

  let mut fake_drawer = FakeDrawer::new();

  // A brand new archipelago is considered clean.
  let mut archipelago =
    Archipelago::<XYZ>::new(ArchipelagoOptions::from_agent_radius(0.5));
  assert_eq!(draw_archipelago_debug(&archipelago, &mut fake_drawer), Ok(()));

  // Creating an island marks the nav data as dirty.
  let island_id =
    archipelago.add_island(Transform::default(), nav_mesh.clone());
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
  use std::collections::HashMap;

  use crate::{
    AgentId,
    debug::{
      AvoidanceDrawer, ConstraintKind, ConstraintLine, draw_avoidance_data,
    },
  };

  let nav_mesh = NavigationMesh {
    vertices: vec![
      Vec3::new(1.0, 1.0, 1.0),
      Vec3::new(11.0, 1.0, 1.0),
      Vec3::new(11.0, 11.0, 1.0),
      Vec3::new(1.0, 11.0, 1.0),
    ],
    polygons: vec![vec![0, 1, 2, 3]],
    polygon_type_indices: vec![0],
    height_mesh: None,
  }
  .validate()
  .expect("The mesh is valid.");

  let mut archipelago = Archipelago::<XYZ>::new(ArchipelagoOptions {
    neighbourhood: 100.0,
    avoidance_time_horizon: 100.0,
    obstacle_avoidance_time_horizon: 100.0,
    ..ArchipelagoOptions::from_agent_radius(0.5)
  });
  archipelago.add_island(Transform::default(), nav_mesh.clone());

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

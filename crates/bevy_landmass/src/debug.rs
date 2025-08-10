use std::{marker::PhantomData, ops::Deref};

use bevy_app::{Plugin, Update};
use bevy_color::Color;
use bevy_ecs::{
  entity::Entity,
  resource::Resource,
  schedule::IntoScheduleConfigs,
  system::{Query, Res},
};
use bevy_gizmos::{
  AppGizmoBuilder,
  config::{GizmoConfig, GizmoConfigGroup},
  gizmos::Gizmos,
};
use bevy_math::{Isometry3d, Quat};
use bevy_reflect::Reflect;
use bevy_time::Time;
use bevy_transform::components::Transform;

use crate::{
  Archipelago, LandmassSystemSet,
  coords::{CoordinateSystem, ThreeD, TwoD},
};

#[cfg(feature = "debug-avoidance")]
use bevy_math::Vec2;

pub use landmass::debug::DebugDrawError;

#[cfg(feature = "debug-avoidance")]
pub use landmass::debug::ConstraintKind;

/// The type of debug points.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum PointType {
  /// The position of an agent.
  AgentPosition(Entity),
  /// The target of an agent.
  TargetPosition(Entity),
  /// The waypoint of an agent.
  Waypoint(Entity),
}

/// The type of debug lines.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum LineType {
  /// An edge of a node that is the boundary of a nav mesh.
  BoundaryEdge,
  /// An edge of a node that is connected to another node.
  ConnectivityEdge,
  /// An edge of a triangle in the detail mesh.
  HeightEdge,
  /// A link between two islands along their boundary edge.
  BoundaryLink,
  /// Part of an agent's current path. The corridor follows the path along
  /// nodes, not the actual path the agent will travel.
  AgentCorridor(Entity),
  /// Line from an agent to its target.
  Target(Entity),
  /// Line to the waypoint of an agent.
  Waypoint(Entity),
}

/// The type of debug triangles.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum TriangleType {
  /// Part of a node/polygon in a nav mesh.
  Node,
}

/// Trait to "draw" Archipelago state to. Users should implement this to
/// visualize the state of their Archipelago.
pub trait DebugDrawer<CS: CoordinateSystem> {
  fn add_point(&mut self, point_type: PointType, point: CS::Coordinate);
  fn add_line(&mut self, line_type: LineType, line: [CS::Coordinate; 2]);
  fn add_triangle(
    &mut self,
    triangle_type: TriangleType,
    triangle: [CS::Coordinate; 3],
  );
}

/// Draws all parts of `archipelago` to `debug_drawer`. This is a lower level
/// API to allow custom debug drawing. For a pre-made implementation, use
/// [`LandmassDebugPlugin`].
pub fn draw_archipelago_debug<CS: CoordinateSystem>(
  archipelago: &crate::Archipelago<CS>,
  debug_drawer: &mut impl DebugDrawer<CS>,
) -> Result<(), DebugDrawError> {
  struct DebugDrawerAdapter<'a, CS: CoordinateSystem, D: DebugDrawer<CS>> {
    archipelago: &'a crate::Archipelago<CS>,
    drawer: &'a mut D,
  }

  impl<CS: CoordinateSystem, D: DebugDrawer<CS>>
    landmass::debug::DebugDrawer<CS> for DebugDrawerAdapter<'_, CS, D>
  {
    fn add_point(
      &mut self,
      point_type: landmass::debug::PointType,
      point: CS::Coordinate,
    ) {
      let point_type = match point_type {
        landmass::debug::PointType::AgentPosition(agent_id) => {
          PointType::AgentPosition(
            *self.archipelago.reverse_agents.get(&agent_id).unwrap(),
          )
        }
        landmass::debug::PointType::TargetPosition(agent_id) => {
          PointType::TargetPosition(
            *self.archipelago.reverse_agents.get(&agent_id).unwrap(),
          )
        }
        landmass::debug::PointType::Waypoint(agent_id) => PointType::Waypoint(
          *self.archipelago.reverse_agents.get(&agent_id).unwrap(),
        ),
      };
      self.drawer.add_point(point_type, point);
    }

    fn add_line(
      &mut self,
      line_type: landmass::debug::LineType,
      line: [CS::Coordinate; 2],
    ) {
      let line_type = match line_type {
        landmass::debug::LineType::BoundaryEdge => LineType::BoundaryEdge,
        landmass::debug::LineType::ConnectivityEdge => {
          LineType::ConnectivityEdge
        }
        landmass::debug::LineType::HeightEdge => LineType::HeightEdge,
        landmass::debug::LineType::BoundaryLink => LineType::BoundaryLink,
        landmass::debug::LineType::AgentCorridor(agent_id) => {
          LineType::AgentCorridor(
            *self.archipelago.reverse_agents.get(&agent_id).unwrap(),
          )
        }
        landmass::debug::LineType::Target(agent_id) => LineType::Target(
          *self.archipelago.reverse_agents.get(&agent_id).unwrap(),
        ),
        landmass::debug::LineType::Waypoint(agent_id) => LineType::Waypoint(
          *self.archipelago.reverse_agents.get(&agent_id).unwrap(),
        ),
      };
      self.drawer.add_line(line_type, line);
    }

    fn add_triangle(
      &mut self,
      triangle_type: landmass::debug::TriangleType,
      triangle: [CS::Coordinate; 3],
    ) {
      let triangle_type = match triangle_type {
        landmass::debug::TriangleType::Node => TriangleType::Node,
      };
      self.drawer.add_triangle(triangle_type, triangle);
    }
  }

  landmass::debug::draw_archipelago_debug(
    &archipelago.archipelago,
    &mut DebugDrawerAdapter { archipelago, drawer: debug_drawer },
  )
}

#[cfg(feature = "debug-avoidance")]
/// A constraint in velocity-space for an agent's velocity for local collision
/// avoidance. The constraint restricts the velocity to lie on one side of a
/// line (aka., only a half-plane is considered valid). This is equivalent to
/// [`landmass::debug::ConstraintLine`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConstraintLine {
  /// A point on the line separating the valid and invalid velocities.
  pub point: Vec2,
  /// The normal of the line separating the valid and invalid velocities. The
  /// normal always points towards the valid velocities.
  pub normal: Vec2,
}

#[cfg(feature = "debug-avoidance")]
/// A trait for reporting agent local collision avoidance constraints.
pub trait AvoidanceDrawer {
  /// Reports a single avoidance constraint.
  fn add_constraint(
    &mut self,
    agent: Entity,
    constraint: ConstraintLine,
    kind: ConstraintKind,
  );
}

#[cfg(feature = "debug-avoidance")]
impl ConstraintLine {
  fn from_landmass(line: &landmass::debug::ConstraintLine) -> Self {
    Self {
      point: Vec2::new(line.point.x, line.point.y),
      normal: Vec2::new(line.normal.x, line.normal.y),
    }
  }
}

/// Draws the avoidance data for any agent marked with TODO
#[cfg(feature = "debug-avoidance")]
pub fn draw_avoidance_data<CS: CoordinateSystem>(
  archipelago: &crate::Archipelago<CS>,
  avoidance_drawer: &mut impl AvoidanceDrawer,
) {
  struct AvoidanceDrawerAdapter<'a, CS: CoordinateSystem, D: AvoidanceDrawer> {
    archipelago: &'a crate::Archipelago<CS>,
    drawer: &'a mut D,
  }

  impl<CS: CoordinateSystem, D: AvoidanceDrawer>
    landmass::debug::AvoidanceDrawer for AvoidanceDrawerAdapter<'_, CS, D>
  {
    fn add_constraint(
      &mut self,
      agent: landmass::AgentId,
      constraint: landmass::debug::ConstraintLine,
      kind: landmass::debug::ConstraintKind,
    ) {
      self.drawer.add_constraint(
        *self.archipelago.reverse_agents.get(&agent).unwrap(),
        ConstraintLine::from_landmass(&constraint),
        kind,
      );
    }
  }

  landmass::debug::draw_avoidance_data(
    &archipelago.archipelago,
    &mut AvoidanceDrawerAdapter { archipelago, drawer: avoidance_drawer },
  );
}

/// A plugin to draw landmass debug data with Bevy gizmos.
pub struct LandmassDebugPlugin<CS: CoordinateSystem> {
  /// Whether to begin drawing on startup.
  pub draw_on_start: bool,
  // Marker for the coordinate systems.
  pub marker: PhantomData<CS>,
}

pub type Landmass2dDebugPlugin = LandmassDebugPlugin<TwoD>;
pub type Landmass3dDebugPlugin = LandmassDebugPlugin<ThreeD>;

impl<CS: CoordinateSystem> Default for LandmassDebugPlugin<CS> {
  fn default() -> Self {
    Self { draw_on_start: true, marker: Default::default() }
  }
}

impl<CS: CoordinateSystem> Plugin for LandmassDebugPlugin<CS> {
  fn build(&self, app: &mut bevy_app::App) {
    app
      .insert_resource(EnableLandmassDebug(self.draw_on_start))
      .add_systems(
        Update,
        draw_archipelagos_default::<CS>
          .in_set(LandmassSystemSet::Output)
          .run_if(|enable: Res<EnableLandmassDebug>| enable.0),
      )
      .insert_gizmo_config(
        LandmassGizmos::default(),
        GizmoConfig { depth_bias: -1.0, ..Default::default() },
      );
  }
}

/// A resource controlling whether debug data is drawn.
#[derive(Resource, Default)]
pub struct EnableLandmassDebug(pub bool);

impl std::ops::Deref for EnableLandmassDebug {
  type Target = bool;
  fn deref(&self) -> &Self::Target {
    &self.0
  }
}

impl std::ops::DerefMut for EnableLandmassDebug {
  fn deref_mut(&mut self) -> &mut Self::Target {
    &mut self.0
  }
}

/// A config group for landmass debug gizmos.
#[derive(Reflect, GizmoConfigGroup, Debug, Clone, Copy, PartialEq)]
pub struct LandmassGizmos {
  // points
  /// The color to use when drawing an agent's current position.
  ///
  /// If [`None`], agent positions are not drawn.
  pub agent_position: Option<Color>,
  /// The color to use when drawing an agent's target position.
  ///
  /// If [`None`], target positions are not drawn.
  pub target_position: Option<Color>,
  /// The color to use when drawing waypoints along a path.
  ///
  /// If [`None`], waypoints are not drawn.
  pub waypoint_position: Option<Color>,

  // lines
  /// The color to use when drawing boundary edges on a navmesh.
  ///
  /// If [`None`], waypoints are not drawn.
  pub boundary_edge: Option<Color>,
  /// The color to use when drawing connectivity edges on a navmesh.
  ///
  /// If [`None`], waypoints are not drawn.
  pub connectivity_edge: Option<Color>,
  /// The color to use when drawing height edges on a navmesh.
  ///
  /// If [`None`], waypoints are not drawn.
  pub height_edge: Option<Color>,
  /// The color to use when drawing boundary links on a navmesh.
  ///
  /// If [`None`], waypoints are not drawn.
  pub boundary_link: Option<Color>,
  /// The color to use when drawing agent corridors.
  ///
  /// If [`None`], waypoints are not drawn.
  pub agent_corridor: Option<Color>,
  /// The color to use when drawing lines between targets.
  ///
  /// If [`None`], waypoints are not drawn.
  pub target_line: Option<Color>,
  /// The color to use when drawing lines between waypoints.
  ///
  /// If [`None`], waypoints are not drawn.
  pub waypoint_line: Option<Color>,
}

impl Default for LandmassGizmos {
  fn default() -> Self {
    Self {
      agent_position: Color::srgba(0.0, 1.0, 0.0, 0.6).into(),
      target_position: Color::srgba(1.0, 1.0, 0.0, 0.6).into(),
      waypoint_position: Color::srgba(0.6, 0.6, 0.6, 0.6).into(),
      boundary_edge: Color::srgba(0.0, 0.0, 1.0, 0.6).into(),
      connectivity_edge: Color::srgba_u8(33, 102, 57, 128).into(),
      height_edge: Color::srgba_u8(33, 102, 57, 128).into(),
      boundary_link: Color::srgba(0.0, 1.0, 0.0, 0.6).into(),
      agent_corridor: Color::srgba(0.6, 0.0, 0.6, 0.6).into(),
      target_line: Color::srgba(1.0, 1.0, 0.0, 0.6).into(),
      waypoint_line: Color::srgba(0.6, 0.6, 0.6, 0.6).into(),
    }
  }
}

/// A gizmo debug drawer.
struct GizmoDrawer<'w, 's, 'a, CS: CoordinateSystem>(
  &'a mut Gizmos<'w, 's, LandmassGizmos>,
  PhantomData<CS>,
);

impl<'w, 's, 'a, CS: CoordinateSystem> Deref for GizmoDrawer<'w, 's, 'a, CS> {
  type Target = Gizmos<'w, 's, LandmassGizmos>;

  fn deref(&self) -> &Self::Target {
    self.0
  }
}

impl<CS: CoordinateSystem> DebugDrawer<CS> for GizmoDrawer<'_, '_, '_, CS> {
  fn add_point(&mut self, point_type: PointType, point: CS::Coordinate) {
    let Some(color) = (match point_type {
      PointType::AgentPosition(..) => self.config_ext.agent_position,
      PointType::TargetPosition(..) => self.config_ext.target_position,
      PointType::Waypoint(..) => self.config_ext.waypoint_position,
    }) else {
      return;
    };
    self.0.sphere(
      Isometry3d::new(CS::to_world_position(&point), Quat::IDENTITY),
      0.2,
      color,
    );
  }

  fn add_line(&mut self, line_type: LineType, line: [CS::Coordinate; 2]) {
    if line_type == LineType::BoundaryLink {
      let Some(color) = self.config_ext.boundary_link else {
        return;
      };
      let line =
        [CS::to_world_position(&line[0]), CS::to_world_position(&line[1])];
      self.0.cuboid(
        Transform::default()
          .looking_to(line[1] - line[0], bevy_math::Vec3::new(0.0, 1.0, 0.0))
          .with_translation((line[0] + line[1]) * 0.5)
          .with_scale(bevy_math::Vec3::new(
            0.01,
            0.01,
            line[0].distance(line[1]),
          )),
        color,
      );
      return;
    }
    let Some(color) = (match line_type {
      LineType::BoundaryEdge => self.config_ext.boundary_edge,
      LineType::ConnectivityEdge => self.config_ext.connectivity_edge,
      LineType::HeightEdge => self.config_ext.height_edge,
      LineType::BoundaryLink => self.config_ext.boundary_link,
      LineType::AgentCorridor(..) => self.config_ext.agent_corridor,
      LineType::Target(..) => self.config_ext.target_line,
      LineType::Waypoint(..) => self.config_ext.waypoint_line,
    }) else {
      return;
    };
    self.0.line(
      CS::to_world_position(&line[0]),
      CS::to_world_position(&line[1]),
      color,
    );
  }

  fn add_triangle(
    &mut self,
    _triangle_type: TriangleType,
    _triangle: [CS::Coordinate; 3],
  ) {
    // Bevy doesn't have a way to draw triangles :'(
  }
}

/// A system for drawing debug data.
fn draw_archipelagos_default<CS: CoordinateSystem>(
  time: Res<Time>,
  archipelagos: Query<&Archipelago<CS>>,
  mut gizmos: Gizmos<'_, '_, LandmassGizmos>,
) {
  if time.delta_secs() == 0.0 {
    return;
  }
  let mut drawer = GizmoDrawer(&mut gizmos, PhantomData::<CS>);
  for archipelago in archipelagos.iter() {
    draw_archipelago_debug(archipelago, &mut drawer)
      .expect("the archipelago can be debug-drawn");
  }
}

#[cfg(test)]
#[path = "debug_test.rs"]
mod test;

use std::marker::PhantomData;

use crate::{
  coords::{CoordinateSystem, ThreeD, TwoD},
  Archipelago, LandmassSystemSet,
};
use bevy::{
  app::Update,
  color::Color,
  gizmos::AppGizmoBuilder,
  math::{Isometry3d, Quat},
  prelude::{
    Deref, DerefMut, Entity, GizmoConfig, GizmoConfigGroup, Gizmos,
    IntoSystemConfigs, Plugin, Query, Res, Resource,
  },
  reflect::Reflect,
  time::Time,
  transform::components::Transform,
};

pub use landmass::debug::DebugDrawError;

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
  fn build(&self, app: &mut bevy::prelude::App) {
    app
      .insert_resource(EnableLandmassDebug(self.draw_on_start))
      .add_systems(
        Update,
        draw_archipelagos_default::<CS>
          .in_set(LandmassSystemSet::Output)
          .run_if(|enable: Res<EnableLandmassDebug>| enable.0),
      )
      .insert_gizmo_config(
        LandmassGizmoConfigGroup,
        GizmoConfig { depth_bias: -1.0, ..Default::default() },
      );
  }
}

/// A resource controlling whether debug data is drawn.
#[derive(Resource, Default, Deref, DerefMut)]
pub struct EnableLandmassDebug(pub bool);

/// A config group for landmass debug gizmos.
#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct LandmassGizmoConfigGroup;

/// A gizmo debug drawer.
struct GizmoDrawer<'w, 's, 'a, CS: CoordinateSystem>(
  &'a mut Gizmos<'w, 's, LandmassGizmoConfigGroup>,
  PhantomData<CS>,
);

impl<CS: CoordinateSystem> DebugDrawer<CS> for GizmoDrawer<'_, '_, '_, CS> {
  fn add_point(&mut self, point_type: PointType, point: CS::Coordinate) {
    self.0.sphere(
      Isometry3d::new(CS::to_world_position(&point), Quat::IDENTITY),
      0.2,
      match point_type {
        PointType::AgentPosition(_) => Color::srgba(0.0, 1.0, 0.0, 0.6),
        PointType::TargetPosition(_) => Color::srgba(1.0, 1.0, 0.0, 0.6),
        PointType::Waypoint(_) => Color::srgba(0.6, 0.6, 0.6, 0.6),
      },
    );
  }

  fn add_line(&mut self, line_type: LineType, line: [CS::Coordinate; 2]) {
    if line_type == LineType::BoundaryLink {
      let line =
        [CS::to_world_position(&line[0]), CS::to_world_position(&line[1])];
      self.0.cuboid(
        Transform::default()
          .looking_to(line[1] - line[0], bevy::math::Vec3::new(0.0, 1.0, 0.0))
          .with_translation((line[0] + line[1]) * 0.5)
          .with_scale(bevy::math::Vec3::new(
            0.01,
            0.01,
            line[0].distance(line[1]),
          )),
        Color::srgba(0.0, 1.0, 0.0, 0.6),
      );
      return;
    }
    self.0.line(
      CS::to_world_position(&line[0]),
      CS::to_world_position(&line[1]),
      match line_type {
        LineType::BoundaryEdge => Color::srgba(0.0, 0.0, 1.0, 0.6),
        LineType::ConnectivityEdge => Color::srgba(0.5, 0.5, 1.0, 0.6),
        LineType::BoundaryLink => unreachable!(),
        LineType::AgentCorridor(_) => Color::srgba(0.6, 0.0, 0.6, 0.6),
        LineType::Target(_) => Color::srgba(1.0, 1.0, 0.0, 0.6),
        LineType::Waypoint(_) => Color::srgba(0.6, 0.6, 0.6, 0.6),
      },
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
  mut gizmos: Gizmos<'_, '_, LandmassGizmoConfigGroup>,
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

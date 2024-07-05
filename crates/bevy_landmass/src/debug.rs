use crate::{util::landmass_vec3_to_bevy_vec3, Archipelago, LandmassSystemSet};
use bevy::{
  app::Update,
  color::Color,
  gizmos::AppGizmoBuilder,
  math::Quat,
  prelude::{
    Deref, DerefMut, GizmoConfig, GizmoConfigGroup, Gizmos, IntoSystemConfigs,
    Plugin, Query, Res, Resource,
  },
  reflect::Reflect,
  time::Time,
  transform::components::Transform,
};

pub use landmass::debug::*;

/// Draws all parts of `archipelago` to `debug_drawer`. This is a lower level
/// API to allow custom debug drawing. For a pre-made implementation, use
/// [`LandmassDebugPlugin`].
pub fn draw_archipelago_debug(
  archipelago: &crate::Archipelago,
  debug_drawer: &mut impl DebugDrawer,
) {
  landmass::debug::draw_archipelago_debug(
    &archipelago.archipelago,
    debug_drawer,
  )
}

/// A plugin to draw landmass debug data with Bevy gizmos.
pub struct LandmassDebugPlugin {
  /// Whether to begin drawing on startup.
  pub draw_on_start: bool,
}

impl Default for LandmassDebugPlugin {
  fn default() -> Self {
    Self { draw_on_start: true }
  }
}

impl Plugin for LandmassDebugPlugin {
  fn build(&self, app: &mut bevy::prelude::App) {
    app
      .insert_resource(EnableLandmassDebug(self.draw_on_start))
      .add_systems(
        Update,
        draw_archipelagos_default
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
struct GizmoDrawer<'w, 's, 'a>(
  &'a mut Gizmos<'w, 's, LandmassGizmoConfigGroup>,
);

impl<'w, 's, 'a> DebugDrawer for GizmoDrawer<'w, 's, 'a> {
  fn add_point(&mut self, point_type: PointType, point: landmass::Vec3) {
    self.0.sphere(
      landmass_vec3_to_bevy_vec3(point),
      Quat::IDENTITY,
      0.2,
      match point_type {
        PointType::AgentPosition(_) => Color::srgba(0.0, 1.0, 0.0, 0.6),
        PointType::TargetPosition(_) => Color::srgba(1.0, 1.0, 0.0, 0.6),
        PointType::Waypoint(_) => Color::srgba(0.6, 0.6, 0.6, 0.6),
      },
    );
  }

  fn add_line(&mut self, line_type: LineType, line: [landmass::Vec3; 2]) {
    if line_type == LineType::BoundaryLink {
      self.0.cuboid(
        Transform::default()
          .looking_to(
            landmass_vec3_to_bevy_vec3(line[1] - line[0]),
            bevy::math::Vec3::new(0.0, 1.0, 0.0),
          )
          .with_translation(landmass_vec3_to_bevy_vec3(
            (line[0] + line[1]) * 0.5,
          ))
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
      landmass_vec3_to_bevy_vec3(line[0]),
      landmass_vec3_to_bevy_vec3(line[1]),
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
    _triangle: [landmass::Vec3; 3],
  ) {
    // Bevy doesn't have a way to draw triangles :'(
  }
}

/// A system for drawing debug data.
fn draw_archipelagos_default(
  time: Res<Time>,
  archipelagos: Query<&Archipelago>,
  mut gizmos: Gizmos<'_, '_, LandmassGizmoConfigGroup>,
) {
  if time.delta_seconds() == 0.0 {
    return;
  }
  let mut drawer = GizmoDrawer(&mut gizmos);
  for archipelago in archipelagos.iter() {
    draw_archipelago_debug(archipelago, &mut drawer);
  }
}

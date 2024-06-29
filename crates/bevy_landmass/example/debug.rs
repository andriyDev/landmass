use bevy::{input::common_conditions::input_toggle_active, prelude::*};
use bevy_landmass::{debug::*, LandmassSystemSet};

pub struct DebugPlugin;

impl Plugin for DebugPlugin {
  fn build(&self, app: &mut bevy::prelude::App) {
    app
      .add_systems(
        Update,
        draw_archipelagos
          .in_set(LandmassSystemSet::Output)
          .run_if(input_toggle_active(false, KeyCode::F12)),
      )
      .insert_gizmo_group(
        DefaultGizmoConfigGroup,
        GizmoConfig { depth_bias: -1.0, ..Default::default() },
      );
  }
}

struct LyonDrawer<'w, 's, 'a>(&'a mut Gizmos<'w, 's>);

impl<'w, 's, 'a> DebugDrawer for LyonDrawer<'w, 's, 'a> {
  fn add_point(&mut self, point_type: PointType, point: Vec3) {
    self.0.sphere(
      point,
      Quat::IDENTITY,
      0.2,
      match point_type {
        PointType::AgentPosition(_) => Color::rgba(0.0, 1.0, 0.0, 0.6),
        PointType::TargetPosition(_) => Color::rgba(1.0, 1.0, 0.0, 0.6),
        PointType::Waypoint(_) => Color::rgba(0.6, 0.6, 0.6, 0.6),
      },
    );
  }

  fn add_line(&mut self, line_type: LineType, line: [Vec3; 2]) {
    self.0.line(
      line[0],
      line[1],
      match line_type {
        LineType::BoundaryEdge => Color::rgba(0.0, 0.0, 1.0, 0.6),
        LineType::ConnectivityEdge => Color::rgba(0.5, 0.5, 1.0, 0.6),
        LineType::AgentCorridor(_) => Color::rgba(0.6, 0.0, 0.6, 0.6),
        LineType::Target(_) => Color::rgba(1.0, 1.0, 0.0, 0.6),
        LineType::Waypoint(_) => Color::rgba(0.6, 0.6, 0.6, 0.6),
      },
    );
  }

  fn add_triangle(
    &mut self,
    _triangle_type: TriangleType,
    _triangle: [Vec3; 3],
  ) {
    // Bevy doesn't have a way to draw triangles :'(
  }
}

fn draw_archipelagos(
  time: Res<Time>,
  archipelagos: Query<&bevy_landmass::Archipelago>,
  mut gizmos: Gizmos,
) {
  if time.delta_seconds() == 0.0 {
    return;
  }
  let mut drawer = LyonDrawer(&mut gizmos);
  for archipelago in archipelagos.iter() {
    draw_archipelago_debug(archipelago, &mut drawer);
  }
}

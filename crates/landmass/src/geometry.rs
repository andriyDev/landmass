use glam::{FloatExt, Vec2, Vec3, Vec3Swizzles};

pub(crate) fn edge_intersection(
  edge_1: (Vec3, Vec3),
  edge_2: (Vec3, Vec3),
  intersection_distance_squared: f32,
) -> Option<(Vec3, Vec3)> {
  let edge_1_dir = edge_1.1 - edge_1.0;
  let edge_2_on_edge_1_t_ = (
    edge_1_dir.dot(edge_2.0 - edge_1.0) / edge_1_dir.length_squared(),
    edge_1_dir.dot(edge_2.1 - edge_1.0) / edge_1_dir.length_squared(),
  );
  let edge_2_on_edge_1_t = (
    edge_2_on_edge_1_t_.0.clamp(0.0, 1.0),
    edge_2_on_edge_1_t_.1.clamp(0.0, 1.0),
  );
  if edge_2_on_edge_1_t.0 <= edge_2_on_edge_1_t.1 {
    return None;
  }

  let edge_2_dir = edge_2.1 - edge_2.0;
  let edge_1_on_edge_2_t_ = (
    edge_2_dir.dot(edge_1.0 - edge_2.0) / edge_2_dir.length_squared(),
    edge_2_dir.dot(edge_1.1 - edge_2.0) / edge_2_dir.length_squared(),
  );
  let edge_1_on_edge_2_t = (
    edge_1_on_edge_2_t_.0.clamp(0.0, 1.0),
    edge_1_on_edge_2_t_.1.clamp(0.0, 1.0),
  );
  if edge_1_on_edge_2_t.0 <= edge_1_on_edge_2_t.1 {
    return None;
  }

  let edge_2_on_edge_1 = (
    edge_2_on_edge_1_t.0 * edge_1_dir + edge_1.0,
    edge_2_on_edge_1_t.1 * edge_1_dir + edge_1.0,
  );
  let edge_1_on_edge_2 = (
    edge_1_on_edge_2_t.0 * edge_2_dir + edge_2.0,
    edge_1_on_edge_2_t.1 * edge_2_dir + edge_2.0,
  );

  if edge_2_on_edge_1.1.distance_squared(edge_1_on_edge_2.0)
    > intersection_distance_squared
    || edge_2_on_edge_1.0.distance_squared(edge_1_on_edge_2.1)
      > intersection_distance_squared
  {
    None
  } else {
    Some((
      (edge_2_on_edge_1.1 + edge_1_on_edge_2.0) * 0.5,
      (edge_2_on_edge_1.0 + edge_1_on_edge_2.1) * 0.5,
    ))
  }
}

/// Clips `edge` to `triangle` horizontal, ensuring that the
/// `max_vertical_distance` between the triangle and the edge is satisfied.
///
/// Returns the "time" interval along `edge` where the clipping and the
/// "vertical" clipping occurs. Returns `None` if the edge does not intersect
/// the triangle, or the edge is entirely too far from the triangle.
pub(crate) fn clip_edge_to_triangle(
  triangle: (Vec3, Vec3, Vec3),
  edge: (Vec3, Vec3),
  max_vertical_distance: f32,
) -> Option<(f32, f32)> {
  let triangle_2d = (triangle.0.xy(), triangle.1.xy(), triangle.2.xy());
  let edge_2d = (edge.0.xy(), edge.1.xy());

  let triangle_deltas = (
    triangle_2d.1 - triangle_2d.0,
    triangle_2d.2 - triangle_2d.1,
    triangle_2d.0 - triangle_2d.2,
  );
  let edge_delta = edge_2d.1 - edge_2d.0;

  let triangle_off = (
    triangle_2d.0 - edge_2d.0,
    triangle_2d.1 - edge_2d.0,
    triangle_2d.2 - edge_2d.0,
  );

  // Similar to https://gamedev.stackexchange.com/a/63203
  let v0 = triangle_deltas.0;
  let v1 = triangle_deltas.2;
  let denominator = v0.perp_dot(v1);
  let estimate_height = |point: Vec2| {
    let v2 = point - triangle_2d.0;
    let v = v2.perp_dot(v1) / denominator;
    let w = v2.perp_dot(v0) / denominator;
    let u = 1.0 - v - w;
    triangle.0.z * u + triangle.1.z * v + triangle.2.z * w
  };

  let triangle_height_at_edge =
    (estimate_height(edge_2d.0), estimate_height(edge_2d.1));

  fn intersection_t(d1: Vec2, d2: Vec2, off: Vec2) -> f32 {
    let determinant = -d1.perp_dot(d2);
    if determinant == 0.0 {
      return f32::INFINITY;
    }

    Vec2::new(-d2.y, d2.x).dot(off) / determinant
  }

  let edge_t0 = intersection_t(edge_delta, triangle_deltas.0, triangle_off.0);
  let edge_t1 = intersection_t(edge_delta, triangle_deltas.1, triangle_off.1);
  let edge_t2 = intersection_t(edge_delta, triangle_deltas.2, triangle_off.2);

  let mut edge_tmin: f32 = 0.0;
  let mut edge_tmax: f32 = 1.0;

  let mut clip_edge =
    |edge_t: f32, triangle_delta: Vec2, triangle_off: Vec2| {
      if edge_t.is_infinite() {
        if triangle_off.perp_dot(triangle_delta) < 0.0 {
          return false;
        }
      } else if edge_delta.perp_dot(triangle_delta) < 0.0 {
        edge_tmin = edge_tmin.max(edge_t);
      } else {
        edge_tmax = edge_tmax.min(edge_t);
      }
      true
    };

  if !clip_edge(edge_t0, triangle_deltas.0, triangle_off.0) {
    return None;
  }
  if !clip_edge(edge_t1, triangle_deltas.1, triangle_off.1) {
    return None;
  }
  if !clip_edge(edge_t2, triangle_deltas.2, triangle_off.2) {
    return None;
  }

  if edge_tmin >= 1.0 || edge_tmax <= 0.0 {
    return None;
  }

  let triangle_height_tmin =
    triangle_height_at_edge.0.lerp(triangle_height_at_edge.1, edge_tmin);
  let triangle_height_tmax =
    triangle_height_at_edge.0.lerp(triangle_height_at_edge.1, edge_tmax);

  let edge_height_tmin = edge.0.z.lerp(edge.1.z, edge_tmin);
  let edge_height_tmax = edge.0.z.lerp(edge.1.z, edge_tmax);

  let height_slope = (edge_height_tmax - edge_height_tmin)
    - (triangle_height_tmax - triangle_height_tmin);

  if height_slope == 0.0 {
    return ((triangle_height_tmin - edge_height_tmin).abs()
      < max_vertical_distance)
      .then_some((edge_tmin, edge_tmax));
  }

  let t0 = (max_vertical_distance - edge_height_tmin + triangle_height_tmin)
    / height_slope;
  let t1 = (-max_vertical_distance - edge_height_tmin + triangle_height_tmin)
    / height_slope;
  let (mut t0, mut t1) = (t0.min(t1), t0.max(t1));
  if t0 >= 1.0 || t1 <= 0.0 {
    return None;
  }
  (t0, t1) = (t0.max(0.0), t1.min(1.0));
  Some((edge_tmin.lerp(edge_tmax, t0), edge_tmin.lerp(edge_tmax, t1)))
}

#[cfg(test)]
#[path = "geometry_test.rs"]
mod test;

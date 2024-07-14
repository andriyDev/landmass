use glam::Vec3;

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

#[cfg(test)]
#[path = "geometry_test.rs"]
mod test;

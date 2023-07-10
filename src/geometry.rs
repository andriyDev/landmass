use glam::Vec3;

pub fn edge_intersection(
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
mod tests {
  use glam::Vec3;

  use super::edge_intersection;

  #[test]
  fn edge_intersects_when_on_same_line() {
    assert_eq!(
      edge_intersection(
        /* edge_1= */ (Vec3::ONE, Vec3::new(4.0, 2.0, 2.5)),
        /* edge_2= */
        (Vec3::new(7.0, 3.0, 4.0), Vec3::new(3.0, 5.0 / 3.0, 2.0)),
        /* intersection_distance_squared= */ 0.1 * 0.1,
      ),
      Some((Vec3::new(3.0, 5.0 / 3.0, 2.0), Vec3::new(4.0, 2.0, 2.5))),
    );

    assert_eq!(
      edge_intersection(
        /* edge_1= */ (Vec3::ONE, Vec3::new(4.0, 2.0, 2.5)),
        /* edge_2= */
        (Vec3::new(7.0, 3.0, 4.0), Vec3::new(5.0, 7.0 / 3.0, 3.0)),
        /* intersection_distance_squared= */ 0.1 * 0.1,
      ),
      None,
    );
  }

  #[test]
  fn edge_intersects_only_when_nearby() {
    assert_eq!(
      edge_intersection(
        /* edge_1= */
        (Vec3::new(7.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0)),
        /* edge_2= */
        (Vec3::new(4.0, 0.0, 0.09), Vec3::new(6.0, 0.0, 0.09)),
        /* intersection_distance_squared= */ 0.1 * 0.1,
      ),
      Some((Vec3::new(6.0, 0.0, 0.045), Vec3::new(4.0, 0.0, 0.045)))
    );
    assert_eq!(
      edge_intersection(
        /* edge_1= */
        (Vec3::new(7.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0)),
        /* edge_2= */
        (Vec3::new(4.0, 0.0, 0.11), Vec3::new(6.0, 0.0, 0.11)),
        /* intersection_distance_squared= */ 0.1 * 0.1,
      ),
      None,
    );
  }

  #[test]
  fn crossed_edges_dont_intersect_unless_close() {
    assert_eq!(
      edge_intersection(
        /* edge_1= */
        (Vec3::new(5.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0)),
        /* edge_2= */
        (Vec3::new(14.0, 0.0, 4.5), Vec3::new(9.0, 0.0, -0.5)),
        /* intersection_distance_squared= */ 0.51 * 0.51,
      ),
      Some((Vec3::new(9.0, 0.0, -0.25), Vec3::new(9.875, 0.0, 0.125)))
    );
    assert_eq!(
      edge_intersection(
        /* edge_1= */
        (Vec3::new(5.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0)),
        /* edge_2= */
        (Vec3::new(14.0, 0.0, 4.5), Vec3::new(9.0, 0.0, -0.5)),
        /* intersection_distance_squared= */ 0.49 * 0.49,
      ),
      None,
    );
  }
}

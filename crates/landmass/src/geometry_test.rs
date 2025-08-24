use glam::{Vec2, Vec3};
use googletest::{expect_eq, expect_that, matchers::*};

use crate::geometry::project_point_to_line_segment;

use super::{clip_edge_to_triangle, edge_intersection};

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

#[googletest::test]
fn clips_edge_to_triangle_through() {
  let triangle = (
    Vec3::new(1.0, 1.0, 7.0),
    Vec3::new(11.0, 1.0, 7.0),
    Vec3::new(6.0, 11.0, 7.0),
  );

  // Perfectly horizontal, fully across.
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(0.0, 6.0, 7.0), Vec3::new(12.0, 6.0, 7.0)),
      1.0
    ),
    Some((0.29166666, 0.70833333))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(12.0, 6.0, 7.0), Vec3::new(0.0, 6.0, 7.0)),
      1.0
    ),
    Some((0.29166666, 0.70833333))
  );

  // Perfectly vertical, fully across.
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, 0.0, 7.0), Vec3::new(6.0, 12.0, 7.0)),
      1.0
    ),
    Some((1.0 / 12.0, 11.0 / 12.0)),
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, 12.0, 7.0), Vec3::new(6.0, 0.0, 7.0)),
      1.0
    ),
    Some((1.0 / 12.0, 11.0 / 12.0)),
  );
}

#[googletest::test]
fn clips_edge_to_triangle_one_sided() {
  let triangle = (
    Vec3::new(1.0, 1.0, 13.0),
    Vec3::new(11.0, 1.0, 13.0),
    Vec3::new(6.0, 11.0, 13.0),
  );

  // From the center outwards. Only one side gets clipped.

  // Horizontal.
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, 6.0, 13.0), Vec3::new(1.0, 6.0, 13.0)),
      1.0
    ),
    Some((0.0, 0.5))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(1.0, 6.0, 13.0), Vec3::new(6.0, 6.0, 13.0)),
      1.0
    ),
    Some((0.5, 1.0))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, 6.0, 13.0), Vec3::new(11.0, 6.0, 13.0)),
      1.0
    ),
    Some((0.0, 0.5))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(11.0, 6.0, 13.0), Vec3::new(6.0, 6.0, 13.0)),
      1.0
    ),
    Some((0.5, 1.0))
  );

  // Horizontal.
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, 6.0, 13.0), Vec3::new(6.0, -4.0, 13.0)),
      1.0
    ),
    Some((0.0, 0.5))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, -4.0, 13.0), Vec3::new(6.0, 6.0, 13.0)),
      1.0
    ),
    Some((0.5, 1.0))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, 6.0, 13.0), Vec3::new(6.0, 16.0, 13.0)),
      1.0
    ),
    Some((0.0, 0.5))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, 16.0, 13.0), Vec3::new(6.0, 6.0, 13.0)),
      1.0
    ),
    Some((0.5, 1.0))
  );
}

#[googletest::test]
fn clips_edge_entirely_outside() {
  let triangle = (
    Vec3::new(1.0, 1.0, 2.0),
    Vec3::new(11.0, 1.0, 2.0),
    Vec3::new(6.0, 11.0, 2.0),
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(11.0, 6.0, 2.0), Vec3::new(15.0, 6.0, 2.0)),
      1.0
    ),
    None
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(15.0, 6.0, 2.0), Vec3::new(11.0, 6.0, 2.0)),
      1.0
    ),
    None
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(0.0, 6.0, 2.0), Vec3::new(3.0, 6.0, 2.0)),
      1.0
    ),
    None
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(3.0, 6.0, 2.0), Vec3::new(0.0, 6.0, 2.0)),
      1.0
    ),
    None
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, 0.0, 2.0), Vec3::new(6.0, -5.0, 2.0)),
      1.0
    ),
    None
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, -5.0, 2.0), Vec3::new(6.0, 0.0, 2.0)),
      1.0
    ),
    None
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, 12.0, 2.0), Vec3::new(6.0, 15.0, 2.0)),
      1.0
    ),
    None
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(6.0, 15.0, 2.0), Vec3::new(6.0, 12.0, 2.0)),
      1.0
    ),
    None
  );
}

#[googletest::test]
fn clips_edge_to_triangle_against_all_sides() {
  let triangle = (
    Vec3::new(1.0, 1.0, 10.0),
    Vec3::new(11.0, 1.0, 13.0),
    Vec3::new(6.0, 11.0, 20.0),
  );

  let edge_centers = (
    triangle.0.midpoint(triangle.1),
    triangle.1.midpoint(triangle.2),
    triangle.2.midpoint(triangle.0),
  );

  let side_deltas = (
    edge_centers.1 - edge_centers.0,
    edge_centers.2 - edge_centers.1,
    edge_centers.0 - edge_centers.2,
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (
        edge_centers.0 - side_deltas.0 * 0.5,
        edge_centers.1 + side_deltas.0 * 0.5,
      ),
      1.0
    ),
    Some((0.25, 0.75))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (
        edge_centers.1 + side_deltas.0 * 0.5,
        edge_centers.0 - side_deltas.0 * 0.5,
      ),
      1.0
    ),
    Some((0.25, 0.75))
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (
        edge_centers.1 - side_deltas.1 * 0.5,
        edge_centers.2 + side_deltas.1 * 0.5,
      ),
      1.0
    ),
    Some((0.25, 0.75))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (
        edge_centers.2 + side_deltas.1 * 0.5,
        edge_centers.1 - side_deltas.1 * 0.5,
      ),
      1.0
    ),
    Some((0.25, 0.75))
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (
        edge_centers.2 - side_deltas.2 * 0.5,
        edge_centers.0 + side_deltas.2 * 0.5,
      ),
      1.0
    ),
    Some((0.25, 0.75))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (
        edge_centers.0 + side_deltas.2 * 0.5,
        edge_centers.2 - side_deltas.2 * 0.5,
      ),
      1.0
    ),
    Some((0.25, 0.75))
  );
}

#[googletest::test]
fn clips_whole_edge_if_too_far_flat() {
  let triangle = (
    Vec3::new(1.0, 1.0, 5.0),
    Vec3::new(11.0, 1.0, 5.0),
    Vec3::new(6.0, 11.0, 5.0),
  );

  let flat_edge = (Vec2::new(1.0, 6.0), Vec2::new(11.0, 6.0));

  // Ensure that the edge isn't clipped when within the vertical distance.
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (flat_edge.0.extend(1.0), flat_edge.1.extend(1.0)),
      5.0
    ),
    Some((0.25, 0.75))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (flat_edge.0.extend(1.0), flat_edge.1.extend(1.0)),
      3.0
    ),
    None
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (flat_edge.0.extend(9.0), flat_edge.1.extend(9.0)),
      5.0
    ),
    Some((0.25, 0.75))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (flat_edge.0.extend(11.0), flat_edge.1.extend(11.0)),
      5.0
    ),
    None
  );
}

#[googletest::test]
fn clips_whole_edge_if_too_far_sloped() {
  let triangle = (
    Vec3::new(1.0, 1.0, 5.0),
    Vec3::new(11.0, 1.0, 5.0),
    Vec3::new(6.0, 11.0, 5.0),
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(1.0, 6.0, 5.0), Vec3::new(11.0, 6.0, 15.0)),
      2.0
    ),
    None
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(1.0, 6.0, 15.0), Vec3::new(11.0, 6.0, 5.0)),
      2.0
    ),
    None
  );

  // This time the slope is "caused" by the triangle, meaning we needed to track
  // the triangle heights correctly.
  let triangle = (
    Vec3::new(1.0, 1.0, 5.0),
    Vec3::new(11.0, 1.0, 15.0),
    Vec3::new(6.0, 11.0, 10.0),
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(1.0, 6.0, 5.0), Vec3::new(11.0, 6.0, 5.0)),
      2.0
    ),
    None
  );
}

#[googletest::test]
fn clips_edge_part_way_when_vertical_distance_too_great() {
  let triangle = (
    Vec3::new(1.0, 1.0, 5.0),
    Vec3::new(11.0, 1.0, 5.0),
    Vec3::new(6.0, 11.0, 5.0),
  );

  let flat_edge = (Vec2::new(6.0, 0.0), Vec2::new(6.0, 12.0));

  expect_that!(
    clip_edge_to_triangle(
      triangle,
      (flat_edge.0.extend(0.0), flat_edge.1.extend(6.0)),
      2.0
    ),
    some((approx_eq(0.5), approx_eq(11.0 / 12.0)))
  );
  expect_that!(
    clip_edge_to_triangle(
      triangle,
      (flat_edge.0.extend(6.0), flat_edge.1.extend(0.0)),
      2.0
    ),
    some((approx_eq(1.0 / 12.0), approx_eq(0.5)))
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (flat_edge.0.extend(11.0), flat_edge.1.extend(-1.0)),
      3.0
    ),
    Some((0.25, 0.75))
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (flat_edge.0.extend(-1.0), flat_edge.1.extend(11.0)),
      3.0
    ),
    Some((0.25, 0.75))
  );
}

#[googletest::test]
fn clips_edge_part_way_estimating_triangle_height() {
  let triangle = (
    Vec3::new(1.0, 1.0, 0.0),
    Vec3::new(11.0, 1.0, 10.0),
    Vec3::new(6.0, 11.0, 5.0),
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(1.0, 6.0, 5.0), Vec3::new(11.0, 6.0, 5.0)),
      2.0,
    ),
    Some((0.3, 0.7))
  );
  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(11.0, 6.0, 5.0), Vec3::new(1.0, 6.0, 5.0)),
      2.0,
    ),
    Some((0.3, 0.7))
  );
}

#[googletest::test]
fn clips_edge_height_correctly_when_extending_line() {
  let triangle = (
    Vec3::new(0.0, 0.0, 2.0),
    Vec3::new(1.0, 0.0, 2.0),
    Vec3::new(1.0, 1.0, 2.0),
  );

  expect_eq!(
    clip_edge_to_triangle(
      triangle,
      (Vec3::new(0.0, 0.1, 1.5), Vec3::new(0.5, 0.1, 1.5)),
      1.0
    ),
    Some((0.20000005, 1.0))
  );
}

#[googletest::test]
fn projects_point_to_middle_of_line_segment() {
  expect_eq!(
    project_point_to_line_segment(
      Vec3::new(0.0, 0.0, 0.0),
      (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0))
    ),
    (Vec3::new(0.5, 0.5, 0.0), 0.5)
  );
}

#[googletest::test]
fn projects_point_to_start_of_line_segment() {
  expect_eq!(
    project_point_to_line_segment(
      Vec3::new(2.0, -2.0, 0.0),
      (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0))
    ),
    (Vec3::new(1.0, 0.0, 0.0), 0.0)
  );
}

#[googletest::test]
fn projects_point_to_end_of_line_segment() {
  expect_eq!(
    project_point_to_line_segment(
      Vec3::new(-2.0, 2.0, 0.0),
      (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0))
    ),
    (Vec3::new(0.0, 1.0, 0.0), 1.0)
  );
}

#[googletest::test]
fn projects_point_to_degenerate_line_segment() {
  expect_eq!(
    project_point_to_line_segment(
      Vec3::new(-2.0, 2.0, 0.0),
      (Vec3::new(10.0, 3.0, 0.0), Vec3::new(10.0, 3.0, 0.0))
    ),
    (Vec3::new(10.0, 3.0, 0.0), 0.0)
  );
}

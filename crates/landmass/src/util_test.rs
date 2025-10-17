use std::f32::consts::PI;

use glam::Vec3;

use crate::util::{BoundingBox, CoreTransform, RaySegment};

use super::BoundingBoxHierarchy;

#[test]
fn bounding_box_expands_to_points() {
  let mut bbox = BoundingBox::Empty;

  let starter_point = Vec3::new(3.0, 5.0, -10.0);
  bbox = bbox.expand_to_point(starter_point);
  assert_eq!(bbox.as_box(), (starter_point, starter_point));

  // Other corner of box.
  bbox = bbox.expand_to_point(Vec3::new(1.0, 7.0, -8.0));
  assert_eq!(
    bbox.as_box(),
    (Vec3::new(1.0, 5.0, -10.0), Vec3::new(3.0, 7.0, -8.0))
  );

  let max_bound = Vec3::new(30.0, 30.0, 30.0);
  bbox = bbox.expand_to_point(max_bound);
  assert_eq!(bbox.as_box(), (Vec3::new(1.0, 5.0, -10.0), max_bound));

  let min_bound = Vec3::new(-10.0, -10.0, -10.0);
  bbox = bbox.expand_to_point(min_bound);
  assert_eq!(bbox.as_box(), (min_bound, max_bound));

  // Completely contained in box so no change.
  bbox = bbox.expand_to_point(Vec3::new(3.0, -1.0, 3.0));
  assert_eq!(bbox.as_box(), (min_bound, max_bound));
}

#[test]
fn bounding_box_expands_to_other_bounds() {
  let mut bbox = BoundingBox::Empty;

  let starter_box_max = Vec3::new(10.0, 9.0, 8.0);
  let starter_box =
    BoundingBox::new_box(Vec3::new(1.0, 2.0, 3.0), starter_box_max);
  bbox = bbox.expand_to_bounds(&starter_box);
  assert_eq!(bbox, starter_box);

  let disjoint_box_min = Vec3::new(-1.0, -2.0, -3.0);
  let disjoint_box =
    BoundingBox::new_box(disjoint_box_min, Vec3::new(0.0, 1.0, 2.0));
  bbox = bbox.expand_to_bounds(&disjoint_box);
  assert_eq!(bbox.as_box(), (disjoint_box_min, starter_box_max));

  let contained_box =
    BoundingBox::new_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(3.0, 3.0, 3.0));
  bbox = bbox.expand_to_bounds(&contained_box);
  assert_eq!(bbox.as_box(), (disjoint_box_min, starter_box_max));

  let intersected_box = BoundingBox::new_box(
    Vec3::new(0.0, -100.0, 0.0),
    Vec3::new(3.0, 100.0, 3.0),
  );
  bbox = bbox.expand_to_bounds(&intersected_box);
  assert_eq!(
    bbox.as_box(),
    (Vec3::new(-1.0, -100.0, -3.0), Vec3::new(10.0, 100.0, 8.0))
  );
}

#[test]
fn bounding_box_detects_containment() {
  let bbox =
    BoundingBox::new_box(Vec3::new(-1.0, -2.0, -3.0), Vec3::new(5.0, 4.0, 3.0));

  assert!(bbox.contains_point(Vec3::ZERO));
  assert!(bbox.contains_point(Vec3::new(5.0, 4.0, 3.0)));
  assert!(bbox.contains_point(Vec3::new(4.0, 3.0, 2.0)));
  assert!(bbox.contains_point(Vec3::new(-0.5, -1.0, -1.0)));

  assert!(!bbox.contains_point(Vec3::new(6.0, 0.0, 0.0)));
  assert!(!bbox.contains_point(Vec3::new(-6.0, 0.0, 0.0)));
  assert!(!bbox.contains_point(Vec3::new(0.0, 6.0, 0.0)));
  assert!(!bbox.contains_point(Vec3::new(0.0, -6.0, 0.0)));
  assert!(!bbox.contains_point(Vec3::new(0.0, 0.0, 6.0)));
  assert!(!bbox.contains_point(Vec3::new(0.0, 0.0, -6.0)));

  assert!(!bbox.contains_bounds(&BoundingBox::Empty));
  assert!(!BoundingBox::Empty.contains_bounds(&bbox));

  assert!(bbox.contains_bounds(&BoundingBox::new_box(
    Vec3::new(-0.5, -1.0, -2.0),
    Vec3::new(1.0, 2.0, 3.0)
  )));
  assert!(!bbox.contains_bounds(&BoundingBox::new_box(
    Vec3::new(-6.0, 0.0, 0.0),
    Vec3::new(0.1, 0.1, 0.1),
  )));
  assert!(!bbox.contains_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, -6.0, 0.0),
    Vec3::new(0.1, 0.1, 0.1),
  )));
  assert!(!bbox.contains_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, 0.0, -6.0),
    Vec3::new(0.1, 0.1, 0.1),
  )));
  assert!(!bbox.contains_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(6.0, 0.1, 0.1),
  )));
  assert!(!bbox.contains_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(0.1, 6.0, 0.1),
  )));
  assert!(!bbox.contains_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(0.1, 0.1, 6.0),
  )));
}

#[test]
fn bounding_box_detects_intersection() {
  let bbox =
    BoundingBox::new_box(Vec3::new(-1.0, -2.0, -3.0), Vec3::new(5.0, 4.0, 3.0));

  assert!(bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(-0.5, -1.0, -2.0),
    Vec3::new(1.0, 2.0, 3.0)
  )));
  assert!(bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(-6.0, 0.0, 0.0),
    Vec3::new(0.1, 0.1, 0.1),
  )));
  assert!(bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, -6.0, 0.0),
    Vec3::new(0.1, 0.1, 0.1),
  )));
  assert!(bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, 0.0, -6.0),
    Vec3::new(0.1, 0.1, 0.1),
  )));
  assert!(bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(6.0, 0.1, 0.1),
  )));
  assert!(bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(0.1, 6.0, 0.1),
  )));
  assert!(bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(0.1, 0.1, 6.0),
  )));

  assert!(!bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(5.9, 0.0, 0.0),
    Vec3::new(6.0, 6.0, 6.0),
  )));
  assert!(!bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, 5.9, 0.0),
    Vec3::new(6.0, 6.0, 6.0),
  )));
  assert!(!bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(0.0, 0.0, 5.9),
    Vec3::new(6.0, 6.0, 6.0),
  )));
  assert!(!bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(-6.0, -6.0, -6.0),
    Vec3::new(-5.9, 0.0, 0.0),
  )));
  assert!(!bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(-6.0, -6.0, -6.0),
    Vec3::new(0.0, -5.9, 0.0),
  )));
  assert!(!bbox.intersects_bounds(&BoundingBox::new_box(
    Vec3::new(-6.0, -6.0, -6.0),
    Vec3::new(0.0, 0.0, -5.9),
  )));
}

#[test]
fn bounding_box_detects_ray_segment_intersection() {
  let bbox =
    BoundingBox::new_box(Vec3::new(-1.0, -2.0, -3.0), Vec3::new(5.0, 4.0, 3.0));

  fn test(
    bbox: &BoundingBox,
    v0: (f32, f32, f32),
    v1: (f32, f32, f32),
    expected_result: bool,
  ) -> bool {
    let v0 = Vec3::new(v0.0, v0.1, v0.2);
    let v1 = Vec3::new(v1.0, v1.1, v1.2);
    if bbox.intersects_ray_segment(&RaySegment::new(v0, v1)) != expected_result
    {
      return false;
    }
    if bbox.intersects_ray_segment(&RaySegment::new(v1, v0)) != expected_result
    {
      return false;
    }
    true
  }

  // Axis aligned.
  // X axis
  assert!(test(&bbox, (2.0, 2.0, 2.0), (4.0, 2.0, 2.0), true));
  assert!(test(&bbox, (-3.0, 2.0, 2.0), (6.0, 2.0, 2.0), true));
  assert!(test(&bbox, (-10.0, 2.0, 2.0), (2.0, 2.0, 2.0), true));
  assert!(test(&bbox, (2.0, 2.0, 2.0), (10.0, 2.0, 2.0), true));
  assert!(test(&bbox, (-10.0, 2.0, 2.0), (-3.0, 2.0, 2.0), false));
  assert!(test(&bbox, (6.0, 2.0, 2.0), (10.0, 2.0, 2.0), false));
  // Y axis
  assert!(test(&bbox, (2.0, 2.0, 2.0), (2.0, 4.0, 2.0), true));
  assert!(test(&bbox, (2.0, -3.0, 2.0), (2.0, 6.0, 2.0), true));
  assert!(test(&bbox, (2.0, -10.0, 2.0), (2.0, 2.0, 2.0), true));
  assert!(test(&bbox, (2.0, 2.0, 2.0), (2.0, 10.0, 2.0), true));
  assert!(test(&bbox, (2.0, -10.0, 2.0), (2.0, -3.0, 2.0), false));
  assert!(test(&bbox, (2.0, 6.0, 2.0), (2.0, 10.0, 2.0), false));
  // Z axis
  assert!(test(&bbox, (2.0, 2.0, 2.0), (2.0, 2.0, 4.0), true));
  assert!(test(&bbox, (2.0, 2.0, -3.0), (2.0, 2.0, 6.0), true));
  assert!(test(&bbox, (2.0, 2.0, -10.0), (2.0, 2.0, 2.0), true));
  assert!(test(&bbox, (2.0, 2.0, 2.0), (2.0, 2.0, 10.0), true));
  assert!(test(&bbox, (2.0, 2.0, -10.0), (2.0, 2.0, -3.0), false));
  assert!(test(&bbox, (2.0, 2.0, 6.0), (2.0, 2.0, 10.0), false));

  // Diagonals through box.
  assert!(test(&bbox, (-2.0, -3.0, -4.0), (6.0, 5.0, 4.0), true));
  assert!(test(&bbox, (6.0, -3.0, -4.0), (-2.0, 5.0, 4.0), true));
  assert!(test(&bbox, (-2.0, 5.0, -4.0), (6.0, -3.0, 4.0), true));
  assert!(test(&bbox, (-2.0, -3.0, 4.0), (6.0, 5.0, -4.0), true));
  // Arbitrary diagonals outside box.
  assert!(test(&bbox, (8.0, -13.0, -4.0), (-12.0, 7.0, -4.0), false));
  assert!(test(&bbox, (16.0, -5.0, 4.0), (-4.0, 15.0, 4.0), false));
}

#[test]
fn transform_empty_does_nothing() {
  assert_eq!(
    BoundingBox::Empty.transform(&CoreTransform {
      translation: Vec3::new(1.0, 2.0, 3.0),
      rotation: 0.75
    }),
    BoundingBox::Empty
  );
}

#[test]
fn transforms_bounds() {
  let root_2 = 2.0f32.sqrt();
  let (actual_min, actual_max) =
    BoundingBox::new_box(Vec3::new(1.0, 3.0, 2.0), Vec3::new(6.0, 4.0, 5.0))
      .transform(&CoreTransform {
        translation: Vec3::new(-4.0, 1.0, -3.0),
        rotation: PI * -0.75,
      })
      .as_box();
  let expected_min = Vec3::new(-3.0 / root_2 - 4.0, -10.0 / root_2 + 1.0, -1.0);
  assert!(
    actual_min.abs_diff_eq(expected_min, 1e-6),
    "actual_min={actual_min} expected_min={expected_min}"
  );
  let expected_max = Vec3::new(3.0 / root_2 - 4.0, -4.0 / root_2 + 1.0, 2.0);
  assert!(
    actual_max.abs_diff_eq(expected_max, 1e-6),
    "actual_max={actual_max} expected_max={expected_max}"
  );
}

#[test]
fn octant_bounding_box_hierarchy() {
  let mut values = vec![
    (
      BoundingBox::new_box(Vec3::new(1.0, 1.0, 1.0), Vec3::new(4.0, 4.0, 4.0)),
      Some(0),
    ),
    (
      BoundingBox::new_box(Vec3::new(5.0, 1.0, 1.0), Vec3::new(8.0, 4.0, 4.0)),
      Some(1),
    ),
    (
      BoundingBox::new_box(Vec3::new(1.0, 5.0, 1.0), Vec3::new(4.0, 8.0, 4.0)),
      Some(2),
    ),
    (
      BoundingBox::new_box(Vec3::new(5.0, 5.0, 1.0), Vec3::new(8.0, 8.0, 4.0)),
      Some(3),
    ),
    (
      BoundingBox::new_box(Vec3::new(1.0, 1.0, 5.0), Vec3::new(4.0, 4.0, 8.0)),
      Some(4),
    ),
    (
      BoundingBox::new_box(Vec3::new(5.0, 1.0, 5.0), Vec3::new(8.0, 4.0, 8.0)),
      Some(5),
    ),
    (
      BoundingBox::new_box(Vec3::new(1.0, 5.0, 5.0), Vec3::new(4.0, 8.0, 8.0)),
      Some(6),
    ),
    (
      BoundingBox::new_box(Vec3::new(5.0, 5.0, 5.0), Vec3::new(8.0, 8.0, 8.0)),
      Some(7),
    ),
  ];

  let bbh = BoundingBoxHierarchy::new(&mut values);

  assert_eq!(bbh.depth(), 4);

  assert!(bbh.query_box(BoundingBox::Empty).is_empty());
  assert_eq!(
    bbh.query_box(BoundingBox::new_box(
      Vec3::new(1.0, 5.0, 1.0),
      Vec3::new(4.0, 8.0, 4.0),
    )),
    [&2]
  );
  assert_eq!(
    bbh.query_box(BoundingBox::new_box(
      Vec3::new(1.0, 5.0, 1.0),
      Vec3::new(6.0, 8.0, 4.0),
    )),
    [&2, &3]
  );
  assert_eq!(
    bbh.query_box(BoundingBox::new_box(
      Vec3::new(2.0, 2.0, 2.0),
      Vec3::new(7.0, 7.0, 7.0),
    )),
    [&0, &1, &2, &3, &4, &5, &6, &7]
  );
  assert_eq!(
    bbh.query_ray_segment(RaySegment::new(
      Vec3::new(1.0, 5.0, 1.0),
      Vec3::new(4.0, 8.0, 4.0),
    )),
    [&2]
  );
  assert_eq!(
    bbh.query_ray_segment(RaySegment::new(
      Vec3::new(1.0, 5.0, 1.0),
      Vec3::new(6.0, 8.0, 4.0),
    )),
    [&2, &3]
  );
  assert_eq!(
    bbh.query_ray_segment(RaySegment::new(
      Vec3::new(2.0, 2.0, 2.0),
      Vec3::new(7.0, 7.0, 7.0),
    )),
    [&0, &7]
  );
}

#[test]
fn bounding_box_hierarchy_with_same_big_dimension() {
  let mut values = vec![
    (
      BoundingBox::new_box(Vec3::new(1.0, 1.0, 1.0), Vec3::new(2.0, 2.0, 11.0)),
      Some(0),
    ),
    (
      BoundingBox::new_box(Vec3::new(4.0, 1.0, 1.0), Vec3::new(5.0, 2.0, 11.0)),
      Some(1),
    ),
  ];

  let bbh = BoundingBoxHierarchy::new(&mut values);
  assert_eq!(
    bbh.query_box(BoundingBox::new_box(
      Vec3::new(1.5, 1.5, 1.5),
      Vec3::new(1.5, 1.5, 1.5)
    )),
    [&0],
  );
}

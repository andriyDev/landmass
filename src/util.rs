use glam::Vec3;

#[derive(PartialEq, Clone, Copy, Debug)]
pub enum BoundingBox {
  Empty,
  Box { min: Vec3, max: Vec3 },
}

impl BoundingBox {
  pub fn new_box(min: Vec3, max: Vec3) -> Self {
    Self::Box { min, max }
  }

  pub fn as_box(&self) -> (Vec3, Vec3) {
    match self {
      Self::Empty => panic!("BoundingBox is not a box."),
      &Self::Box { min, max } => (min, max),
    }
  }

  pub fn size(&self) -> Vec3 {
    match self {
      Self::Empty => Vec3::ZERO,
      &Self::Box { min, max } => max - min,
    }
  }

  pub fn is_valid(&self) -> bool {
    let size = self.size();
    size.x >= 0.0 && size.y >= 0.0 && size.z >= 0.0
  }

  pub fn expand_to_bounds(&self, other: &Self) -> Self {
    match (self, other) {
      (Self::Empty, Self::Empty) => Self::Empty,
      (Self::Box { .. }, Self::Empty) => *self,
      (Self::Empty, Self::Box { .. }) => *other,
      (
        Self::Box { min, max },
        Self::Box { min: other_min, max: other_max },
      ) => Self::Box { min: min.min(*other_min), max: max.max(*other_max) },
    }
  }

  pub fn expand_to_point(&self, point: Vec3) -> Self {
    match self {
      Self::Empty => Self::Box { min: point, max: point },
      &Self::Box { min, max } => {
        Self::Box { min: min.min(point), max: max.max(point) }
      }
    }
  }

  pub fn expand_by_size(&self, size: Vec3) -> BoundingBox {
    let expanded_box = match self {
      BoundingBox::Empty => BoundingBox::Empty,
      &BoundingBox::Box { min, max } => {
        BoundingBox::Box { min: min - size, max: max + size }
      }
    };

    if !expanded_box.is_valid() {
      return BoundingBox::Empty;
    }

    expanded_box
  }

  pub fn contains_point(&self, point: Vec3) -> bool {
    match self {
      Self::Empty => false,
      Self::Box { min, max } => {
        min.x <= point.x
          && point.x <= max.x
          && min.y <= point.y
          && point.y <= max.y
          && min.z <= point.z
          && point.z <= max.z
      }
    }
  }

  pub fn contains_bounds(&self, other: &Self) -> bool {
    let (other_min, other_max) = match other {
      Self::Empty => return false,
      Self::Box { min, max } => (min, max),
    };
    match self {
      Self::Empty => false,
      Self::Box { min, max } => {
        min.x <= other_min.x
          && other_max.x <= max.x
          && min.y <= other_min.y
          && other_max.y <= max.y
          && min.z <= other_min.z
          && other_max.z <= max.z
      }
    }
  }

  pub fn intersects_bounds(&self, other: &Self) -> bool {
    let (other_min, other_max) = match other {
      Self::Empty => return false,
      Self::Box { min, max } => (min, max),
    };
    match self {
      Self::Empty => false,
      Self::Box { min, max } => {
        min.x <= other_max.x
          && other_min.x <= max.x
          && min.y <= other_max.y
          && other_min.y <= max.y
          && min.z <= other_max.z
          && other_min.z <= max.z
      }
    }
  }
}

#[cfg(test)]
mod tests {
  use glam::Vec3;

  use crate::BoundingBox;

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
    let bbox = BoundingBox::new_box(
      Vec3::new(-1.0, -2.0, -3.0),
      Vec3::new(5.0, 4.0, 3.0),
    );

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
    let bbox = BoundingBox::new_box(
      Vec3::new(-1.0, -2.0, -3.0),
      Vec3::new(5.0, 4.0, 3.0),
    );

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
}
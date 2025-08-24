use slotmap::new_key_type;

use crate::CoordinateSystem;

new_key_type! {
  /// The ID of an [`AnimationLink`].
  pub struct AnimationLinkId;
}

/// A link connecting two edges where an agent must perform some action (or
/// animation) to use the link.
///
/// This is often referred to as an off-mesh link in other navigation systems.
pub struct AnimationLink<CS: CoordinateSystem> {
  /// The edge that the agent must reach to use the animation link.
  ///
  /// The order of the edge is arbitrary.
  pub start_edge: (CS::Coordinate, CS::Coordinate),
  /// The edge that the agent will be sent to after using the animation link.
  ///
  /// The order of the edge must match the order of `start_edge`. So
  /// `start_edge.0` will take the agent to `end_edge.0` and the same for `.1`.
  pub end_edge: (CS::Coordinate, CS::Coordinate),
  /// The kind of the animation link.
  ///
  /// This is an arbitrary number that can be filtered on.
  pub kind: usize,
}

/// The state of an animation link.
pub(crate) struct AnimationLinkState<CS: CoordinateSystem> {
  /// The link given to us by the user.
  pub(crate) main_link: AnimationLink<CS>,
}

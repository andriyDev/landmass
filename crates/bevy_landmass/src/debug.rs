pub use landmass::debug::*;

/// Draws all parts of `archipelago` to `debug_drawer`.
pub fn draw_archipelago_debug(
  archipelago: &crate::Archipelago,
  debug_drawer: &mut impl DebugDrawer,
) {
  landmass::debug::draw_archipelago_debug(
    &archipelago.archipelago,
    debug_drawer,
  )
}

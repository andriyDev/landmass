use bevy::prelude::{Component, Plugin, Query};

struct LandmassPlugin;

impl Plugin for LandmassPlugin {
  fn build(&self, app: &mut bevy::prelude::App) {
    app.add_system(update_archipelagos);
  }
}

#[derive(Component)]
pub struct Archipelago {
  pub archipelago: landmass::Archipelago,
}

fn update_archipelagos(mut archipelago_query: Query<&mut Archipelago>) {
  for mut archipelago in archipelago_query.iter_mut() {
    archipelago.archipelago.update();
  }
}

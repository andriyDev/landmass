use std::collections::HashMap;

use bevy::prelude::{Component, Entity, Plugin, Query};
use landmass::AgentId;

pub struct LandmassPlugin;

impl Plugin for LandmassPlugin {
  fn build(&self, app: &mut bevy::prelude::App) {
    app.add_system(update_archipelagos).add_system(add_agents_to_archipelagos);
  }
}

#[derive(Component)]
pub struct Archipelago {
  pub archipelago: landmass::Archipelago,
  pub agents: HashMap<Entity, AgentId>,
}

fn update_archipelagos(mut archipelago_query: Query<&mut Archipelago>) {
  for mut archipelago in archipelago_query.iter_mut() {
    archipelago.archipelago.update();
  }
}

#[derive(Component)]
pub struct Agent {
  pub radius: f32,
  pub max_velocity: f32,
}

#[derive(Component)]
pub struct ArchipelagoRef(pub Entity);

fn add_agents_to_archipelagos(
  mut archipelago_query: Query<(Entity, &mut Archipelago)>,
  agent_query: Query<(Entity, &Agent, &ArchipelagoRef)>,
) {
  let mut archipelago_to_agents = HashMap::<_, HashMap<_, _>>::new();
  for (entity, agent, archipleago_ref) in agent_query.iter() {
    archipelago_to_agents
      .entry(archipleago_ref.0)
      .or_default()
      .insert(entity, agent);
  }

  for (archipelago_entity, mut archipelago) in archipelago_query.iter_mut() {
    let mut new_agent_map = archipelago_to_agents
      .remove(&archipelago_entity)
      .unwrap_or_else(|| HashMap::new());
    let archipelago = archipelago.as_mut();

    // Remove any agents that aren't in the `new_agent_map`. Also remove any
    // agents from the `new_agent_map` that are in the archipelago.
    archipelago.agents.retain(|agent_entity, agent_id| {
      match new_agent_map.remove(&agent_entity) {
        None => {
          archipelago.archipelago.remove_agent(*agent_id);
          false
        }
        Some(_) => true,
      }
    });

    for (new_agent_entity, new_agent) in new_agent_map.drain() {
      let agent_id =
        archipelago.archipelago.add_agent(landmass::Agent::create(
          /* position= */ glam::Vec3::ZERO,
          /* velocity= */ glam::Vec3::ZERO,
          new_agent.radius,
          new_agent.max_velocity,
        ));
      archipelago.agents.insert(new_agent_entity, agent_id);
    }
  }
}

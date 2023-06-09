use std::collections::HashMap;

use bevy::prelude::{
  Bundle, Component, Entity, GlobalTransform, IntoSystemConfig,
  IntoSystemSetConfig, Plugin, Query, SystemSet, Vec3, With,
};
use landmass::AgentId;
use util::{bevy_vec3_to_glam_vec3, glam_vec3_to_bevy_vec3};

mod util;

pub struct LandmassPlugin;

// A bundle to create agents. This omits the GlobalTransform component, since
// this is commonly added in other bundles (which is redundant and can override
// previous bundles).
#[derive(Bundle)]
pub struct AgentBundle {
  pub agent: Agent,
  pub archipelago_ref: ArchipelagoRef,
  pub velocity: AgentVelocity,
  pub desired_velocity: AgentDesiredVelocity,
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum LandmassSystemSet {
  SyncExistence,
  SyncValues,
  Update,
  Output,
}

impl Plugin for LandmassPlugin {
  fn build(&self, app: &mut bevy::prelude::App) {
    app.configure_sets((
      LandmassSystemSet::SyncExistence.before(LandmassSystemSet::SyncValues),
      LandmassSystemSet::SyncValues.before(LandmassSystemSet::Update),
      LandmassSystemSet::Update.before(LandmassSystemSet::Output),
    ));
    app.add_system(
      add_agents_to_archipelagos.in_set(LandmassSystemSet::SyncExistence),
    );
    app.add_system(
      sync_transform_and_velocity.in_set(LandmassSystemSet::SyncValues),
    );
    app.add_system(update_archipelagos.in_set(LandmassSystemSet::Update));
    app.add_system(sync_desired_velocity.in_set(LandmassSystemSet::Output));
  }
}

#[derive(Component)]
pub struct Archipelago {
  archipelago: landmass::Archipelago,
  agents: HashMap<Entity, AgentId>,
}

impl Archipelago {
  pub fn new(mut landmass_archipelago: landmass::Archipelago) -> Self {
    for agent_id in landmass_archipelago.get_agent_ids().collect::<Vec<_>>() {
      landmass_archipelago.remove_agent(agent_id);
    }
    Self { archipelago: landmass_archipelago, agents: HashMap::new() }
  }

  fn get_agent(&self, entity: Entity) -> &landmass::Agent {
    self.archipelago.get_agent(*self.agents.get(&entity).unwrap())
  }

  fn get_agent_mut(&mut self, entity: Entity) -> &mut landmass::Agent {
    self.archipelago.get_agent_mut(*self.agents.get(&entity).unwrap())
  }
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

#[derive(Component, Default)]
pub struct AgentVelocity(pub Vec3);

#[derive(Component, Default)]
pub struct AgentDesiredVelocity(Vec3);

impl AgentDesiredVelocity {
  pub fn velocity(&self) -> Vec3 {
    self.0
  }
}

fn add_agents_to_archipelagos(
  mut archipelago_query: Query<(Entity, &mut Archipelago)>,
  agent_query: Query<(Entity, &Agent, &ArchipelagoRef), With<GlobalTransform>>,
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

fn sync_transform_and_velocity(
  agent_query: Query<
    (Entity, &ArchipelagoRef, &GlobalTransform, Option<&AgentVelocity>),
    With<Agent>,
  >,
  mut archipelago_query: Query<&mut Archipelago>,
) {
  for (agent_entity, &ArchipelagoRef(arch_entity), transform, velocity) in
    agent_query.iter()
  {
    let mut archipelago = match archipelago_query.get_mut(arch_entity) {
      Err(_) => continue,
      Ok(arch) => arch,
    };

    let agent = archipelago.get_agent_mut(agent_entity);
    agent.set_position(bevy_vec3_to_glam_vec3(transform.translation()));
    if let Some(AgentVelocity(velocity)) = velocity {
      agent.set_velocity(bevy_vec3_to_glam_vec3(*velocity));
    }
  }
}

fn sync_desired_velocity(
  mut agent_query: Query<
    (Entity, &ArchipelagoRef, &mut AgentDesiredVelocity),
    With<Agent>,
  >,
  archipelago_query: Query<&Archipelago>,
) {
  for (agent_entity, &ArchipelagoRef(arch_entity), mut desired_velocity) in
    agent_query.iter_mut()
  {
    let archipelago = match archipelago_query.get(arch_entity).ok() {
      None => continue,
      Some(arch) => arch,
    };

    desired_velocity.0 = glam_vec3_to_bevy_vec3(
      archipelago.get_agent(agent_entity).get_desired_velocity(),
    );
  }
}

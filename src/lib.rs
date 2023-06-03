mod agent;
mod nav_mesh;
mod path;
mod pathfinding;
mod util;

use rand::Rng;
use std::collections::HashMap;

use nav_mesh::ValidNavigationMesh;

pub use agent::{Agent, AgentId};
pub use util::BoundingBox;

pub struct Archipelago {
  nav_data: NavigationData,
  agents: HashMap<AgentId, Agent>,
}

struct NavigationData {
  nav_mesh: ValidNavigationMesh,
}

impl Archipelago {
  pub fn create_from_navigation_mesh(
    navigation_mesh: ValidNavigationMesh,
  ) -> Self {
    Self {
      nav_data: NavigationData { nav_mesh: navigation_mesh },
      agents: HashMap::new(),
    }
  }

  pub fn add_agent(&mut self, agent: Agent) -> AgentId {
    let mut rng = rand::thread_rng();

    let agent_id: AgentId = rng.gen();
    assert!(self.agents.insert(agent_id, agent).is_none());

    agent_id
  }

  pub fn get_agent(&self, agent_id: AgentId) -> &Agent {
    self.agents.get(&agent_id).unwrap()
  }

  pub fn get_agent_mut(&mut self, agent_id: AgentId) -> &mut Agent {
    self.agents.get_mut(&agent_id).unwrap()
  }
}

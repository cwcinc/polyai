mod ammo;
mod ammo_sys;
mod sim;

use sim::SimState;

fn main() {
    let sim_state = SimState::new(
        include_str!("../trackParts.json"),
        include_str!("../carCollisionShapeVertices.json"),
    );
}

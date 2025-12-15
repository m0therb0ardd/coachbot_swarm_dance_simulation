## Swarm Simulator Overview

This repository contains a collection of swarm control algorithms designed for the CoachBot platform and the CoachBot Simulator. Each algorithm encodes a distinct swarm choreography—such as encircling, flocking, dispersion, or wave-like motion—by defining how individual robots respond to their neighbors, the dancer, and the shared performance space.

## Algorithm Deployment & Execution on Bots 
These algorithms are written to run unchanged in both simulation and on physical CoachBots, enabling rapid iteration in the simulator before deployment on hardware.
When run on real hardware, apply_from_json.py reads from the output of the Random Forest gesture classifier to select and deploy the corresponding swarm algorithm. This enables a live, closed-loop system in which the dancer’s movement dynamically reshapes the collective behavior of the robots.

## Intended Use

These scripts are designed for:
The CoachBot Simulator, for testing and visualizing swarm behaviors
The physical CoachBot testbed, for live performance and embodied interaction
Together, they form a choreography engine that supports both experimental development and real-time performance with human collaborators.

## Brief Overview

This repository contains swarm choreography scripts developed for the CoachBot platform and CoachBot Simulator.

- `00_move_to_circle.py`  
  Moves all designated robots into an evenly spaced circular ring used as the starting formation.

- `0_directional_left.py`, `0_directional_right.py`  
  Simple directional motion behaviors used for legible, gesture-driven control.

- `1_glitch.py`  
  A disruption-based swarm behavior associated with the “hands up” gesture.

- `2_encircle.py`  
  Encircling behavior in which robots orbit a shared center.

- `3_glide.py`  
  Light, sustained swarm motion corresponding to the *glide* gesture.

- `4_punch.py`  
  Sharp, direct, high-energy swarm response.

- `5_float.py`  
  Slow, indirect, buoyant swarm motion.

- `6_slash.py`  
  Fast, indirect, and forceful swarm behavior.

This repository is a clone of the Northwestern CoachBot Simulator, based on the original simulation framework and documentation  
(https://github.com/michelleezhang/swarm-simulator) and the accompanying README  
(https://github.com/m0therb0ardd/coachbot_swarm_dance_simulation/blob/main/README1.md).  
All custom work lives in `sim_pkg/user/`, where the swarm algorithms are implemented and tested in simulation before deployment to physical CoachBots.

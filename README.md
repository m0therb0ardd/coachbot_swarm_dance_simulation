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
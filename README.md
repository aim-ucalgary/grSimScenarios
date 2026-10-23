# grSim scenarios

This repository contains various scenarios for the grSim robot soccer simulator. The standardized format is detailed here. Save your scenario files in this repository following the specified format, while naming them descriptively to indicate the scenario they represent. 

- `aaron/template.py`: executable four-direction velocity example and starting point for new scenarios. Use this as a base when creating new scenario files.
- `helpers/`: will contain utility functions to assist in creating scenarios.

Simulation installation details can be found in the AIM google drive, under the Software Division folder.

## Scenario writing standard

Every scenario should include:

1. **Purpose:** What it demonstrates or tests.
2. **Parameters:** Robots, connections, speeds, and timing.
3. **Setup:** Initial scene and simulation settings.
4. **Actions:** Named steps with explicit units and coordinate frames.
5. **Results:** State what was sent, observed, or measured.

## Organization

Save scenario files in the your personal directory (e.g., `aaron/`) to keep them organized and easily accessible.
Scenario files should be named descriptively to indicate the scenario they represent. For example:

- `simon/four_direction_velocity.py`
- `zain/obstacle_avoidance.py`

# Running

To run a scenario, save it in this repository and execute it using Python on Linux.
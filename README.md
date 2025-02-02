![Crazyswarm ROS CI](https://github.com/USC-ACTLab/crazyswarm/workflows/Crazyswarm%20ROS%20CI/badge.svg)
![Sim-Only Conda CI](https://github.com/USC-ACTLab/crazyswarm/workflows/Sim-Only%20Conda%20CI/badge.svg)
[![Documentation Status](https://readthedocs.org/projects/crazyswarm/badge/?version=latest)](https://crazyswarm.readthedocs.io/en/latest/?badge=latest)

# Crazyswarm
A Large Nano-Quadcopter Swarm.


> **Warning**
> This repository is not actively maintained anymore. For new projects, please consider [Crazyswarm2](https://imrclab.github.io/crazyswarm2/).


The documentation is available here: http://crazyswarm.readthedocs.io/en/latest/.

## Troubleshooting
Please start a [Discussion](https://github.com/USC-ACTLab/crazyswarm/discussions) for...

- Getting Crazyswarm to work with your hardware setup.
- Advice on how to use the [Crazyswarm Python API](https://crazyswarm.readthedocs.io/en/latest/api.html) to achieve your goals.
- Rough ideas for a new feature.

Please open an [Issue](https://github.com/USC-ACTLab/crazyswarm/issues) if you believe that fixing your problem will involve a **change in the Crazyswarm source code**, rather than your own configuration files. For example...

- Bug reports.
- New feature proposals with details.

## Added Features
- `crazyswarm_controller.launch`
Conversion from standard ROS messages to custom crazyswarm messages. Allows both position and velocity drones control.

- `crazyswarm_supervisor.launch`
VIsion-based relative localization emulation using OptiTrack. A topic is published for each robot, containing relative position of detected neighbors (undetected neighbors have a fake position of *(x,y) = (100.0, 100.0)* ).
It requires parameters indicating drone's field of view  range and angle.
<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_hydraulic_ctrl`
===================================
[![Build Status](https://github.com/107-systems/l3xz_hydraulic_ctrl/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_hydraulic_ctrl/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/l3xz_hydraulic_ctrl/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_hydraulic_ctrl/actions/workflows/spell-check.yml)

Hydraulic controller for the [L3X-Z](https://github.com/107-systems/l3xz) electric/hydraulic hexapod robot.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
##### Build via `colcon`
```bash
cd $COLCON_WS/src
git clone https://github.com/107-systems/l3xz_hydraulic_ctrl
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select l3xz_hydraulic_ctrl
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch l3xz_hydraulic_ctrl hydraulic_ctrl.py
```

#### Interface Documentation
##### Subscribed Topics
| Default name | Type |
|:-:|:-:|

##### Published Topics
|           Default name           |                                     Type                                     |
|:--------------------------------:|:----------------------------------------------------------------------------:|

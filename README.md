# CoppeliaSim Robot Simulation with ROS2 Teleoperation

![CoppeliaSim-ROS2 Integration](https://img.shields.io/badge/CoppeliaSim-ROS2-blue) 
![License](https://img.shields.io/badge/License-MIT-green)

This repository integrates a CoppeliaSim robotic scene with ROS2 for teleoperation. It includes:
- A CoppeliaSim scene (.ttt) with robot model
- Attached control script for command processing in development (Lua)
- ROS2 node for teleoperation control in development (Python)

## 📦 Repository Structure

repo_root/
├── coppelia/
│ ├── acmr5_snake_robot.ttt # Main simulation scene
│ └── scripts/ # Simulation scripts
└── ros2_teleop/
    └── src/
        └── coppelia_sim_ros2/ # ROS2 package
            └── coppelia_sim_ros2/ # ROS2 package nodes
                └── acmr5_controller.py # Teleoperation node
            ├── launch #
            ├── package.xml # ROS2 package definition
            ├── setup.py # Setup file

## 🚀 Quick Start

### Prerequisites
- [CoppeliaSim EDU](https://www.coppeliarobotics.com/downloads) (≥4.5.0)
- ROS2 (Humble/Iron)
- Python 3.8+

### CoppeliaSim Installation

Consult the CoppeliaSim website to download the simulator: https://www.coppeliarobotics.com/

### Build ros2_teleop ROS2 package

To build the package with **colcon build**, set the environment variable in your ~/.bashrc file that points to the path of your CoppeliaSim application **COPPELIASIM_ROOT_DIR**. Consider the following example:

``` 
{ 
    export COPPELIA_ROOT_DIR="~/path/to/coppeliaSim/folder"
    ulimit -s unlimited # Otherwise compilation might freeze / crash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 
}
```

The simROS2 package requires the **xmlschema** Python package used for validating XML files during the build process. To resolve this issue, install the package:

``` pip install xmlschema ```

Also install the following two packages ZeroMQ (ZMQ) and cbor (Concise Binary Objrect Representation) which are used for asynchronous messaging between processes (IPC) over networks (TCP) and efficient message encoding/decoding. This allows us to create and run Python scripts in CoppeliaSim.

``` pip install zmq cbor ```

Consult the following ROS2 tutorial in the CoppeliaSim website for further details:
https://manual.coppeliarobotics.com/en/ros2Tutorial.htm

### Submodule Management

This ROS2 workspace depends on the **'simROS2'** and **'ros2_bubble_rob'** packages loaded as submodules, go to the repository root directory and run the following command:

``` git submodule update --init --recursive ```

After that, build the **'mobile_land_robots'** workspace

``` colcon build ```

Set environment variables with install/setup.bash file

``` source install/setup.bash ```

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/MSR325/MSRacmr5_robot_teleoperation.git
   cd repo_name
2. Launch CoppeliaSim:
    ```bash
    coppeliaSim.sh coppelia/acmr5_snake_robot.ttt
3. Run the ROS2 node
    ```bash
    cd ros2_teleop
    colcon build && source install/setup.bash
    ros2 run coppelia_sim_ros2 acmr5_controller
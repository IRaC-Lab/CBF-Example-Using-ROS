# UR3 Trajectory using MATLAB (Ver ROS Noetic)

## Packages

### UR3 Simulation (Noetic Devel)
- **Description**: ROS packages for controlling the Universal Robots UR3 robotic arm.
- **Source**: [ur3](https://github.com/ros-industrial/universal_robot/tree/noetic-devel)

## Setting up ROS workspace

1. **Clone the repository**:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/IRaC-Lab/CBF-Example-Using-MATLAB.git
    ```

## UR3 Package Install

1. **Building from Source**:

    ```bash
    cd ~/catkin_ws
    
    # checking dependencies
    rosdep update
    rosdep install --rosdistro noetic --ignore-src --from-paths src
    ```

2. **Build the packages**

    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
    
## MATLAB Environment

- Matlab 2024a
- ROS Toolbox
- Robotics System Toolbox Support Package for Universal Robots UR Series Manipulators

## Running Gazebo and Executing CBF filter

```bash
roscore
roslaunch ur_gazebo ur3_pos_bringup.launch
rosrun ur_gazebo cbf_filter.py
```

## Running MATLAB for UR3 Trajectory

Run cbf_joint.mlx or cbf_task.mlx in MATLAB, and make sure to modify the ROS_IP and MASTER_URI according to your environment.

## Running CBF filter in ROS

```bash
roscore
roslaunch ur_gazebo ur3_pos_bringup.launch
rosrun ur_gazebo cbf_trajectory_test.py
```

## License

Package within this repository is distributed under its respective license. Please refer to each package's original repository for specific licensing information.

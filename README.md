# ROS 2 Turtlesim Catch Turtle

 Turtlesim-Catch-Turtle: A ROS 2 project designed to navigate a turtle in the turtlesim simulator using a P controller. Aims to drive the master turtle to catch other turtles with specified coordinates as they spawn with regular time intervals. Check https://www.udemy.com/course/ros2-for-beginners/ for futher information.

## Features

- Spawns turtles with regular time intervals.
- Retrieves the current pose of the master turtle and other spawned turtles.
- Computes the error in distance and heading to the desired position of the target spawned turtle.
- Uses a simple P controller to compute the required velocities.
- Publishes these velocities to command the master turtle.
- When the master turtle is sufficiently close to the target turtle, calls the catch turtle service to delete the target turtle. Then, specifies the new target based on the spawn order if there is any.

## Prerequisites

- ROS 2 (Humble,Foxy, Galactic, or later versions recommended).
- `turtlesim` package.

## Usage

1. Ensure you have ROS 2 and `turtlesim` installed.

2. Clone this repository:

```bash
git clone https://github.com/berkiskender/ros2_turtlesim_catch_turtle.git
cd ros2_turtlesim_catch_turtle 
```

3. Source your ROS 2 installation:

```bash
source /opt/ros/[YOUR_ROS2_DISTRO]/setup.bash
```

4. Build the packages:

```bash
colcon build --packages-select my_robot_interfaces
colcon build --packages-select my_turtlesim_project
colcon build --packages-select my_turtlesim_project_bringup
```

```bash
source install/setup.bash
colcon build
```

5. Source the built package:

```bash
source install/setup.bash
```

6. Run the `my_turtlesim_project` launcher:

```bash
ros2 launch my_turtlesim_project_bringup my_turtlesim_project.launch.py 
```

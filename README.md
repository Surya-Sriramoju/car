# car

## Instructions

### Building the package
- Copy the package 'car' into the src folder of ros workspace
- Enter into the workspace directory and build the package using "colcon build --packages-select car"

### Launching gazebo
- Open another terminal execute the following "ros2 launch car gazebo.launch.py"

### Launching RViz
- Open another terminal execute the following "ros2 launch car display.launch.py"

### Launching Gazebo and Rviz together
- Open another terminal execute the following "ros2 launch car debug.launch.py"

### Launching teleop control
- Open another terminal execute the following "ros2 launch car teleop_template.py"

### Launching pub-sub proportional controller
- Open another terminal execute the following "ros2 run car proportional_control.py"


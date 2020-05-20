# labyrinth_turtlebot

## Requirements

- turtlebot3
- turtlebot3_msgs
- turtlebot3_simulations
- gazebo
- rviz

rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 3
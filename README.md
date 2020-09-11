# uv_robot_pkg

This is the updated version and package of the UV Robot Fluoresce designed for the Hackster.io and Micron UV Robot Competition. </br>
</br>
- `cd robot_ws/src`
-  Clone this repo here : `git clone "https://github.com/YugAjmera/omni3ros_pkg"`
- `cd ..` (Go back to catkin_ws/)
- `catkin_make`
- `source devel/setup.bash`

View the robot in Gazebo or RVIZ </br>
- `roslaunch uv_robot_description robot_gazebo.launch`
- `roslaunch uv_robot_description robot_rviz.launch`
</br>
v1.0 : Updated the model, created transmission and controllable joints to adjust tubes height and angle. Obstacle Detection, Wall Follower, Laser Scanner.

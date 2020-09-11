# uv_robot_pkg

This is the updated version and package of the UV Robot Fluoresce designed for the Hackster.io and Micron UV Robot Competition. </br>
</br>
Install the package </br>
mkdir -p ~/robot_ws/src && cd ~/robot_ws/src </br>
git clone ""  </br>
cd .. </br>
catkin_make </br>
</br>
View the robot in Gazebo or RVIZ </br>
roslaunch uv_robot_description robot_gazebo.launch </br>
roslaunch uv_robot_description robot_rviz.launch </br>
v1.0 : Updated the model, created transmission and controllable joints to adjust tubes height and angle. Obstacle Detection, Wall Follower, Laser Scanner.

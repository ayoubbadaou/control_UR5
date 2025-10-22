start ros:

  source /opt/ros/humble/setup.bash
  source install/setup.bash  




Launch driver:

  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=10.2.30.60 launch_rviz:=false

launch ur moveit config:

  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 robot_ip:=10.2.30.60 launch_rviz:=true

 
 
 

ur5_control contruction du package:

  colcon build --symlink-install --packages-select ur5_controller
  source install/setup.bash 



run ur5_control nodes

  ros2 run ur5_controller self_collision 
  ros2 run ur5_controller ur5_controller 
  ros2 run ur5_controller ros_control_ur5
  ros2 run ur5_controller auto_calib


Gestion des controleurs :

ros2 control list_controllers

Activer le controleur

  ros2 control set_controller_state scaled_joint_trajectory_controller active
  ros2 control set_controller_state joint_trajectory_controller active

Desactiver le controleur:

  ros2 control set_controller_state scaled_joint_trajectory_controller inactive
  ros2 control set_controller_state joint_trajectory_controller inactive



**workspace Build's method to avoid pc's crashing:**


sudo fallocate -l 4G /tmp/swapfile
sudo chmod 600 /tmp/swapfile
sudo mkswap /tmp/swapfile
sudo swapon /tmp/swapfile

source /opt/ros/humble/setup.bash
colcon build --symlink-install --mixin release --executor sequential

sudo swapoff /tmp/swapfile
sudo rm /tmp/swapfile



monitoriser sur un autre terminale:
watch -n 1 free -h



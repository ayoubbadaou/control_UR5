**Etape pour utilser l'espace detravail ROS correctement**


**1-  Il faut d'abord télécharger les packages moveit2 et ur ros driver sur l'espace de travail**


**Conseil avant de faire build dans l'espace de travail:**
il faut d'abord parametrer d'abord le swapfile en ajoutant un espace de stockage supplimentaire pour eviter le crash du pc, car les packages de moveit sont lourd et demande plus d'espace dans le processeur

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


**2-  initialiser ros et l'espace de travail:**

lancez cette commande à chaque fois vous ouvrez un nouveau terminale

    source /opt/ros/humble/setup.bash
    source install/setup.bash

ur5_control contruction du package:

    colcon build --symlink-install --packages-select ur5_controller
    source install/setup.bash


 **3-  Dans un premier terminale lancez le driver:**

    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=10.2.30.60 launch_rviz:=false

**4-  Dans un deuxièm terminale lancer ur moveit config:**

    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 robot_ip:=10.2.30.60 launch_rviz:=true




**6-  lancez et testez le noeud self_collision pour la planification de trajectoire**

    ros2 run ur5_controller self_collision 
    ros2 run ur5_controller auto_calib

**5-  Gestion des controleurs :**

    ros2 control list_controllers

-  Activer le controleur

lancer une des deux commande pour activer l'un des controleur ROS

    ros2 control set_controller_state scaled_joint_trajectory_controller active
    ros2 control set_controller_state joint_trajectory_controller active

-  Désactiver le controleur:

lancer une des deux commande pour desacitver lun des deux controleur

    ros2 control set_controller_state scaled_joint_trajectory_controller inactive
    ros2 control set_controller_state joint_trajectory_controller inactive

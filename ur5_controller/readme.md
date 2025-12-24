**Etapes pour connecter le robot avec el controleur ROS**

Avant de créer le package ROS il faut configurer le Robot UR5 afin d'assurer la connectivité avec votre serveur d'où vous controlerez le robot par le controleur ROS.

Pour cela il faut d'abord configurer les parametre control externe (**External control**) dans l'onglet **Installation**, pour qu'il soit connecté à la bonne adresse IP et le bon port du controleur ROS. Comme montré dans l'iamge ci-dessous

<img width="666" height="442" alt="image" src="https://github.com/user-attachments/assets/51484671-bc66-47cf-9d30-36166cba6a22" />

Dans le package ROS Driver le port 50002 est défini par defaut;
<img width="808" height="82" alt="image" src="https://github.com/user-attachments/assets/0faac828-3f15-4997-86a9-024dd0642313" />


Dans l'onglet entrée sortie (**E/S**) il faut s'assurer aussi que le robot est bien controlé par utilisateur comme montré dans l'image ci-dessous. si le une autre option est sélectionné modifiez la et mettez le choix utilisateur. 

![USer](https://github.com/user-attachments/assets/b42258b2-303b-464e-8cdc-26ba39333925)

Finalement pour que le robot executer le mouvement il faut lancer le programme nommé ROS comme montré dans l'image suivante :

<img width="640" height="391" alt="image" src="https://github.com/user-attachments/assets/122c1101-3f23-4aaa-b06f-f03b56884d36" />




**Etapes pour utilser l'espace de travail ROS correctement**


**1-  Il faut d'abord télécharger les packages moveit2 et ur ros driver sur l'espace de travail**


**Conseil avant de faire build dans l'espace de travail:**
il faut d'abord parametrer le swapfile en ajoutant un espace de stockage supplimentaire avant de construire le package pour eviter le crash du pc, car les packages de moveit sont lourd et demande plus d'espace dans le processeur

    sudo fallocate -l 4G /tmp/swapfile
    sudo chmod 600 /tmp/swapfile
    sudo mkswap /tmp/swapfile
    sudo swapon /tmp/swapfile

ensuite avec ces deux commande on lance la contruction des packages

    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --mixin release --executor sequential

    sudo swapoff /tmp/swapfile
    sudo rm /tmp/swapfile

Pour monitoriser l'utilisation de l'espace du processeur lancer cette commande ci-dessous sur un autre terminale:

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

lancer une des deux commande pour activer l'un des deux controleur ROS

    ros2 control set_controller_state scaled_joint_trajectory_controller active
    ros2 control set_controller_state joint_trajectory_controller active

-  Désactiver le controleur:

lancer une des deux commande pour desacitver l'un des deux controleur

    ros2 control set_controller_state scaled_joint_trajectory_controller inactive
    ros2 control set_controller_state joint_trajectory_controller inactive

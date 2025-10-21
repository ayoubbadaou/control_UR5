**Commennt faire la calibration Hand-eye**

Notre methode de calibration se fait en 2 etapes:
Première etapes : collecter des données oùondoit collecter les coordonées du tableau Charuco dans le repère de la camera et les positions de l'outil dans la base du robot (pose TCP) dans le fichier eyetohand_data.yaml
Deuxième etapes : calcule de la matrice de transfert entre le repère de la camera et le repère du robot à pratir des données collectées et l'enregistrer dans le fichier TF_matrixs.yaml

-Avant de commencé la calibratin il faut bien configurer les paramètre intrinsèque et les parametres Charuco dans le fichier intrinsics.yaml

    camera_matrix:
    - - 609.0755095503881
      - 0.0
      - 333.78002935525393
    - - 0.0
      - 608.6477356099793
      - 254.14077041164182
    - - 0.0
      - 0.0
      - 1.0
    dist_coeffs:
    - - 0.0
      - -0.0
      - -0.0
      - -0.0
      - 0.0
    charuco_params:
      board_squares_x: 7
      board_squares_y: 5
      square_length: 0.0215
      marker_length: 0.016
      aruco_dict: DICT_4X4_50

-Pour cela on utilisé le code prisePhoto.py pour sauvegarder les images sur un dossier nommé image ainsi in lance le code pick_and_place_system.py afin de calibrer les parametres intrinsèques de la camera et copier les valeurs sur le fichier intrinsics.yaml en avec les parametre charuco.
pour les parametre charuco il faut bien definir le nombre des ligne et des colonnes dans le tableau charuco, la taille des marqueurs et des carreaux, et le dicitionaire des aruco utilsé.

-Ensuite cela on lance le code calibration_eye_hand_data.py qui permet la collectte des données:
    
    python3 calibration_eye_hand_data.py
    
-Après le lancement du code une fenetre sera ouvert visualisant le contnue de la camera rgb et les donées de camera et robot en temps réel.

<img width="1185" height="957" alt="Screenshot from 2025-10-21 10-23-51" src="https://github.com/user-attachments/assets/23fd3753-ac09-49e3-b48b-58429e93eaf4" />

-Ensuite ON appuie sur la bouton C pour capturer les données : "Target in Cam" (pour la caméra) et "TCP" et "Rot" pour le robot.
-On bouge le Tableau Charuco, en le gardant dans le champs de vision de la camèra, afin d'avoir une position différente puis on appuie sur c pour capturer les nouvelles données.
-On refait la meme chose  plusieurs fois (au moins 20 capture) et enfin on Appuie sur s pour enregistrer les données capturées dans le fichier eyetohand_data.yaml

-Ensuite dans la deuxième etape on lance le code do_hand_eye_calib.py pour calculer la matrice de transfomation entre le repère de la caméra et le repère du robot

    python3 do_hand_eye_calib.py

-La matrice sera enregistré ainsi dans le fichier TF_matrixs.yaml

Enfin pour tester la calibration le code detect_marker.py permet de detecter le tableau Charuco dans le repère de la caméra et calcule sa position dans le repère du robot à l'aide de la matrice de transfert calculée

    python3 detect_marker.py

Le code va visualiser le tableau et sa position dans le repère du Robot calculée par la matrice de transfert

<img width="1335" height="1055" alt="Screenshot from 2025-10-21 11-11-44" src="https://github.com/user-attachments/assets/9bf0828b-3836-476b-96e9-8c77a3ed60aa" />

La première ligne en jaune montre les coordonnées de tableau charuco sur le repère de la caméra.

la deuxième et la troisième ligne en bleu ciel montre les coordonées du tableau charuco dans le repère du robot calculés par la matrice de transformation

la dernière ligne en rouge montre les x y z réel du robot

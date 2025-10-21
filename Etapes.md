Commennt faire la calibration Hand-eye

Notre methode de calibration se fait en 2 etapes:
Première etapes : collecter des données oùondoit collecter les coordonées du tableau Charuco dans le repère de la camera et les positions de l'outil dans la base du robot (pose TCP) dans le fichier eyetohand_data.yaml
Deuxième etapes : calcule de la matrice de transformation entre le repère de la camera et le repère du robot à pratir des données collectées et l'enregistrer dans le fichier TF_matrixs.yaml

-Pour cela on lance le code calibration_eye_hand_data.py qui permet la collectte des données:

python3 calibration_eye_hand_data.py 

-Après le lancement du code une fenetre sera ouvert visualisant le contnue de la camera rgb et les donées de camera et robot en temps réel.

<img width="1185" height="957" alt="Screenshot from 2025-10-21 10-23-51" src="https://github.com/user-attachments/assets/23fd3753-ac09-49e3-b48b-58429e93eaf4" />

-Ensuite ON appuie sur la bouton C pour capturer les données : "Target in Cam" (pour la caméra) et "TCP" et "Rot" pour le robot.
-On bouge le Tableau Charuco, en le gardant dans le champs de vision de la camèra, afin d'avoir une position différente puis on appuie sur c pour capturer les nouvelles données.
-On refait la meme chose  plusieurs fois (au moins 20 capture) et enfin on Appuie sur s pour enregistrer les données capturées dans le fichier eyetohand_data.yaml

-Ensuite dans la deuxième etape on lance le code do_hand_eye_calib.py pour calculer la matrice de transfomation entre le repère de la caméra et le repère du robot

python3 do_hand_eye_calib.py

-La matrice sera enregistré ainsi dans le fichier TF_matrixs.yaml

Enfin pour tester la calibration le code detect_marker.py permet de detecter le tableau Charuco dans le repère de la caméra et calcule sa position dans le repère du robot à l'aide de la matrice de transformation calculée

python3 detect_marker.py

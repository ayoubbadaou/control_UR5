**Comment publier les données de la base de données dans le réseau avec OPC Ua**
La publication de la base des données se fait sous 3 etapes:

-  Publication des données en temps réels
-  Enregistrement des données dans la base de données avec Node red
-  publication de la base des données dans le reseau à travers d'un serveur OPC UA

Et enfin pour verifier la publication des données on utilise un client OPC UA pour lire et afficher les données

**1-  Publication des données en temps réel**

Dans le dossier Data_run j'ai crée 3 codes qui publient trois type de données: coordonnées d'un tableau Charuco dans le repère camera, coordonnée de l'outil du robot dans le repère du robot (TCP pose) et les données des capteurs de position pour detecter l'arriver d'un produit dans le convoyeur et l'existance du produit sur le bac. pour lancer les 3 code, il suffit de lancer le code Data_run.py qui prend le role d'un fichier main lanceur en lançant les 3 code simultanémnt 

    python3 data_run.py

les données seront ainsi publiées et affichées sur le terminale

**2-  Enregistrement des données dans la base de données avec Node red**

Ainsi à l'aide des flux node red défini dans le code txt vous pouvez importer les flux en copiant le code sur la case d'importage de Node red et en deployant les données seront ainsi enregistré dans le fichier base de donnée vous pouvez aussi changer le derictoire du fichier selon votre besoin. 
**(faisez attention aux chemin des fichiers et des code ça peut varié dans chaque pc il faut donc absolument les adapter)**

**3-  publication de la base des données dans le reseau à travers d'un serveur OPC UA**

Ensuite dans le dossier Db_pub_read contient les code qui publient les données enregistré (les code qui finissent par "_pub.py" ) et pour lancer tous le code j'utilse de meme le code lanceur db_pub.py qui lance tous les code simultanément

    python3 db_pub.py

Finalement pour vérifier la publication j'utilise le group des code finissant par "_read.py" qui sont des clients OPC UA et qui vont lire les tableau publiés et les afficher sur une terminale. Pour lancer le code il suffit de lancer le code db_read.py:

    python3 db_read.py

**Comment adapter le flux avec données reçu?**

Prenons l'exemple de la première camera et voyons quels sont les parametres à configurer pour une bonne aquisition des donneés.
Dans le code deep_detect.py nous avons défini deux variables vont jouer un role important pour partager les données avec le flux node red. 

<img width="508" height="24" alt="image" src="https://github.com/user-attachments/assets/a4355906-4e41-4ffe-9239-e99cc2238d53" />

camera_index et cam_id. La camera realsense a 6 cannaux chacun porte un flux (de la canale 0 à la canale 5), le premier canal porte le fulx principale de l'image RGB, sa valeur dans notre cas la canale 0 est ce qui nous intérésse. si plusieurs camera sont connecté le flux principale de chacun sera le multiple de 6, soit 0 , 6, 12, etc. Ensuite cam_id va definir le port surlequel on va envoyé nos données à node red via OPC UA. Nous avons utilisé ainsi la valeur 0 du cam_id dans la 32 du code en indiquant alors le port 2000. ensuite on defini les node id de chaque variable qu'on partagera dans ce port. dans notre exemple nous avons définie 6 variable qui x, y, z, rx, ry, rz on dédinira ainsi 6 node id pour chacun

<img width="912" height="165" alt="image" src="https://github.com/user-attachments/assets/d710aeee-8310-4b6f-9520-e70210d2edb9" />

la forme de node id sera donc 0Marker1 ou bien 0Marker2 puisque le numero de marqueur aruco défini sera 1 ou 2 (voir la ligne 46 du code). selon votre besoin vous pouvez définir plusieur marqueur.

Allons maintenant à l'interface node red:

<img width="962" height="420" alt="image" src="https://github.com/user-attachments/assets/13af0951-9770-4a48-a77a-52d68e6b00a5" />

on définit dans le premier les node id et le mode d'aquisition des données (soit une seule soit d'une manière répétitive) par exemple le premier on défini le node id de la variable x:

<img width="486" height="617" alt="image" src="https://github.com/user-attachments/assets/a88f2156-ac7f-45f9-bfd1-06a0f7c61f24" />

dans le deuxieme noeud on definit le port OPC UA :

<img width="421" height="620" alt="image" src="https://github.com/user-attachments/assets/287ffdc8-fc49-421f-9426-fd7582af6d02" />

ensuite à partir de ces données on va générer la base de données sql à l'aide du noeud où nous avons mis notre fonction

<img width="879" height="617" alt="image" src="https://github.com/user-attachments/assets/2dd2ef1c-199c-48c8-b954-b0c38d4f4cfc" />

dans la fonction nous avons pris les variable stoqués sur msg.topic pour génerer le tableau à base sqlite ensuite le dernier noeud data va enregister le tableau sur le fichier .db:

<img width="416" height="619" alt="image" src="https://github.com/user-attachments/assets/48d8d989-76c8-4805-8069-83d8e1f00d9f" />

les autres flux suivent les memes instructions






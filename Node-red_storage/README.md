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



from opcua import Client
import time
from datetime import datetime

# ==================== CONFIGURATION ====================
OPC_ENDPOINT = "opc.tcp://localhost:3008/cam1_data/"
NAMESPACE_URI = "UR5.CAM1Data"
POLL_INTERVAL = 0.3  # Plus rapide que le polling du serveur
# ======================================================

class Cam1TableMonitor:
    def __init__(self):
        self.client = Client(OPC_ENDPOINT)
        self.last_row_count = 0

    def connect(self):
        self.client.connect()
        print(f"Connecté à {OPC_ENDPOINT}")

        # Obtenir l'indice de l'espace de noms
        self.ns_idx = self.client.get_namespace_index(NAMESPACE_URI)
        if self.ns_idx == -1:
            raise Exception(f"L'espace de noms '{NAMESPACE_URI}' n'a pas été trouvé")

        # Obtenir la référence de la table
        self.table_node = self.client.get_objects_node().get_child([
            f"{self.ns_idx}:CAM1PoseTable"
        ])

        # Obtenir toutes les références de colonnes
        self.columns = {
            "id": self.table_node.get_child([f"{self.ns_idx}:RowIDs"]),
            "timestamp": self.table_node.get_child([f"{self.ns_idx}:Timestamps"]),
            "x": self.table_node.get_child([f"{self.ns_idx}:X"]),
            "y": self.table_node.get_child([f"{self.ns_idx}:Y"]),
            "z": self.table_node.get_child([f"{self.ns_idx}:Z"]),
            "rx": self.table_node.get_child([f"{self.ns_idx}:Rx"]),
            "ry": self.table_node.get_child([f"{self.ns_idx}:Ry"]),
            "rz": self.table_node.get_child([f"{self.ns_idx}:Rz"])
        }

        # Chargement initial des données
        self.display_full_table()

    def display_full_table(self):
        """Afficher la table complète actuelle"""
        data = {name: var.get_value() for name, var in self.columns.items()}
        print("\nTable des poses CAM1 actuelle:")
        self._print_table(data)

    def _print_table(self, data, highlight_rows=None):
        """Aide pour imprimer les données de la table"""
        headers = " | ".join([f"{col:<10}" for col in data.keys()])
        print(headers)
        print("-" * len(headers))

        for i in range(len(data["id"])):
            row = " | ".join([
                f"{str(data[col][i]):<10}"[:10] for col in data.keys()
            ])
            # Surligner les nouvelles lignes si spécifié
            if highlight_rows and i in highlight_rows:
                print(f"\033[92m{row}\033[0m")  # Couleur verte
            else:
                print(row)

    def monitor(self):
        try:
            print("\nSurveillance de nouvelles données... (Ctrl+C pour arrêter)")
            while True:
                current_count = len(self.columns["id"].get_value())

                if current_count > self.last_row_count:
                    new_rows = range(self.last_row_count, current_count)
                    data = {name: var.get_value() for name, var in self.columns.items()}

                    print(f"\n{datetime.now().strftime('%H:%M:%S')} - {len(new_rows)} nouvelle(s) ligne(s) ajoutée(s) à la table des poses CAM1:")
                    self._print_table(data, highlight_rows=new_rows)

                self.last_row_count = current_count
                time.sleep(POLL_INTERVAL)

        except KeyboardInterrupt:
            print("\nSurveillance arrêtée")
        finally:
            self.client.disconnect()
    def read(self):
        self.connect()
        self.monitor()

if __name__ == "__main__":
    monitor = Cam1TableMonitor()
    monitor.read()

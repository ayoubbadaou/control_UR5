from opcua import Client
import time
from datetime import datetime

# ==================== CONFIGURATION ====================
OPC_ENDPOINT = "opc.tcp://localhost:3016/sen_data/"
NAMESPACE_URI = "UR5.SENData"
POLL_INTERVAL = 0.3  # Faster than server polling
# ======================================================

class SenTableMonitor:
    def __init__(self):
        self.client = Client(OPC_ENDPOINT)
        self.last_row_count = 0
        
    def connect(self):
        self.client.connect()
        print(f"Connected to {OPC_ENDPOINT}")
        
        # Get namespace index
        self.ns_idx = self.client.get_namespace_index(NAMESPACE_URI)
        if self.ns_idx == -1:
            raise Exception(f"Namespace '{NAMESPACE_URI}' not found")
            
        # Get table reference
        self.table_node = self.client.get_objects_node().get_child([
            f"{self.ns_idx}:SENPoseTable"
        ])
        
        # Get all column references
        self.columns = {
            "id": self.table_node.get_child([f"{self.ns_idx}:RowIDs"]),
            "timestamp": self.table_node.get_child([f"{self.ns_idx}:Timestamps"]),
            "conv1": self.table_node.get_child([f"{self.ns_idx}:CONV1"]),
            "conv2": self.table_node.get_child([f"{self.ns_idx}:CONV2"]),
            "bac1": self.table_node.get_child([f"{self.ns_idx}:BAC1"]),
            "bac2": self.table_node.get_child([f"{self.ns_idx}:BAC2"]),
            "bac3": self.table_node.get_child([f"{self.ns_idx}:BAC3"]),
            "bac4": self.table_node.get_child([f"{self.ns_idx}:BAC4"]),
            "bac5": self.table_node.get_child([f"{self.ns_idx}:BAC5"])
        }
        
        # Initial data load
        self.display_full_table()
        
    def display_full_table(self):
        """Show complete current table"""
        data = {name: var.get_value() for name, var in self.columns.items()}
        print("\nCurrent SEN Pose Table:")
        self._print_table(data)
        
    def _print_table(self, data, highlight_rows=None):
        """Helper to print table data"""
        headers = " | ".join([f"{col:<10}" for col in data.keys()])
        print(headers)
        print("-" * len(headers))
        
        for i in range(len(data["id"])):
            row = " | ".join([
                f"{str(data[col][i]):<10}"[:10] for col in data.keys()
            ])
            # Highlight new rows if specified
            if highlight_rows and i in highlight_rows:
                print(f"\033[92m{row}\033[0m")  # Green color
            else:
                print(row)
    
    def monitor(self):
        try:
            print("\nMonitoring for new data... (Ctrl+C to stop)")
            while True:
                current_count = len(self.columns["id"].get_value())
                
                if current_count > self.last_row_count:
                    new_rows = range(self.last_row_count, current_count)
                    data = {name: var.get_value() for name, var in self.columns.items()}
                    
                    print(f"\n{datetime.now().strftime('%H:%M:%S')} - New {len(new_rows)} row(s) added to sensors table:")
                    self._print_table(data, highlight_rows=new_rows)
                    
                self.last_row_count = current_count
                time.sleep(POLL_INTERVAL)
                
        except KeyboardInterrupt:
            print("\nMonitoring stopped")
        finally:
            self.client.disconnect()
    def read(self):
        self.connect()
        self.monitor()
        
if __name__ == "__main__":
    monitor = SenTableMonitor()
    monitor.read()

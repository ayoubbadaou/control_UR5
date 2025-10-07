from opcua import Client
import time
from datetime import datetime

# ==================== CONFIGURATION ====================
OPC_ENDPOINT = "opc.tcp://localhost:3010/cam_data/"
NAMESPACE_URI = "UR5.CAMData"
POLL_INTERVAL = 0.3  # Faster than server polling
# ======================================================

class TableMonitor:
    def __init__(self,OPC_ENDPOINT = "opc.tcp://localhost:3010/cam_data/",NAMESPACE_URI = "UR5.CAMData"):
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
            f"{self.ns_idx}:CAMPoseTable"
        ])
        
        # Get all column references
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
        
        # Initial data load
        self.display_full_table()
        
    def display_full_table(self):
        """Show complete current table"""
        data = {name: var.get_value() for name, var in self.columns.items()}
        print("\nCurrent CAM Pose Table:")
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
                    
                    print(f"\n{datetime.now().strftime('%H:%M:%S')} - New {len(new_rows)} row(s) added to CAM pose table:")
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
    monitor = TableMonitor()
    monitor.read()

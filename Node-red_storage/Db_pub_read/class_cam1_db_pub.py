import sqlite3
from opcua import Server, ua
import time

class Cam1DbToOPCUA:
    def __init__(self):
        self.db_path = "/home/robot/Bureau/Node-red_storage/mydata.db"
        self.opc_endpoint = "opc.tcp://0.0.0.0:3008/cam1_data/"
        self.namespace_uri = "UR5.CAM1Data"
        self.table_name = "camera_pose1"
        self.poll_interval = 0.5
        
        self.server = None
        self.conn = None
        self.cursor = None
        self.column_nodes = {}
        
    def initialize_opcua_server(self):
        """Initialize OPC UA server and create nodes"""
        self.server = Server()
        self.server.set_endpoint(self.opc_endpoint)
        idx = self.server.register_namespace(self.namespace_uri)
        objects = self.server.get_objects_node()
        
        # Create OPC UA nodes structure
        table_node = objects.add_object(idx, "CAM1PoseTable")
        self.last_row_node = table_node.add_variable(idx, "LastRowID", 0, varianttype=ua.VariantType.Int32)
        self.new_data_node = table_node.add_variable(idx, "NewDataAvailable", False, varianttype=ua.VariantType.Boolean)
        
        # Initialize OPC UA variables for all columns
        self.column_nodes = {
            "id": table_node.add_variable(idx, "RowIDs", [], varianttype=ua.VariantType.Int32),
            "timestamp": table_node.add_variable(idx, "Timestamps", [], varianttype=ua.VariantType.String),
            "x": table_node.add_variable(idx, "X", [], varianttype=ua.VariantType.Double),
            "y": table_node.add_variable(idx, "Y", [], varianttype=ua.VariantType.Double),
            "z": table_node.add_variable(idx, "Z", [], varianttype=ua.VariantType.Double),
            "rx": table_node.add_variable(idx, "Rx", [], varianttype=ua.VariantType.Double),
            "ry": table_node.add_variable(idx, "Ry", [], varianttype=ua.VariantType.Double),
            "rz": table_node.add_variable(idx, "Rz", [], varianttype=ua.VariantType.Double)
        }
        
        self.server.start()
        print(f"OPC UA server started at {self.opc_endpoint}")
        
    def initialize_database_connection(self):
        """Initialize SQLite database connection"""
        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()
        print(f"Connected to database at {self.db_path}")
        
    def get_last_rowid(self):
        """Get the last row ID from the database"""
        self.cursor.execute(f"SELECT MAX(id) FROM {self.table_name}")
        return self.cursor.fetchone()[0] or 0
        
    def update_opcua_with_new_data(self):
        """Check for new data and update OPC UA nodes"""
        current_last_id = self.get_last_rowid()
        if current_last_id > self.last_row_node.get_value():
            # New data detected
            self.cursor.execute(
                f"SELECT * FROM {self.table_name} WHERE id > ?", 
                (self.last_row_node.get_value(),)
            )
            new_rows = self.cursor.fetchall()
            
            if new_rows:
                # Update OPC UA variables
                for row in new_rows:
                    self.column_nodes["id"].set_value(self.column_nodes["id"].get_value() + [row[0]])
                    self.column_nodes["timestamp"].set_value(self.column_nodes["timestamp"].get_value() + [str(row[1])])
                    self.column_nodes["x"].set_value(self.column_nodes["x"].get_value() + [float(row[2] or 0.0)])
                    self.column_nodes["y"].set_value(self.column_nodes["y"].get_value() + [float(row[3] or 0.0)])
                    self.column_nodes["z"].set_value(self.column_nodes["z"].get_value() + [float(row[4] or 0.0)])
                    self.column_nodes["rx"].set_value(self.column_nodes["rx"].get_value() + [float(row[5] or 0.0)])
                    self.column_nodes["ry"].set_value(self.column_nodes["ry"].get_value() + [float(row[6] or 0.0)])
                    self.column_nodes["rz"].set_value(self.column_nodes["rz"].get_value() + [float(row[7] or 0.0)])
                
                self.last_row_node.set_value(current_last_id)
                self.new_data_node.set_value(True)  # Notify clients
                self.new_data_node.set_value(False)  # Reset flag
                
                print(f"Added {len(new_rows)} new row(s) to OPC UA")
                
    def publish(self):
        """Main execution loop"""
        self.initialize_database_connection()
        self.initialize_opcua_server()
        
        try:
            while True:
                self.update_opcua_with_new_data()
                time.sleep(self.poll_interval)
                
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Clean up resources"""
        if self.conn:
            self.conn.close()
            print("Database connection closed")
        if self.server:
            self.server.stop()
            print("OPC UA server stopped")

def main():
    # Configuration
    
    
    # Create and run the bridge
    bridge = Cam1DbToOPCUA()
    bridge.publish()

if __name__ == "__main__":
    main()

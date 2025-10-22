from rtde_receive import RTDEReceiveInterface
from opcua import Server, ua
import time

class Robot:
    def __init__(self, robot_ip="10.2.30.60", opcua_endpoint="opc.tcp://0.0.0.0:4842/"):
        self.robot_ip = robot_ip
        self.opcua_endpoint = opcua_endpoint
        self.rtde_r = None
        self.server = None
        self.node_ids = {}
        
    def initialize_robot_connection(self):
        """Initialize connection to the UR5 robot"""
        self.rtde_r = RTDEReceiveInterface(self.robot_ip)
        print(f"✅ Connected to robot at {self.robot_ip}")
        
    def initialize_opcua_server(self):
        """Initialize OPC UA server and variables"""
        self.server = Server()
        self.server.set_endpoint(self.opcua_endpoint)
        
        namespace_index = self.server.register_namespace("UR5Data")
        objects_node = self.server.get_objects_node()
        
        # Create variables with forced NodeIds
        self.node_ids = {
            "TCP_X": objects_node.add_variable(ua.NodeId("TCP_X", namespace_index), "TCP_X", 0.0),
            "TCP_Y": objects_node.add_variable(ua.NodeId("TCP_Y", namespace_index), "TCP_Y", 0.0),
            "TCP_Z": objects_node.add_variable(ua.NodeId("TCP_Z", namespace_index), "TCP_Z", 0.0),
            "TCP_Rx": objects_node.add_variable(ua.NodeId("TCP_Rx", namespace_index), "TCP_Rx", 0.0),
            "TCP_Ry": objects_node.add_variable(ua.NodeId("TCP_Ry", namespace_index), "TCP_Ry", 0.0),
            "TCP_Rz": objects_node.add_variable(ua.NodeId("TCP_Rz", namespace_index), "TCP_Rz", 0.0),
        }
        
        # Make all variables writable
        for var in self.node_ids.values():
            var.set_writable()
            
        self.server.start()
        print(f"✅ OPC UA server started at {self.opcua_endpoint}")
        print("🟢 NodeIds available: ns=2;s=TCP_X ... TCP_Rz")
        
    def update_pose(self):
        """Read robot pose and update OPC UA variables"""
        try:
            while True:
                pose = self.rtde_r.getActualTCPPose()  # [x, y, z, rx, ry, rz]
                
                keys = ["TCP_X", "TCP_Y", "TCP_Z", "TCP_Rx", "TCP_Ry", "TCP_Rz"]
                for i, key in enumerate(keys):
                    self.node_ids[key].set_value(pose[i])
                
                print("🔁 Pose updated:", pose)
                time.sleep(0.5)
                
        except Exception as e:
            print("🛑 Stopping server...")
        finally:
            self.server.stop()
            print("Server stopped")
            
    def run(self):
        """Main function that runs all components"""
        self.initialize_robot_connection()
        self.initialize_opcua_server()
        self.update_pose()

def main():
    robot = Robot()
    robot.run()

if __name__ == "__main__":
    main()

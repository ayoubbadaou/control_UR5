import json
import threading
import time
from opcua import ua, Server
import paho.mqtt.client as mqtt


class MqttData:
    def __init__(self, broker="10.2.30.162", port=1883):
        self.broker = broker
        self.port = port
        self.topic_conv = "capteurs_convoyeur/etat"
        self.topic_bac = "capteurs_bac/etat"
        
        # Shared variables, default to 0
        self.conv1 = 0
        self.conv2 = 0
        self.bac1 = 0
        self.bac2 = 0
        self.bac3 = 0
        self.bac4 = 0
        self.bac5 = 0
        
        self.lock = threading.Lock()

    def start(self):
        def on_connect(client, userdata, flags, rc):
            print("MQTT connecté avec le code:", rc)
            client.subscribe(self.topic_conv)
            client.subscribe(self.topic_bac)

        def on_message(client, userdata, msg):
            try:
                data = json.loads(msg.payload.decode("utf-8"))
                with self.lock:
                    if msg.topic == self.topic_conv:
                        #self.conv1 = data.get("pin1", 0) or 0
                        #self.conv2 = data.get("pin2", 0) or 0
                        self.conv1 = int(data.get("pin1", 0) or 0)
                        self.conv2 = int(data.get("pin2", 0) or 0)

                    elif msg.topic == self.topic_bac:
                        #self.bac1 = data.get("pin1", 0) or 0
                        #self.bac2 = data.get("pin2", 0) or 0
                        #self.bac3 = data.get("pin3", 0) or 0
                        #self.bac4 = data.get("pin4", 0) or 0
                        #self.bac5 = data.get("pin5", 0) or 0
                        self.bac1 = int(data.get("pin1", 0) or 0)
                        self.bac2 = int(data.get("pin2", 0) or 0)
                        self.bac3 = int(data.get("pin3", 0) or 0)
                        self.bac4 = int(data.get("pin4", 0) or 0)
                        self.bac5 = int(data.get("pin5", 0) or 0)

                print("MQTT Data:", self.get_data())
            except Exception as e:
                print("Erreur MQTT:", e)

        client = mqtt.Client("MqttReceiver")
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(self.broker, self.port, 60)

        mqtt_thread = threading.Thread(target=client.loop_forever, daemon=True)
        mqtt_thread.start()

    def get_data(self):
        with self.lock:
            return {
                "conv1": self.conv1,
                "conv2": self.conv2,
                "bac1": self.bac1,
                "bac2": self.bac2,
                "bac3": self.bac3,
                "bac4": self.bac4,
                "bac5": self.bac5,
            }


class OpcUaServer:
    def __init__(self, mqtt_data: MqttData):
        self.mqtt_data = mqtt_data
        self.server = Server()
        self.server.set_endpoint("opc.tcp://0.0.0.0:4846")
        self.namespace = self.server.register_namespace("MQTT_OPCUA")

        # Create variables
        self.objects = self.server.get_objects_node()
        self.vars = {
            name: self.objects.add_variable(
                ua.NodeId(name, self.namespace), name, 0
            )
            for name in ["conv1", "conv2", "bac1", "bac2", "bac3", "bac4", "bac5"]
        }

        for var in self.vars.values():
            var.set_writable()

    def start(self):
        self.server.start()
        print("OPC UA Server started at opc.tcp://0.0.0.0:4846")

        try:
            while True:
                data = self.mqtt_data.get_data()
                for key, value in data.items():
                    self.vars[key].set_value(value)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Arrêt du serveur OPC UA.")
        finally:
            self.server.stop()


if __name__ == "__main__":
    mqtt_data = MqttData()
    mqtt_data.start()

    time.sleep(1)  # ensure MQTT thread starts first

    opcua_server = OpcUaServer(mqtt_data)
    opcua_server.start()


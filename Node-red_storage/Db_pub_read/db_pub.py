from class_tcp_db_pub import TcpDbToOPCUA
from class_cam0_db_pub import Cam0DbToOPCUA
from class_cam1_db_pub import Cam1DbToOPCUA
from class_sen_db_pub import SenDbToOPCUA
import json
import paho.mqtt.client as mqtt
import threading
import time
from opcua import ua, Server


def main():
    
    cam0db = Cam0DbToOPCUA()
    cam1db = Cam1DbToOPCUA()
    tcpdb = TcpDbToOPCUA()
    sendb = SenDbToOPCUA()
    
    
    threads = [
        threading.Thread(target=cam0db.publish, daemon=True),
        threading.Thread(target=cam1db.publish, daemon=True),
        threading.Thread(target=tcpdb.publish, daemon=True),
        threading.Thread(target=sendb.publish, daemon=True)    
    ]
    
    # Start threads
    
    for t in threads:
        t.start()
      
    
    try:
        # Main thread can do other work or just wait
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Arrêt du programme.")
        # Since threads are daemon threads, they'll exit when main exits

if __name__ == "__main__":
    main()

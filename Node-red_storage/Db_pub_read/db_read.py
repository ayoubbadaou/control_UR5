from class_tcp_db_read import TcpTableMonitor
from class_cam0_db_read import Cam0TableMonitor
from class_cam1_db_read import Cam1TableMonitor
from class_sen_db_read import SenTableMonitor
import json
import paho.mqtt.client as mqtt
import threading
import time
from opcua import ua, Server


def main():
    
    cam0db = Cam0TableMonitor()
    cam1db = Cam1TableMonitor()
    tcpdb = TcpTableMonitor()
    sendb = SenTableMonitor()
    
    
    threads = [
        threading.Thread(target=cam0db.read, daemon=True),
        threading.Thread(target=cam1db.read, daemon=True),
        threading.Thread(target=tcpdb.read, daemon=True),
        threading.Thread(target=sendb.read, daemon=True)   
    ]
    
    time.sleep(0.5)
    
    for t in threads :
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

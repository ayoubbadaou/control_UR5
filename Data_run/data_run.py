from deep_detect import ArucoDetector
from mqtt_read import MqttData, OpcUaServer
from ur5_opcua_tcp import Robot
import threading
import time
import cv2

def sensors_thread(mqtt_data, opcua_server):
    mqtt_data.start()
    time.sleep(1)
    opcua_server.start()

def main():
    # Instantiate detectors
    detector1 = ArucoDetector(camera_index=6, cam_id=0)
    detector2 = ArucoDetector(camera_index=12, cam_id=1)
    detector3 = ArucoDetector(camera_index=18, cam_id=2)

    # Instantiate robot & MQTT interface
    robot = Robot()
    mqtt_data = MqttData()
    opcua_server = OpcUaServer(mqtt_data)

    # Create threads for each component
    threads = [
        threading.Thread(target=detector1.run, daemon=True),
        threading.Thread(target=detector2.run, daemon=True),
        threading.Thread(target=detector3.run, daemon=True),
        threading.Thread(target=robot.run, daemon=True),
        threading.Thread(target=sensors_thread, args=(mqtt_data, opcua_server), daemon=True),
    ]

    # Start all threads
    for t in threads:
        t.start()
    while True:
        detector1.show_frame()
        detector2.show_frame()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    
    # Keep the main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("🛑 Program interrupted. Exiting...")

if __name__ == "__main__":
    main()

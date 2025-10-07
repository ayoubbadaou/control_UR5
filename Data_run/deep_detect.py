# aruco_detector.py
import cv2
import numpy as np
from opcua import Server, ua
import threading

class ArucoDetector:
    def __init__(self, camera_index=0, marker_size=0.04, cam_id=0):
        self.camera_index = camera_index
        self.cam_id = cam_id
        self.marker_size = marker_size
        self.camera_matrix = np.array([
            [910.7143457624005, 0.0, 653.4930616632473],
            [0.0, 909.0258096414667, 371.1417530429486],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([0.218435, -0.729251, -0.000172, -0.005002, 0.768460])
        self.window_position = (100 + self.cam_id * 300, 100)
        self.window_size = (640, 480)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.marker_corners_3d = np.array([
            [-self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, -self.marker_size / 2, 0],
            [-self.marker_size / 2, -self.marker_size / 2, 0]
        ], dtype=np.float32)

        self.opcua_endpoint = f"opc.tcp://0.0.0.0:200{self.cam_id}"
        self.server = None
        self.marker_vars = {}

        self.cap = None
        self.latest_frame = None
        self.lock = threading.Lock()

    def initialize_opcua_server(self):
        self.server = Server()
        self.server.set_endpoint(self.opcua_endpoint)
        ns_idx = self.server.register_namespace("ArUcoData")
        objects = self.server.get_objects_node()

        for marker_num in [1, 2]:
            self.marker_vars[f"{self.cam_id}marker{marker_num}_pos"] = objects.add_variable(
                ua.NodeId(f"{self.cam_id}Marker{marker_num}_Pos", ns_idx),
                f"{self.cam_id}Marker{marker_num}_Position",
                "0,0,0,0,0,0"
            )
            self.marker_vars[f"{self.cam_id}marker{marker_num}_pos"].set_writable(True)

            for axis in ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']:
                self.marker_vars[f"{self.cam_id}marker{marker_num}_{axis.lower()}"] = objects.add_variable(
                    ua.NodeId(f"{self.cam_id}Marker{marker_num}_{axis}", ns_idx),
                    f"{self.cam_id}Marker{marker_num}_{axis}",
                    0.0
                )
                self.marker_vars[f"{self.cam_id}marker{marker_num}_{axis.lower()}"] .set_writable(True)

        self.server.start()
        print(f"✅ OPC UA Server started at {self.opcua_endpoint}")

    def initialize_camera(self):
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera at index {self.camera_index}")
        print(f"✅ Camera {self.cam_id} initialized.")

    def update_marker_data(self, marker_num, x, y, z, rx, ry, rz):
        self.marker_vars[f"{self.cam_id}marker{marker_num}_pos"].set_value(f"{x:.3f},{y:.3f},{z:.3f},{rx:.3f},{ry:.3f},{rz:.3f}")
        self.marker_vars[f"{self.cam_id}marker{marker_num}_x"].set_value(x)
        self.marker_vars[f"{self.cam_id}marker{marker_num}_y"].set_value(y)
        self.marker_vars[f"{self.cam_id}marker{marker_num}_z"].set_value(z)
        self.marker_vars[f"{self.cam_id}marker{marker_num}_rx"].set_value(rx)
        self.marker_vars[f"{self.cam_id}marker{marker_num}_ry"].set_value(ry)
        self.marker_vars[f"{self.cam_id}marker{marker_num}_rz"].set_value(rz)

    def reset_marker_data(self, marker_num):
        self.update_marker_data(marker_num, 0, 0, 0, 0, 0, 0)

    def detect_markers(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            self.reset_marker_data(1)
            self.reset_marker_data(2)

            corners, ids, _ = self.detector.detectMarkers(frame)

            if ids is not None:
                for i, (corner, marker_id) in enumerate(zip(corners, ids)):
                    if i >= 2:
                        continue
                    success, rvec, tvec = cv2.solvePnP(
                        self.marker_corners_3d,
                        corner[0],
                        self.camera_matrix,
                        self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if success:
                        rmat, _ = cv2.Rodrigues(rvec)
                        angles = cv2.RQDecomp3x3(rmat)[0]
                        x, y, z = tvec.flatten()
                        rx, ry, rz = angles
                        self.update_marker_data(i+1, x, y, z, rx, ry, rz)
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size/2)
                        center = corner[0].mean(axis=0).astype(int)
                        cv2.putText(frame, f"ID:{marker_id[0]} Pos:{x:.2f},{y:.2f},{z:.2f}",
                                    (center[0], center[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                        cv2.putText(frame, f"Rot:{rx:.1f},{ry:.1f},{rz:.1f}",
                                    (center[0], center[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

            with self.lock:
                self.latest_frame = frame.copy()

    def cleanup(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        if self.server:
            self.server.stop()

    def run(self):
        try:
            self.initialize_opcua_server()
            self.initialize_camera()
            self.detect_markers()
        except Exception as e:
            print(f"❌ Error in camera {self.cam_id}: {e}")
        finally:
            self.cleanup()

    def get_latest_frame(self):
        with self.lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def show_frame(self):
        frame = self.get_latest_frame()
        if frame is not None:
            win_name = f"Camera {self.cam_id}"
            cv2.imshow(win_name, frame)
            cv2.resizeWindow(win_name, *self.window_size)

if __name__ == "__main__":
    detector = ArucoDetector(camera_index=0, cam_id=0)
    threading.Thread(target=detector.run, daemon=True).start()
    detector.display_loop()


import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

# Load your calibration file
with open('TF_matrix.yaml') as f:
    T_base2camera = np.array(yaml.safe_load(f)['T_calibrated_transform'])

# Extract camera pose as TCP format [x, y, z, rx, ry, rz]
x = T_base2camera[0, 3]  # X position in meters
y = T_base2camera[1, 3]  # Y position in meters  
z = T_base2camera[2, 3]  # Z position in meters

# Convert rotation matrix to rotation vector
rx, ry, rz = R.from_matrix(T_base2camera[:3, :3]).as_rotvec()

camera_tcp_pose = [x, y, z, rx, ry, rz]

print("Camera TCP Pose in Robot Base Frame:")
print(f"Position: X={x:.4f}m, Y={y:.4f}m, Z={z:.4f}m")
print(f"Orientation: Rx={rx:.4f}rad, Ry={ry:.4f}rad, Rz={rz:.4f}rad")
print(f"Full array: {camera_tcp_pose}")

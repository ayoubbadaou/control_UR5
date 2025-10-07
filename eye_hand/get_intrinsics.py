import pyrealsense2 as rs

pipeline = rs.pipeline()
pipeline.start()

intrinsics = pipeline.wait_for_frames().get_color_frame().profile.as_video_stream_profile().intrinsics

print(f"fx: {intrinsics.fx:.2f}, fy: {intrinsics.fy:.2f}")
print(f"cx: {intrinsics.ppx:.2f}, cy: {intrinsics.ppy:.2f}")
print(f"Distortion: {intrinsics.coeffs}")

pipeline.stop()

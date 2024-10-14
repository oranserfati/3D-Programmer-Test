import cv2
import keyboard
import open3d as o3d
import json
import time
import numpy as np

# We have the option to load the trajectory data from json or enter manualy
use_json = False
frames = range(100)

export_vid = False

# Load the camera trajectory from the JSON file
with open("camera_trajectory.json", "r") as f:
    camera_trajectory = json.load(f)

# Extract intrinsic parameters
intrinsic = camera_trajectory["intrinsic"]
width = intrinsic["width"]
height = intrinsic["height"]
fx = intrinsic["fx"]
fy = intrinsic["fy"]
cx = intrinsic["cx"]
cy = intrinsic["cy"]

# Load the 3D scene
mesh = o3d.io.read_triangle_mesh("Models/village_house_obj.obj")
mesh.compute_vertex_normals()

# Setup the visualizer window
vis = o3d.visualization.Visualizer()
vis.create_window(width=width, height=height)

# Set up video writer with correct resolution
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use mp4v for .mp4 format
out = cv2.VideoWriter('output_video.mp4', fourcc, 30.0, (width, height))

# Add geometry to the scene
vis.add_geometry(mesh)

# Create a PinholeCameraParameters object to apply to the camera
camera_params = o3d.camera.PinholeCameraParameters()

# Create camera intrinsic object
camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(
    width, height, fx, fy, cx, cy
)
camera_params.intrinsic = camera_intrinsics

ctr = vis.get_view_control()

speed_modifier = 0.0

def get_speed_input():
    global speed_modifier
    
    # Increase rotation speed
    if keyboard.is_pressed('Z'):
        speed_modifier = min(speed_modifier + 0.01, 0.04)
        print(f"Speed: {speed_modifier}")
    # Decrease rotation speed
    if keyboard.is_pressed('X'):
        speed_modifier = max(speed_modifier - 0.01, -0.4)
        print(f"Speed: {speed_modifier}")

def capture_image():
    # Set video params
    img = vis.capture_screen_float_buffer(do_render=True)
    img = (np.asarray(img) * 255).astype(np.uint8)  
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) 
    img_resized = cv2.resize(img, (width, height))

    out.write(img_resized)

def move_camera(extrinsic_matrix):
    # Set the extrinsic (position and orientation) to camera parameters
    camera_params.extrinsic = extrinsic_matrix
    
    # Apply the camera parameters to the view control
    ctr.convert_from_pinhole_camera_parameters(camera_params, True)
    
    # Render the scene
    vis.poll_events()
    vis.update_renderer()
    
    # Print camera's current location
    camera_location = extrinsic_matrix[:3, 3]
    # print(f"Camera Location: {camera_location}")
    
    if export_vid:
        capture_image()
    
    get_speed_input()
    
    time.sleep(0.05 - speed_modifier)

# Apply each frame's extrinsic parameters to the camera in a loop
def apply_camera_trajectory(frames, trajectory):
    
    # Get trajectory data from json or manualy
    if use_json:
        for frame_data in trajectory["extrinsic"]:
            extrinsic_matrix = np.array(frame_data["extrinsic_matrix"]) 
            move_camera(extrinsic_matrix)
    else:
        for f in frames:
            extrinsic_matrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, -40],
            [0, 0, 1, -f],
            [0, 0, 0, 1]
            ])
            move_camera(extrinsic_matrix)
         
apply_camera_trajectory(frames, camera_trajectory)

# Release resources
vis.destroy_window()
out.release()
cv2.destroyAllWindows()

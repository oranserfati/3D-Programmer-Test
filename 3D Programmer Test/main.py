import keyboard
import open3d as o3d
import numpy as np
import time


# Load 3D scene
mesh_file = "Models/Knight.obj" 
mesh = o3d.io.read_triangle_mesh(mesh_file)

# Reference Mesh
rmesh_file = "Models/Cube.obj"
rmesh = o3d.io.read_triangle_mesh(rmesh_file)

# Check if the meshes loaded successfully
if not mesh.is_empty() and not rmesh.is_empty():
    mesh.compute_vertex_normals()
    rmesh.compute_vertex_normals()
else:
    print(f"Error loading mesh. Make sure the file exists and is a valid format.")
    exit()

# Rotation fix specify for 'Knight' model
def set_knight_model():
        rotation_angle = np.radians(90) 
        R = mesh.get_rotation_matrix_from_xyz((rotation_angle, 0, 0)) 
        mesh.rotate(R, center=(0, 0, 0)) 
            
if mesh_file == "Models/Knight.obj": 
    set_knight_model()       
    
# Set the reference mesh position
rmesh.translate([-20,0,0])

# Setup visualization window
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh)
vis.add_geometry(rmesh)

# Define camera trajectory parameters
center = mesh.get_center()
radius = 10
num_frames = 360  # Num of dots in the trajectory path

# Create a circular path for the camera
def create_camera_path(radius, num_frames, center):
    path = []
    for i in range(num_frames):
        angle = 2 * np.pi * i / num_frames
        x = radius * np.cos(angle) + center[0]
        y = radius * np.sin(angle) + center[1]
        z = 3  # Slightly upward tilt for a better look 
        path.append([x, y, z])
    return path

# Generate camera path
camera_path = create_camera_path(radius, num_frames, center)

# Get the view control to set camera positions
ctr = vis.get_view_control()

# Function to move the camera along the path
def move_camera_along_path(camera_path, frame):
    point = camera_path[frame]   
    # Set the camera position and rotation
    ctr.set_lookat(center)  
    ctr.set_front((np.array(point)))  
    ctr.set_up([0, 0, 1])  
    ctr.set_zoom(0.5)  
    
# Function to show text on the window
def show_text_on_window(text, position):
    # Add 3D label to the mesh at the desired position
    vis.add_3d_label(position, text)
    #vis.update_renderer()

# Main function to run the animation
def run_animation():
    
    frame_buffer = 0
    speed_multi = 1
    
    # Keep the window open until the user closes it
    while vis.poll_events():
        # Reset for continues rotation
        if frame_buffer > num_frames - 1: 
            frame_buffer = 0   
        move_camera_along_path(camera_path, frame_buffer)
        # Increase rotation speed
        if keyboard.is_pressed('Z'):
            speed_multi = min(speed_multi + 1, 10)
        # Decrease rotation speed
        if keyboard.is_pressed('X'):
            speed_multi = max(speed_multi - 1, 1)
        frame_buffer += 1 * speed_multi
        vis.update_renderer()
    vis.destroy_window()
 
run_animation()

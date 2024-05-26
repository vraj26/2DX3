import serial
import numpy as np
import open3d as o3d
import math
import time

# Initialize serial communication with laptop
s = serial.Serial(port='COM4', baudrate=115200, timeout=10)
print("Opening: " + s.name)

# Reset input and output buffers
s.reset_output_buffer()
s.reset_input_buffer()

# List to store combined cartesian coordinates
combined = []
scans = 12  # Number of scans
measurements = 32  # Number of measurements per scan

# Function to convert polar coordinates to cartesian coordinates
def conv_cart(data):
    for i in range(len(data)): 
        X = data[i] * math.sin(math.radians(11.25*i)) 
        Y = data[i] * math.cos(math.radians(11.25*i))
        Z = k * 500  # Placeholder Z coordinate
        combined.append((X, Y, Z))
        print(X, Y, Z)

# Loop through the scans
for k in range(scans):
    data = []
    input("press enter to start")
    s.write(b's')  # Send 's' to start scan
    
    # Read measurements from LiDAR
    for i in range(measurements):
        x = s.readline()
        data_str = x.decode().strip()  
        if data_str.isdigit():  
            if (int(data_str) > 4000):
                data.append(4000)
            else:
                data.append(int(data_str))
            print(x)
    
    print("Number of data points:", len(data))
    conv_cart(data)

s.close()  # Close serial connection
print("Closing: " + s.name)

# Write cartesian coordinates to a file
with open('projectscan.xyz', 'w') as f:
    for X, Y, Z in combined:
        f.write(f"{X:f} {Y:f} {Z:f}\n")
f.close()

# Read point cloud data from the file
pcd = o3d.io.read_point_cloud("projectscan.xyz", format="xyz")

# Print the point cloud array
print("The PCD array:")
print(np.asarray(pcd.points))

# Visualize the point cloud
print("Let's visualize the PCD: (spawns separate interactive window)")
o3d.visualization.draw_geometries([pcd])

# Convert combined_data to numpy array
points = np.asarray(combined)

# Create lines within and between scans
lines_within_scan = []
for scan in range(scans):
    for i in range(measurements - 1):
        index = scan * measurements + i
        lines_within_scan.append([index, index + 1])

for scan in range(scans):
    start_index = scan * measurements
    end_index = (scan + 1) * measurements - 1
    lines_within_scan.append([end_index, start_index])

lines_between_scans = []
for scan in range(scans - 1):
    for i in range(measurements):
        current_index = scan * measurements + i
        next_index = (scan + 1) * measurements + i
        lines_between_scans.append([current_index, next_index])

# Combine the lines
all_lines = lines_within_scan + lines_between_scans

# Create LineSet
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(all_lines)
)

# Visualize point cloud with lines
o3d.visualization.draw_geometries([line_set])

# Write point cloud to a file
o3d.io.write_point_cloud("path_to_your_file.ply", pcd)
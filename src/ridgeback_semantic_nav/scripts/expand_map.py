#!/usr/bin/env python3
import cv2
import numpy as np
import os
import yaml

# Path to map files
map_dir = os.path.expanduser("~/capstone_final/ridgeback_ws/src/orb_slam3_ros/maps")
input_yaml = os.path.join(map_dir, "my_map.yaml")
input_pgm = os.path.join(map_dir, "my_map.pgm")
output_yaml = os.path.join(map_dir, "my_map_large.yaml")
output_pgm = os.path.join(map_dir, "my_map_large.pgm")

# Load map YAML data
with open(input_yaml, 'r') as f:
    map_data = yaml.safe_load(f)

# Get current map parameters
resolution = map_data['resolution']
old_origin = map_data['origin']
print(f"Original map: resolution={resolution}, origin={old_origin}")

# Load map image
map_img = cv2.imread(input_pgm, cv2.IMREAD_GRAYSCALE)
if map_img is None:
    print(f"Failed to load map image: {input_pgm}")
    exit(1)

original_height, original_width = map_img.shape
print(f"Original map size: {original_width}x{original_height} pixels")

# Define padding (in pixels) to add on each side
padding = 200

# Create new, larger image with padding
new_width = original_width + 2*padding
new_height = original_height + 2*padding
large_map = np.ones((new_height, new_width), dtype=np.uint8) * 205  # 205 is "unknown" in maps

# Place original map in the center of the new map
large_map[padding:padding+original_height, padding:padding+original_width] = map_img

# Calculate new origin (adjusting for the padding)
new_origin_x = old_origin[0] - padding * resolution
new_origin_y = old_origin[1] - padding * resolution
new_origin = [new_origin_x, new_origin_y, old_origin[2]]

# Save the new, larger map
cv2.imwrite(output_pgm, large_map)
print(f"Saved expanded map to: {output_pgm}")

# Create and save new YAML file
new_map_data = map_data.copy()
new_map_data['image'] = output_pgm
new_map_data['origin'] = new_origin

with open(output_yaml, 'w') as f:
    yaml.dump(new_map_data, f, default_flow_style=False)

print(f"Saved expanded map YAML to: {output_yaml}")
print(f"New map: resolution={resolution}, origin={new_origin}")
print(f"New map size: {new_width}x{new_height} pixels = {new_width*resolution}x{new_height*resolution} meters")
print(f"To use this map, launch with: roslaunch ridgeback_semantic_nav absolute_path_navigation.launch map_file:={output_yaml}")
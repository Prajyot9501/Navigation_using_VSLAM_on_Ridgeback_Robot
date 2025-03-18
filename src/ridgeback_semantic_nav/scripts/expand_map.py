#!/usr/bin/env python3
import cv2
import numpy as np
import os
import yaml

# Path to map files
map_dir = os.path.expanduser("~/capstone_final/ridgeback_ws/src/orb_slam3_ros/maps")
input_yaml = os.path.join(map_dir, "my_map.yaml")
input_pgm = os.path.join(map_dir, "my_map.pgm")
output_yaml = os.path.join(map_dir, "my_map_scaled.yaml")
output_pgm = os.path.join(map_dir, "my_map_scaled.pgm")

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

# Scale factor (increase this to scale up the map/zoom in)
scale_factor = 2.0  # Double the size

# Calculate new dimensions
new_width = int(original_width * scale_factor)
new_height = int(original_height * scale_factor)

# Resize the map (scale up)
scaled_map = cv2.resize(map_img, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

# Calculate new resolution (smaller resolution for zoomed in map)
new_resolution = resolution / scale_factor

# Calculate new origin
# Origin needs adjustment because scaling happens around the center
new_origin_x = old_origin[0] * scale_factor
new_origin_y = old_origin[1] * scale_factor
new_origin = [new_origin_x, new_origin_y, old_origin[2]]

# Save the new, scaled map
cv2.imwrite(output_pgm, scaled_map)
print(f"Saved scaled map to: {output_pgm}")

# Create and save new YAML file
new_map_data = map_data.copy()
new_map_data['image'] = output_pgm
new_map_data['resolution'] = new_resolution
new_map_data['origin'] = new_origin

with open(output_yaml, 'w') as f:
    yaml.dump(new_map_data, f, default_flow_style=False)

print(f"Saved scaled map YAML to: {output_yaml}")
print(f"New map: resolution={new_resolution}, origin={new_origin}")
print(f"New map size: {new_width}x{new_height} pixels = {new_width*new_resolution}x{new_height*new_resolution} meters")
print(f"The map is now scaled up by {scale_factor}x which should make navigation easier with the Ridgeback model")
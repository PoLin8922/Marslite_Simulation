#!/usr/bin/env python3

import json
import matplotlib.pyplot as plt
import math
import numpy as np
import cv2

# Load map metadata
map_file = '/home/developer/lab/socially-store-robot/mars_ws/src/tools/experiment_tools/maps/shoppingmall_v3.pgm'
resolution = 0.050000
origin = [-50.000000, -50.000000, 0.000000]

def load_positions(file_name):
    with open(file_name, 'r') as f:
        positions = json.load(f)
    return positions

def calculate_path_length(positions):
    path_length = 0.0
    for i in range(1, len(positions)):
        x1, y1 = positions[i-1]['x'], positions[i-1]['y']
        x2, y2 = positions[i]['x'], positions[i]['y']
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        path_length += distance
    return path_length

def transform_positions(positions, resolution, origin):
    x = [(pos['x'] - origin[0]) / resolution for pos in positions]
    y = [(pos['y'] - origin[1]) / resolution for pos in positions]
    return x, y

def plot_path_on_map(positions, map_file, resolution, origin):
    # Load the map image
    map_img = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
    map_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
    
    # Transform positions to map image coordinates
    x, y = transform_positions(positions, resolution, origin)
    
    # Calculate the bounding box
    min_x, max_x = min(x), max(x)
    min_y, max_y = min(y), max(y)
    
    # Determine the side length of the square bounding box
    bias = 100
    side_length = max(max_x - min_x, max_y - min_y) + bias
    
    # Adjust the bounding box to make the axes equal
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    min_x = center_x - side_length / 2
    max_x = center_x + side_length / 2
    min_y = center_y - side_length / 2
    max_y = center_y + side_length / 2
    
    # Crop the map image
    min_x_img, min_y_img = int(min_x), int(map_img.shape[0] - max_y)
    max_x_img, max_y_img = int(max_x), int(map_img.shape[0] - min_y)
    cropped_map = map_img[min_y_img:max_y_img, min_x_img:max_x_img]
    
    # Transform the cropped positions
    x_cropped = [(pos - min_x) for pos in x]
    y_cropped = [((map_img.shape[0] - pos) - min_y_img) for pos in y]

    path_img = np.zeros_like(cropped_map)
    
    # Plot the path on the cropped map
    for i in range(1, len(x_cropped)):
        cv2.line(cropped_map, (int(x_cropped[i-1]), int(y_cropped[i-1])), (int(x_cropped[i]), int(y_cropped[i])), (0, 0, 255), 2)
    
    # Display the image with the path
    plt.figure(figsize=(10, 10))
    plt.imshow(cropped_map)
    plt.title('Robot Path on Map')
    plt.axis('off')
    plt.show()

if __name__ == '__main__':
    file_name = '/home/developer/lab/socially-store-robot/mars_ws/src/tools/experiment_tools/files/path_19.json'
    positions = load_positions(file_name)
    path_length = calculate_path_length(positions)
    print(f"Path length: {path_length:.2f} meters")
    plot_path_on_map(positions, map_file, resolution, origin)

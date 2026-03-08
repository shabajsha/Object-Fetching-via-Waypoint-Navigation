#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import argparse
import math

def parse_world(world_file):
    """Parse Gazebo world and extract collision boxes"""
    tree = ET.parse(world_file)
    root = tree.getroot()
    
    obstacles = []
    
    # Find all models with collision geometry
    for model in root.findall('.//model'):
        model_name = model.get('name')
        pose = model.find('pose')
        
        if pose is not None:
            pose_text = pose.text.strip().split()
            x, y, z = float(pose_text[0]), float(pose_text[1]), float(pose_text[2])
            if len(pose_text) >= 6:
                yaw = float(pose_text[5])
            else:
                yaw = 0
        else:
            x, y, z, yaw = 0, 0, 0, 0
        
        # Find collision boxes
        for link in model.findall('.//link'):
            collision = link.find('collision')
            if collision is not None:
                box = collision.find('.//box/size')
                if box is not None:
                    size_text = box.text.strip().split()
                    width = float(size_text[0])
                    depth = float(size_text[1])
                    
                    obstacles.append({
                        'x': x,
                        'y': y,
                        'width': width,
                        'depth': depth,
                        'yaw': yaw,
                        'name': model_name
                    })
                    
                    print(f"  Found: {model_name} at ({x:.2f}, {y:.2f}) size {width}x{depth}")
    
    return obstacles

def create_occupancy_grid(obstacles, resolution=0.05, width=200, height=200):
    """Create 2D occupancy grid from obstacles"""
    grid = np.ones((height, width), dtype=np.uint8) * 255  # free space = white
    
    center_x = width / 2
    center_y = height / 2
    
    # Set occupied cells
    for obs in obstacles:
        # Convert world coords to grid coords
        grid_x = int((obs['x'] / resolution) + center_x)
        grid_y = int((obs['y'] / resolution) + center_y)
        
        half_w = int(obs['width'] / (2 * resolution))
        half_h = int(obs['depth'] / (2 * resolution))
        
        # Clamp to grid bounds
        x_min = max(0, grid_x - half_w)
        x_max = min(width, grid_x + half_w)
        y_min = max(0, grid_y - half_h)
        y_max = min(height, grid_y + half_h)
        
        grid[y_min:y_max, x_min:x_max] = 0  # occupied = black
    
    return grid

def save_map(grid, output_file, resolution=0.05):
    """Save grid as PGM and YAML"""
    img = Image.fromarray(grid.astype(np.uint8), mode='L')
    img.save(output_file + '.pgm')
    print(f"Saved: {output_file}.pgm")
    
    # Create YAML meta file
    height, width = grid.shape
    yaml_content = f"""image: {output_file.split('/')[-1]}.pgm
resolution: {resolution}
origin: [{-(width/2)*resolution}, {-(height/2)*resolution}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    
    with open(output_file + '.yaml', 'w') as f:
        f.write(yaml_content)
    
    print(f"Saved: {output_file}.yaml")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True, help='Path to Gazebo world file')
    parser.add_argument('--output', required=True, help='Output map file prefix')
    parser.add_argument('--resolution', type=float, default=0.05, help='Map resolution in meters/pixel')
    
    args = parser.parse_args()
    
    print(f"[*] Parsing world: {args.world}")
    obstacles = parse_world(args.world)
    print(f"[*] Found {len(obstacles)} obstacles\n")
    
    print(f"[*] Creating occupancy grid (resolution: {args.resolution}m/px)")
    grid = create_occupancy_grid(obstacles, resolution=args.resolution)
    
    print(f"[*] Saving map...")
    save_map(grid, args.output, resolution=args.resolution)
    print(f"\n[✓] Done! Use this map with Nav2:")
    print(f"    ros2 launch turtlebot3_navigation2 navigation2.launch.py \\")
    print(f"      use_sim_time:=True map:={args.output}.yaml")
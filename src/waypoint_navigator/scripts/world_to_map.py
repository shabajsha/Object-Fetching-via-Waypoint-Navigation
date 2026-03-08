#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import argparse
import math

def parse_world(world_file):
    """Parse Gazebo world and extract collision geometry"""
    tree = ET.parse(world_file)
    root = tree.getroot()
    
    obstacles = []
    
    # Find all models
    for model in root.findall('.//model'):
        model_name = model.get('name')
        pose_elem = model.find('pose')
        
        # Get model pose
        if pose_elem is not None:
            pose_text = pose_elem.text.strip().split()
            if len(pose_text) >= 3:
                x, y, z = float(pose_text[0]), float(pose_text[1]), float(pose_text[2])
            else:
                x, y, z = 0, 0, 0
            if len(pose_text) >= 6:
                yaw = float(pose_text[5])
            else:
                yaw = 0
        else:
            x, y, z, yaw = 0, 0, 0, 0
        
        # Check if static (walls)
        static = model.find('static') is not None and model.find('static').text == '1'
        
        # Find collision boxes
        for link in model.findall('.//link'):
            collision = link.find('collision')
            if collision is not None:
                # Box geometry
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
                        'name': model_name,
                        'type': 'box'
                    })
                    print(f"  [BOX] {model_name} at ({x:.2f}, {y:.2f}) size {width}x{depth}")
                
                # Mesh geometry (for iscas_museum)
                mesh = collision.find('.//mesh/uri')
                if mesh is not None:
                    mesh_uri = mesh.text
                    print(f"  [MESH] {model_name} at ({x:.2f}, {y:.2f}) - {mesh_uri}")
                    
                    # For ISCAS museum, use approximate bounding box
                    if 'iscas_museum' in mesh_uri.lower() or 'museum' in model_name.lower():
                        # ISCAS museum approximate size (large building)
                        obstacles.append({
                            'x': x,
                            'y': y,
                            'width': 30,
                            'depth': 30,
                            'yaw': yaw,
                            'name': model_name,
                            'type': 'mesh'
                        })
                        print(f"    → Using approximate bounding box 30x30m")
    
    return obstacles

def create_occupancy_grid(obstacles, resolution=0.05, width=400, height=400):
    """Create 2D occupancy grid from obstacles"""
    grid = np.ones((height, width), dtype=np.uint8) * 255  # free space = white
    
    center_x = width / 2
    center_y = height / 2
    
    print(f"\n[*] Creating {width}x{height} grid (resolution: {resolution}m/px)")
    print(f"[*] Grid covers: {-(width/2)*resolution} to {(width/2)*resolution} m in X")
    print(f"[*] Grid covers: {-(height/2)*resolution} to {(height/2)*resolution} m in Y\n")
    
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
        
        if x_min < x_max and y_min < y_max:
            grid[y_min:y_max, x_min:x_max] = 0  # occupied = black
            print(f"  Placed {obs['name']:20s} grid:({x_min:3d},{y_min:3d}) to ({x_max:3d},{y_max:3d})")

    return grid

def save_map(grid, output_file, resolution=0.05):
    """Save grid as PGM and YAML"""
    img = Image.fromarray(grid.astype(np.uint8), mode='L')
    img.save(output_file + '.pgm')
    print(f"\n[✓] Saved: {output_file}.pgm")
    
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
    
    print(f"[✓] Saved: {output_file}.yaml")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True, help='Path to Gazebo world file')
    parser.add_argument('--output', required=True, help='Output map file prefix')
    parser.add_argument('--resolution', type=float, default=0.05, help='Map resolution in meters/pixel')
    parser.add_argument('--width', type=int, default=400, help='Grid width in pixels')
    parser.add_argument('--height', type=int, default=400, help='Grid height in pixels')
    
    args = parser.parse_args()
    
    print(f"\n[*] Parsing world: {args.world}")
    obstacles = parse_world(args.world)
    print(f"\n[*] Found {len(obstacles)} obstacles/models")
    
    print(f"\n[*] Creating occupancy grid...")
    grid = create_occupancy_grid(obstacles, resolution=args.resolution, 
                                  width=args.width, height=args.height)
    
    print(f"[*] Saving map...")
    save_map(grid, args.output, resolution=args.resolution)
    print(f"\n[✓] Done! Use this map with Nav2:")
    print(f"    ros2 launch turtlebot3_navigation2 navigation2.launch.py \\")
    print(f"      use_sim_time:=True map:={args.output}.yaml\n")
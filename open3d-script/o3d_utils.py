import open3d as o3d
import numpy as np

def create_grid_mesh(width, height, grid_spacing):

    width_grid_lines = int(width // grid_spacing) + 1
    height_grid_lines = int(height // grid_spacing) + 1

    grid_mesh = o3d.geometry.LineSet()
    points = np.zeros((2*(width_grid_lines + height_grid_lines), 3), dtype=np.float32)
    lines = []

    for i in range(width_grid_lines):
        points[2*i] = [i*grid_spacing, 0, 0]
        points[2*i+1] = [i*grid_spacing, (height_grid_lines-1)*grid_spacing, 0]
        lines.append([2*i, 2*i+1])
    
    for i in range(height_grid_lines):
        points[2* (i + width_grid_lines)] = [0, i*grid_spacing, 0]
        points[2* (i + width_grid_lines)+1] = [(width_grid_lines-1)*grid_spacing, i*grid_spacing, 0]
        lines.append([2* (i + width_grid_lines), 2* (i + width_grid_lines)+1])
    grid_mesh.points = o3d.utility.Vector3dVector(points)
    grid_mesh.lines = o3d.utility.Vector2iVector(lines)

    return grid_mesh

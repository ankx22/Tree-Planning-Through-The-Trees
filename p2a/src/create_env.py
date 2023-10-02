import bpy
import numpy as np
from helperfuncs import *
import mathutils
import math


class Environment:
    def __init__(self, filepath) -> None:
        self.filepath = filepath
        self.map_array = np.array([])
        self.map_array_scale = 10
        self.bloat_amount = 0.1
        self.lowerboundary = None
        self.upperboundary = None

    def make_env(self, bloat_amount=0.1):
        try:
            # Read the file content
            with open(self.filepath, 'r') as f:
                content = f.read()
        except FileNotFoundError:
            return

        # Process the content
        lines = content.split('\n')

        for line in lines:
            words = line.split()
            if words:
                if words[0] == 'boundary':
                    boundary = [float(i) for i in words[1:7]]
                    self.lowerboundary = np.array(
                        [boundary[0], boundary[1], boundary[2]])
                    self.upperboundary = np.array(
                        [boundary[3], boundary[4], boundary[5]])
                    # Create the boundary cube
                    bpy.ops.mesh.primitive_cube_add(size=1, location=((boundary[0] + boundary[3]) / 2,
                                                                      (boundary[1] +
                                                                       boundary[4]) / 2,
                                                                      (boundary[2] + boundary[5]) / 2))

                    boundary_cube = bpy.context.object
                    boundary_cube.dimensions = [round(boundary[3] - boundary[0], 1), round(
                        boundary[4] - boundary[1], 1), round(boundary[5] - boundary[2], 1)]

                    # Get existing transparent material or create a new one
                    mat = bpy.data.materials.get("Transparent_Material")
                    if not mat:
                        mat = bpy.data.materials.new(
                            name="Transparent_Material")
                        mat.use_nodes = True
                        nodes = mat.node_tree.nodes
                        for node in nodes:
                            nodes.remove(node)
                        transparent_bsdf = nodes.new(
                            type='ShaderNodeBsdfTransparent')
                        material_output = nodes.new(
                            type='ShaderNodeOutputMaterial')
                        mat.node_tree.links.new(
                            transparent_bsdf.outputs["BSDF"], material_output.inputs["Surface"])
                        mat.blend_method = 'BLEND'

                    # Assign the material to the boundary_cube
                    boundary_cube.data.materials.append(mat)

                    # Subdivide the cube to create a grid effect
                    bpy.ops.object.mode_set(mode='EDIT')
                    bpy.ops.mesh.subdivide(number_cuts=10)
                    bpy.ops.object.mode_set(mode='OBJECT')

                    # # Subdivide the cube to create a grid effect
                    # bpy.ops.object.mode_set(mode='EDIT')
                    # bpy.ops.mesh.subdivide(number_cuts=10)
                    # bpy.ops.object.mode_set(mode='OBJECT')

                    # # Shader setup for black color with alpha transparency of 0.344
                    # mat = bpy.data.materials.new(name="Grid_Material")
                    # mat.use_nodes = True
                    # nodes = mat.node_tree.nodes
                    # bsdf = nodes["Principled BSDF"]
                    # bsdf.inputs[0].default_value = (0, 0, 0, 0.344)

                    # # Set blend mode to Alpha Blend
                    # mat.blend_method = 'BLEND'
                    # boundary_cube.data.materials.append(mat)

                    max_x, max_y, max_z = round(boundary[3]-boundary[0]+2*bloat_amount, 1), round(
                        boundary[4]-boundary[1]+2*bloat_amount, 1), round(boundary[5]-boundary[2]+2*bloat_amount, 1)
                    self.map_array = np.ones((int(self.map_array_scale*max_x)+1, int(
                        self.map_array_scale*max_y)+1, int(self.map_array_scale*max_z)+1), dtype=np.uint8)

                elif words[0] == 'block':
                    block_coords = [float(i) for i in words[1:7]]
                    color = [int(float(i))/255 for i in words[7:10]]

                    # x_start, y_start, z_start, x_end, y_end, z_end = map(int, block_coords)
                    # self.map_array[50*x_start:50*x_end, 50*y_start:50*y_end, 50*z_start:50*z_end] = 0  # 0 denotes obstacles

                    # Calculate the bloated dimensions
                    bloated_dimensions = [
                        round(
                            (block_coords[3] - block_coords[0]) + 2 * bloat_amount, 1),
                        round(
                            (block_coords[4] - block_coords[1]) + 2 * bloat_amount, 1),
                        round(
                            (block_coords[5] - block_coords[2]) + 2 * bloat_amount, 1),
                    ]

                    # Adjust the location to account for the increased dimensions
                    bloated_location = [
                        (block_coords[0] + block_coords[3]) / 2,
                        (block_coords[1] + block_coords[4]) / 2,
                        (block_coords[2] + block_coords[5]) / 2,
                    ]
                    bloated_coords = [
                        block_coords[0] - bloat_amount,
                        block_coords[1] - bloat_amount,
                        block_coords[2] - bloat_amount,
                        block_coords[3] + bloat_amount,
                        block_coords[4] + bloat_amount,
                        block_coords[5] + bloat_amount,
                    ]
                    # bloated_coords =[]
                    # if 0.<block_coords[0] and block_coords[3]<10.:

                    # elif block_coords[0]<=0. and block_coords[3]<10.:
                    #     bloated_coords = [
                    #         block_coords[0],
                    #         block_coords[1] - bloat_amount,
                    #         block_coords[2] - bloat_amount,
                    #         block_coords[3]+bloat_amount,
                    #         block_coords[4] + bloat_amount,
                    #         block_coords[5] + bloat_amount,
                    #     ]
                    # else:
                    #     bloated_coords = [
                    #         block_coords[0],
                    #         block_coords[1] - bloat_amount,
                    #         block_coords[2] - bloat_amount,
                    #         block_coords[3],
                    #         block_coords[4] + bloat_amount,
                    #         block_coords[5] + bloat_amount,
                    #     ]

                    # print("bloated coords", bloated_coords)
                    # print("bloated dimensions", bloated_dimensions)
                    # print("bloated location", bloated_location)
                    x_start, y_start, z_start, x_end, y_end, z_end = bloated_coords[0], bloated_coords[
                        1], bloated_coords[2], bloated_coords[3], bloated_coords[4], bloated_coords[5]
                    indices_start = convrule(
                        x_start, y_start, z_start, boundary[0], boundary[1], boundary[2], self.map_array_scale, bloat_amount)
                    indices_end = convrule(
                        x_end, y_end, z_end, boundary[0], boundary[1], boundary[2], self.map_array_scale, bloat_amount)
                    self.map_array[indices_start[0]:indices_end[0]+1, indices_start[1]                                   :indices_end[1]+1, indices_start[2]:indices_end[2]+1] = 0  # 0 denotes obstacles
                    # self.map_array[int(self.map_array_scale*(x_start+0.1)):int(self.map_array_scale*(x_end+0.1))+1, int(self.map_array_scale*(y_start+5.1)):int(
                    #     self.map_array_scale*(y_end+5.1))+1, int(self.map_array_scale*(z_start+0.1)):int(self.map_array_scale*(z_end+0.1)+1)] = 0  # 0 denotes obstacles
                    # Create the block with bloated dimensions and adjusted location
                    bpy.ops.mesh.primitive_cube_add(
                        size=1, location=bloated_location)
                    cube = bpy.context.object
                    cube.dimensions = bloated_dimensions

                    # Color the block
                    mat_name = f"Material_{color[0]}_{color[1]}_{color[2]}"
                    mat = bpy.data.materials.get(
                        mat_name) or bpy.data.materials.new(name=mat_name)
                    mat.diffuse_color = (color[0], color[1], color[2], 1)
                    cube.data.materials.append(mat)
        # a,b,c = 18.5,2.5,2
        # indices = convrule(a, b, c, boundary[0], boundary[1], boundary[2], self.map_array_scale, bloat_amount)
        # print(self.map_array[indices[0]][indices[1]][indices[2]])
        # print("point in map array: ", self.map_array[100, 210, 35])
        # print("front idk: ", self.map_array[100, 200, 35])
        # print("bottom idk: ", self.map_array[0, 200, 10])
        # print("bottom back idk: ", self.map_array[0, 210, 10])

    # def add_sphere(self, location, color, radius=0.3):
    #     # Adjust the location to Blender's coordinate system (if needed, not adjusting in this case)
    #     adjusted_location = (location[0], location[1], location[2])

    #     # Create the sphere with specified location and radius
    #     bpy.ops.mesh.primitive_uv_sphere_add(
    #         radius=radius, location=adjusted_location)
    #     sphere = bpy.context.object

    #     # Color the sphere
    #     mat_name = f"Material_{color[0]}_{color[1]}_{color[2]}"
    #     mat = bpy.data.materials.get(
    #         mat_name) or bpy.data.materials.new(name=mat_name)
    #     mat.diffuse_color = color
    #     sphere.data.materials.append(mat)

    # def visualize_nodes(self, path):
    #     # Define nodes and their colors
    #     start_node = (
    #         path[0].x,
    #         path[0].y,
    #         path[0].z
    #     )
    #     start_color = (1, 0, 0, 1)  # Red
    #     self.add_sphere(start_node, start_color)

    #     # Traverse through the intermediate nodes
    #     for i in range(1, len(path) - 1):
    #         # Get the actual Node object from the path list
    #         intermediate_node = path[i]
    #         intermediate = (
    #             intermediate_node.x,
    #             intermediate_node.y,
    #             intermediate_node.z
    #         )
    #         intermediate_color = (0, 1, 0, 1)  # Green
    #         self.add_sphere(intermediate, intermediate_color)

    #     # Define the goal node and its color
    #     goal_node = (
    #         path[-1].x,
    #         path[-1].y,
    #         path[-1].z
    #     )
    #     goal_color = (0, 0, 1, 1)  # Blue
    #     self.add_sphere(goal_node, goal_color)

    def add_sphere(self, location, color, radius=0.15):  # radius=0.6
        # Adjust the location to Blender's coordinate system (if needed, not adjusting in this case)
        adjusted_location = (location[0], location[1], location[2])

        # Create the sphere with specified location and radius
        bpy.ops.mesh.primitive_uv_sphere_add(
            radius=radius, location=adjusted_location)
        sphere = bpy.context.object

        # Color the sphere
        mat_name = f"Material_{color[0]}_{color[1]}_{color[2]}"
        mat = bpy.data.materials.get(
            mat_name) or bpy.data.materials.new(name=mat_name)
        mat.diffuse_color = color
        sphere.data.materials.append(mat)

    def visualize_nodes(self, path):
        # Define nodes and their colors
        start_node = (
            path[0].x,
            path[0].y,
            path[0].z
        )
        start_color = (1, 0, 0, 1)  # Red
        self.add_sphere(start_node, start_color)
        prev_node = (start_node[0], start_node[1], start_node[2])

        # Traverse through the intermediate nodes
        for i in range(1, len(path) - 1):
            # Get the actual Node object from the path list
            intermediate_node = path[i]
            intermediate = (
                intermediate_node.x,
                intermediate_node.y,
                intermediate_node.z
            )
            intermediate_color = (0, 1, 0, 1)  # Green
            self.add_sphere(intermediate, intermediate_color)
            # self.connect_nodes_with_cylinder(intermediate,prev_node)
            prev_node = intermediate

        # Define the goal node and its color
        goal_node = (
            path[-1].x,
            path[-1].y,
            path[-1].z
        )
        goal_color = (0, 0, 1, 1)  # Blue
        self.add_sphere(goal_node, goal_color)
        # self.connect_nodes_with_cylinder(goal_node,prev_node)

    def get_cylinder_name(self, node1, node2):
        return f"cylinder_{node1[0]}_{node1[1]}_{node1[2]}_to_{node2[0]}_{node2[1]}_{node2[2]}"

    def connect_nodes_with_cylinder(self, node1, node2):
        location, rotation_quaternion, scale, depth = self.compute_cylinder_transform(
            node1, node2)
        bpy.ops.mesh.primitive_cylinder_add(
            radius=scale[0], depth=depth, location=location, rotation=rotation_quaternion.to_euler())
        cylinder = bpy.context.object
        cyl_name = self.get_cylinder_name(node1, node2)
        cylinder.name = cyl_name
        cylinder_color = (1, 1, 0, 1)  # Yellow
        mat_name = f"Cylinder_Material_{cylinder_color[0]}_{cylinder_color[1]}_{cylinder_color[2]}"
        mat = bpy.data.materials.get(
            mat_name) or bpy.data.materials.new(name=mat_name)
        mat.diffuse_color = cylinder_color
        cylinder.data.materials.append(mat)

    def compute_cylinder_transform(self, node1, node2):
        point_start = [node1[0], node1[1], node1[2]]
        point_end = [node2[0], node2[1], node2[2]]

        location = [
            (point_start[0] + point_end[0]) / 2,
            (point_start[1] + point_end[1]) / 2,
            (point_start[2] + point_end[2]) / 2,
        ]

        dir_vector = [
            point_end[0] - point_start[0],
            point_end[1] - point_start[1],
            point_end[2] - point_start[2],
        ]

        length = np.linalg.norm(dir_vector)
        if length != 0:
            dir_vector = [x / length for x in dir_vector]

        axis = [0, 0, 1]  # Default axis for quaternion calculation
        # to avoid values slightly out of range due to floating-point errors
        angle = math.acos(max(-1.0, min(1.0, dir_vector[2])))

        # if dir_vector is not parallel to z-axis
        if (dir_vector[0], dir_vector[1]) != (0, 0):
            # perpendicular in the xy-plane
            axis = [-dir_vector[1], dir_vector[0], 0]

        rotation_quaternion = mathutils.Quaternion(axis, angle)

        radius = 0.5  # Adjust the radius according to your need
        # Here, scale in z should be the length
        scale = [radius, radius, length]

        return location, rotation_quaternion, scale, length

    def get_map_array(self):
        return self.map_array

    def get_map_array_scale(self):
        return self.map_array_scale

    def get_bloat_amount(self):
        return self.bloat_amount

    def get_lower_boundary(self):
        return self.lowerboundary

    def get_upper_boundary(self):
        return self.upperboundary


# # Usage
# filepath = 'path_to_your_file.txt'
# bloat_amount = 0.1  # Adjust as needed
# env = Environment(filepath)
# env.make_env(bloat_amount)

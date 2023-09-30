import bpy
import numpy as np 

class Environment:
    def __init__(self, filepath) -> None:
        self.filepath = filepath
        self.map_array = None 
        self.map_array_scale = 50 

    def make_env(self, bloat_amount=0.1):
        try:
            # Read the file content
            with open(self.filepath, 'r') as f:
                content = f.read()
        except FileNotFoundError:
            # print("Error: File not found!")
            return
        
        # Process the content
        lines = content.split('\n')
        
        for line in lines:
            words = line.split()
            if words:
                if words[0] == 'boundary':
                    boundary = [float(i) for i in words[1:7]]
                    
                    # Create the boundary cube
                    bpy.ops.mesh.primitive_cube_add(size=1, location=((boundary[0] + boundary[3]) / 2, 
                                                (boundary[1] + boundary[4]) / 2, 
                                                (boundary[2] + boundary[5]) / 2))

                    boundary_cube = bpy.context.object
                    boundary_cube.dimensions = [boundary[3] - boundary[0], boundary[4] - boundary[1], boundary[5] - boundary[2]]
                    
                    # Subdivide the cube to create a grid effect
                    bpy.ops.object.mode_set(mode='EDIT')
                    bpy.ops.mesh.subdivide(number_cuts=10)
                    bpy.ops.object.mode_set(mode='OBJECT')
                    
                    # Shader setup for black color with alpha transparency of 0.344
                    mat = bpy.data.materials.new(name="Grid_Material")
                    mat.use_nodes = True
                    nodes = mat.node_tree.nodes
                    bsdf = nodes["Principled BSDF"]
                    bsdf.inputs[0].default_value = (0, 0, 0, 0.344)
                    
                    # Set blend mode to Alpha Blend
                    mat.blend_method = 'BLEND'
                    boundary_cube.data.materials.append(mat)

                    max_x, max_y, max_z = int(boundary[3]), int(boundary[4]), int(boundary[5])
                    self.map_array = np.ones((self.map_array_scale*max_x, self.map_array_scale*max_y, self.map_array_scale*max_z), dtype=np.uint8)  # 1 denotes free space
                    # print("boundary indices: ",self.map_array_scale*max_x, self.map_array_scale*max_y, self.map_array_scale*max_z)
                    
                elif words[0] == 'block':
                    block_coords = [float(i) for i in words[1:7]]
                    color = [int(i)/255 for i in words[7:10]]

                    # x_start, y_start, z_start, x_end, y_end, z_end = map(int, block_coords)
                    # self.map_array[50*x_start:50*x_end, 50*y_start:50*y_end, 50*z_start:50*z_end] = 0  # 0 denotes obstacles
                    
                    # Calculate the bloated dimensions
                    bloated_dimensions = [
                        (block_coords[3] - block_coords[0]) + 2 * bloat_amount,
                        (block_coords[4] - block_coords[1]) + 2 * bloat_amount,
                        (block_coords[5] - block_coords[2]) + 2 * bloat_amount,
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

                    x_start, y_start, z_start, x_end, y_end, z_end = map(int, bloated_coords)
                    self.map_array[self.map_array_scale*x_start:self.map_array_scale*x_end, self.map_array_scale*y_start:self.map_array_scale*y_end, self.map_array_scale*z_start:self.map_array_scale*z_end] = 0  # 0 denotes obstacles

                    # print("block indices: ",self.map_array_scale*x_start,self.map_array_scale*x_end, self.map_array_scale*y_start,self.map_array_scale*y_end, self.map_array_scale*z_start,self.map_array_scale*z_end)

                    # Create the block with bloated dimensions and adjusted location
                    bpy.ops.mesh.primitive_cube_add(size=1, location=bloated_location)
                    cube = bpy.context.object
                    cube.dimensions = bloated_dimensions
                    
                    # Color the block
                    mat_name = f"Material_{color[0]}_{color[1]}_{color[2]}"
                    mat = bpy.data.materials.get(mat_name) or bpy.data.materials.new(name=mat_name)
                    mat.diffuse_color = (color[0], color[1], color[2], 1)
                    cube.data.materials.append(mat)


    def add_sphere(self, location, color, radius=0.3):
        # Adjust the location to Blender's coordinate system (if needed, not adjusting in this case)
        adjusted_location = (location[0], location[1], location[2])
        
        # Create the sphere with specified location and radius
        bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=adjusted_location)
        sphere = bpy.context.object
        
        # Color the sphere
        mat_name = f"Material_{color[0]}_{color[1]}_{color[2]}"
        mat = bpy.data.materials.get(mat_name) or bpy.data.materials.new(name=mat_name)
        mat.diffuse_color = color
        sphere.data.materials.append(mat)


    def visualize_nodes(self, path):
        # Define nodes and their colors
        start_node = (
            path[0].x / self.map_array_scale, 
            path[0].y / self.map_array_scale, 
            path[0].z / self.map_array_scale
        )
        start_color = (1, 0, 0, 1)  # Red
        self.add_sphere(start_node, start_color)

        # Traverse through the intermediate nodes
        for i in range(1, len(path) - 1):
            intermediate_node = path[i]  # Get the actual Node object from the path list
            intermediate = (
                intermediate_node.x / self.map_array_scale, 
                intermediate_node.y / self.map_array_scale, 
                intermediate_node.z / self.map_array_scale
            )
            intermediate_color = (0, 1, 0, 1)  # Green
            self.add_sphere(intermediate, intermediate_color)

        # Define the goal node and its color
        goal_node = (
            path[-1].x / self.map_array_scale, 
            path[-1].y / self.map_array_scale, 
            path[-1].z / self.map_array_scale
        )
        goal_color = (0, 0, 1, 1)  # Blue
        self.add_sphere(goal_node, goal_color)


    def get_map_array(self):
        return self.map_array
    
    def get_map_array_scale(self):
        return self.map_array_scale
                    

# # Usage
# filepath = 'path_to_your_file.txt'
# bloat_amount = 0.1  # Adjust as needed
# env = Environment(filepath)
# env.make_env(bloat_amount)

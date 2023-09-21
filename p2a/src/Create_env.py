import bpy

def create_cube(location, dimensions, color, is_boundary=False):
    # Define the mesh and object
    mesh = bpy.data.meshes.new(name="Cube")
    obj = bpy.data.objects.new("Cube", mesh)

    # Set the object's location
    obj.location = location
    bpy.context.collection.objects.link(obj)

    # Create the cube
    bpy.context.view_layer.objects.active = obj
    mesh = bpy.context.object.data
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    bpy.ops.transform.resize(value=dimensions)

    # Set the object's color
    mat = bpy.data.materials.new(name="Material")
    mat.diffuse_color = color
    obj.data.materials.append(mat)

    # Set wireframe mode for boundary
    if is_boundary:
        obj.display_type = 'WIRE'
    bpy.ops.object.mode_set(mode='OBJECT')

def display_map(map_text):
    lines = map_text.strip().split("\n")
    for line in lines:
        tokens = line.split()
        if tokens[0] == 'boundary':
            xmin, ymin, zmin, xmax, ymax, zmax = map(float, tokens[1:])
            location = ((xmax + xmin) / 2, (ymax + ymin) / 2, (zmax + zmin) / 2)
            dimensions = ((xmax - xmin) / 2, (ymax - ymin) / 2, (zmax - zmin) / 2)
            create_cube(location, dimensions, (1.0, 1.0, 1.0, 1.0), is_boundary=True)
        elif tokens[0] == 'block':
            xmin, ymin, zmin, xmax, ymax, zmax = map(float, tokens[1:7])
            r, g, b = map(float, tokens[7:10])
            location = ((xmax + xmin) / 2, (ymax + ymin) / 2, (zmax + zmin) / 2)
            dimensions = ((xmax - xmin) / 2, (ymax - ymin) / 2, (zmax - zmin) / 2)
            color = (r / 255, g / 255, b / 255, 1.0)
            create_cube(location, dimensions, color)


map_text = """
boundary 0 0 0 45 35 6
block 1.0 1.0 1.0 3.0 3.0 3.0 0 255 0
block 20.0 10.0 0.0 21.0 20.0 6.0 0 0 255
"""

display_map(map_text)
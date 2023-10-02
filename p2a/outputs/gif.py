import os
import re
import imageio

# Set the directory where the images are located and the gif filename
directory = "/home/ankit/Downloads/Tree-Planning-Through-The-Trees/p2a/outputs"
gif_filename = "output.gif"

# Define a regex pattern to extract timestamp from the filenames
pattern = re.compile(r"(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})")

# Get a list of all the .png files in the directory
files = [f for f in os.listdir(directory) if f.endswith('.png')]

# Sort files by timestamp
files.sort(key=lambda x: pattern.findall(x)[0] if pattern.findall(x) else '')

# Create a gif from images
with imageio.get_writer(gif_filename, mode='I', duration = 0.01) as writer:
    for filename in files:
        image = imageio.imread(os.path.join(directory, filename))
        writer.append_data(image)
        
print(f"{gif_filename} has been created.")
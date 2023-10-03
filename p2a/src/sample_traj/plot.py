# import pandas as pd
# import matplotlib.pyplot as plt

# # Read the CSV file
# data = pd.read_csv('trajnew.csv', header=None)

# # Plot each row
# labels = ['posx', 'posy', 'posz', 'velx', 'vely', 'velz', 'accx', 'accy', 'accz']
# for idx, label in enumerate(labels):
#     plt.plot(data.iloc[idx], label=label)

# # Show the plot
# plt.legend()
# plt.title('CSV Data Plot')
# plt.xlabel('Index')
# plt.ylabel('Value')
# plt.show()

import pandas as pd
import matplotlib.pyplot as plt

# 1. Read the CSV file
data = pd.read_csv('trajnew.csv', header=None)

# 2. Extract the posx, posy, and posz columns
posx = data.iloc[0].values
posy = data.iloc[1].values
posz = data.iloc[2].values

# 3. Plot the data in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(posx, posy, posz, label='Position trajectory')
ax.set_xlabel('X position')
ax.set_ylabel('Y position')
ax.set_zlabel('Z position')
ax.legend()
plt.show()

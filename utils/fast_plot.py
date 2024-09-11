import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Example trajectory data

import yaml



DYNOBENCH_BASE =  "dynobench/"
problem_in = DYNOBENCH_BASE + "envs/multirobot/example/moving_obs_twd_start.yaml";
# problem_in = DYNOBENCH_BASE + "envs/multirobot/example/moving_obs_same_path.yaml";
file_in = "/tmp/moving_obs_sol3.yaml"
    # moving_obs_sol2.yaml"



with open(file_in, 'r') as stream:
    D = yaml.safe_load(stream)


with open(problem_in, 'r') as stream:
    Dproblem = yaml.safe_load(stream)


states = D['states']



# Extract the first three components
x = [state[0] for state in states]
y = [state[1] for state in states]
z = [state[2] for state in states]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, marker='o')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

import numpy as  np


max_range = np.array([
    Dproblem['environment']['max'][0] - Dproblem['environment']['min'][0],
    Dproblem['environment']['max'][1] - Dproblem['environment']['min'][1],
    Dproblem['environment']['max'][2] - Dproblem['environment']['min'][2]] ).max() / 2.0


mid_x = Dproblem['environment']['min'][0] + (Dproblem['environment']['max'][0] - Dproblem['environment']['min'][0]) * 0.5
mid_y = Dproblem['environment']['min'][1] + (Dproblem['environment']['max'][1] - Dproblem['environment']['min'][1]) * 0.5
mid_z = Dproblem['environment']['min'][2] + (Dproblem['environment']['max'][2] - Dproblem['environment']['min'][2]) * 0.5

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)



# Show plot
plt.show()

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initialization function for the animation
def init():
    ax.set_xlim(min(x), max(x))
    ax.set_ylim(min(y), max(y))
    ax.set_zlim(min(z), max(z))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return []

import numpy as np
# Create data for a sphere
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
radius = .5
sphere_x = radius * np.outer(np.cos(u), np.sin(v))
sphere_y = radius * np.outer(np.sin(u), np.sin(v))
sphere_z = radius * np.outer(np.ones(np.size(u)), np.cos(v))




# Update function for the animation
def update(frame):
    ax.cla()  # Clear the axes
    ax.plot(x[:frame+1], y[:frame+1], z[:frame+1], marker='o')
    ax.set_xlim(min(x), max(x))
    ax.set_ylim(min(y), max(y))
    ax.set_zlim(min(z), max(z))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    if frame == len( Dproblem['environment']['moving_obstacles'] ):
        frame = frame - 1

    center = Dproblem['environment']['moving_obstacles'][frame][0]['center']
    # np.array( [-1., 2.25, 1.3] )
    

    ax.plot_surface(sphere_x + center[0] , sphere_y + center[1] , sphere_z + center[2] , color='r', alpha=0.6)


    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


    return []

# Create the animation
ani = FuncAnimation(fig, update, frames=len(x), init_func=init, blit=False, interval=10)

# Show the animation
plt.show()


import yaml

data = {
    'environment': {
        'max': [2, 3.5, 2.5],
        'min': [-5.5, 0, 0],
    },
    'robots': [
        {
            'goal': [-4.0, 3.0, 2.1, 0, 0, 0],
            'start': [1.5, 1.5, 0.5, 0, 0, 0],
            'type': 'integrator2_3d_v0'
        }
    ]
}

num_steps = 200

# lets create 
moving_obs = []

import numpy as np


diff = np.array( [-4.0, 3.0, 2.1])  - np.array([1.5, 1.5, 0.5])
start = np.array([1.5, 1.5, 0.5 ])  + .1 * diff
goal = np.array([1.5, 1.5, 0.5])  + .9 * diff


# np.array([-1.0, 2.25, 1.3])
# goal = np.array([1.5, 1.5, 0.5])


for i in range(num_steps):
    moving_obs.append(
        [
        {
                 'center':   (start + (goal - start) * i / num_steps).tolist(),
                # [-1.0, 2.25, 1.3],
                 'size': [0.5],
                 'type': 'sphere'
        }
        ]
    )
 

data['environment']['moving_obstacles'] = moving_obs



with open('/tmp/moving_obs_same_path.yaml', 'w') as file:
    yaml.dump(data, file, default_flow_style=False)

# This code uses the yaml library to generate the specified YAML st

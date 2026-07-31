What's In an Observation
============================

An observation is returned by the gym environment after resetting and stepping. An observation is a dictionary with the following keys:

- ``'ego_idx'``: index of the ego agent in the list of agents
- ``'scans'``: list of length num_agents of numpy.ndarrays of (num_beams, ), each array is the corresponding laser scan of the agent in the list
- ``'scan_updated'``: list of booleans indicating whether a new LiDAR measurement became available in the current step
- ``'scan_times'``: list of measurement timestamps for the returned LiDAR scans
- ``'scan_ages'``: list of seconds since each returned LiDAR scan was measured
- ``'scan_update_counts'``: list of LiDAR measurement update counts since reset
- ``'poses_x'``: list of length num_agents of floats, each agent's x pose in the world
- ``'poses_y'``: list of length num_agents of floats, each agent's y pose in the world
- ``'poses_theta'``: list of length num_agents of floats, each agent's theta pose in the world
- ``'linear_vels_x'``: list of length num_agents of floats, each agent's current longitudinal velocity
- ``'linear_vels_y'``: list of length num_agents of zeros 
- ``'ang_vels_z'``: list of length num_agents of floats, each agent's current yaw rate
- ``'collisions'``: list of length num_agents of 1s or 0s, whether each agent is in collision with another agent or the environment

std_msgs/Header header

uint32 tick_count
float64 simulation_time  # Time passed since the beginning of the simulation (s)
float64 delta_time       # Time passed since last tick (s)

geoscenario_msgs/Vehicle[] vehicles
geoscenario_msgs/Pedestrian[] pedestrians

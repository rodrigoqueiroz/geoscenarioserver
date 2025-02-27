std_msgs/Header header

uint32 tick_count
float64 simulation_time         # [s] Time passed since the beginning of the simulation
float64 delta_time              # [s] Time passed since last tick
geographic_msgs/GeoPoint origin # [deg, deg, m], in WGS84 (EPSG: 4326)

geoscenario_msgs/Vehicle[] vehicles
geoscenario_msgs/Pedestrian[] pedestrians

std_msgs/Header header

uint32 tick_count
float64 simulation_time         # [s] Time passed since the beginning of the simulation
float64 delta_time              # [s] Time passed since last tick
geographic_msgs/GeoPoint origin # [deg, deg, m, m], in WGS84 (EPSG: 4326)
                                # origin=(0,0,0,0) indicates that WGS84 coordinates are used

geoscenario_msgs/Vehicle[] vehicles
geoscenario_msgs/Pedestrian[] pedestrians

uint16 id
string type

# State
geometry_msgs/Point position        # [m, m, m], in local cartesian frame defined by the element 'origin'
                                    # [deg, deg, m] in WGS84 when origin=(0,0,0)
geometry_msgs/Vector3 velocity      # [m/s, m/s, m/s] along X, Y, Z axes
geometry_msgs/Vector3 dimensions    # [m, m, m] along X, Y, Z axes
float64 yaw                         # [rad], CCW from east
float64 steering_angle              # [rad]
bool active                         # provided only by the client to the server

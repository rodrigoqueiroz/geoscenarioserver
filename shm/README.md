# Shared Memory

`SimSharedMemoryServer.py` defines a class that the server uses to manage shared memory.

`SimSharedMemoryClient.py` defines a class that can be used by client applications to read from the shared memory.
It purposely does not contain any GeoScenario dependencies.

## Shared Memory Layout

The shared memory written by the server is one large space-separated UTF8-encoded string with the following layout:

```
tick_count simulation_time delta_time n_vehicles n_pedestrians
origin_lat origin_lon origin_alt
vid v_type x y z vx vy yaw steering_angle
...
pid p_type x y z vx vy yaw
...
```

All data is in base SI units (m, m/s, s, radians), and position data is ENU with positive yaw indicating counter-clockwise from east.
The positions are in a local cartesian frame defined by (origin_lat, origin_lon, origin_alt) in WGS 84 (EPSG: 4326).

The shared memory written by a client is the same as the server's except the origin line.
The client can the local cartesian projector to obtain lat/lon/alt positions given the origin and the local coordinates.
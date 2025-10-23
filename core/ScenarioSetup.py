#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
# ---------------------------------------------
# GeoScenario Setup
# Build directly from code (instead of GeoScenario (.osm) file)
# --------------------------------------------

from types import LambdaType
try:
    from lanelet2.projection import LocalCartesianProjector
    use_local_cartesian=True
except ImportError:
    from lanelet2.projection import UtmProjector
    use_local_cartesian=False
from lanelet2.core import GPSPoint
from mapping.LaneletMap import *
from SimConfig import SimConfig
from core.SimTraffic import SimTraffic
from core.TrafficLight import TrafficLight as TL
from core.TrafficLight import TrafficLightType, TrafficLightColor
from sv.Vehicle import *
from sp.Pedestrian import *
from gsc.GSParser import GSParser
from core.Actor import *

def extract_tag(vnode, name, default_value, parser_fn):
    return parser_fn(vnode.tags[name]) if name in vnode.tags else default_value

def extract_bool_tag(vnode, name, default_value=False):
    if name in vnode.tags:
        val = vnode.tags[name].strip().lower()
        if val in ['yes', 'true', 'on']:
            return True
        elif val in ['no', 'false', 'off']:
            return False
        else:
            return default_value

def load_geoscenario_from_file(gsfiles, sim_traffic:SimTraffic, sim_config:SimConfig, lanelet_map:LaneletMap, map_path, btree_locations):
    """ Setup scenario from GeoScenario file
    """
    full_scenario_paths = []
    for gsfile in gsfiles:
        if (os.path.isabs(gsfile[0])):
            # absolute path
            full_scenario_paths.append(gsfile)
        else:
            # relative to ROOT_DIR
            full_scenario_paths.append(os.path.join(ROOT_DIR, gsfile))

    log.info("Loading GeoScenario file(s): {}".format(", ".join(full_scenario_paths)))

    #========= Parse GeoScenario File
    parser = GSParser()
    if not parser.load_and_validate_geoscenario(full_scenario_paths):
        log.error("Error loading GeoScenario file(s)")
        return False
    if parser.globalconfig.tags['version'] < 2.0:
        log.error("GSServer requires GeoScenario 2.0 or newer")
        return False

    #========= Scenario config
    sim_config.scenario_name = parser.globalconfig.tags['name']
    sim_config.timeout = parser.globalconfig.tags['timeout']
    if 'plotid' in parser.globalconfig.tags:
        sim_config.plot_id = parser.globalconfig.tags['plotid']

    #========= Map
    if map_path == "":
        map_file = os.path.join(ROOT_DIR, 'scenarios', parser.globalconfig.tags['lanelet']) #use default map path
    else:
        map_file = os.path.join(map_path, parser.globalconfig.tags['lanelet']) #use parameter map path
    # use origin from gsc file to project nodes to sim frame
    altitude  = parser.origin.tags['altitude'] if 'altitude' in parser.origin.tags else 0.0
    area = parser.origin.tags['area'] if 'area' in parser.origin.tags else MPLOT_SIZE
    # preserve the origin
    sim_traffic.set_origin(parser.origin.lat, parser.origin.lon, altitude, area)
    if use_local_cartesian:
        projector = LocalCartesianProjector(lanelet2.io.Origin(parser.origin.lat, parser.origin.lon, altitude))
        log.info("Using LocalCartesianProjector")
    else:
        projector = UtmProjector(lanelet2.io.Origin(parser.origin.lat, parser.origin.lon, altitude))
        log.info("Using UTMProjector")

    parser.project_nodes(projector,altitude)
    lanelet_map.load_lanelet_map(map_file, projector)
    sim_config.map_name = parser.globalconfig.tags['lanelet']

    #========= Traffic lights
    for name, tnode in parser.tlights.items():
        tltype = tnode.tags['type'] if 'type' in tnode.tags else 'default'
        states = list(map(TrafficLightColor.from_str, tnode.tags['states'].split(',')))
        durations = list(map(float, str(tnode.tags['duration']).split(',')))
        if tltype == 'left':
            tl_type = TrafficLightType.left
        elif tltype == 'right':
            tl_type = TrafficLightType.right
        elif tltype == 'pedestrian':
            tl_type = TrafficLightType.pedestrian
        else:
            tl_type = TrafficLightType.default
        #name must match a traffic light in the lanelet map with tag 'name'
        tl = TL(name, states, durations, tl_type = tl_type)
        sim_traffic.add_traffic_light(tl)

    #========= crosswalks
    sim_traffic.crosswalks = lanelet_map.get_crosswalks()

    #=========  Ego (External Vehicle)
    if parser.egostart is not None:
        ego_vehicle = EV(99, 'Ego', [0.0,0.0,0.0, 0.0,0.0,0.0])
        sim_traffic.add_vehicle(ego_vehicle)

    #========= Vehicles
    log.debug("Scenario Setup, initializing vehicles:")
    for vid, vnode in parser.vehicles.items():
        if len(sim_traffic.vehicles) >= MAX_NVEHICLES:
            break
        vid = int(vid)   #<= must ne integer
        name = vnode.tags['name']   #vehicle name
        model = extract_tag(vnode, 'model', '', str)
        start_state = [vnode.x,0.0,0.0,vnode.y,0.0,0.0]
        start_in_frenet = False
        length = extract_tag(vnode, 'length', VEHICLE_LENGTH, float)
        width = extract_tag(vnode, 'width', VEHICLE_WIDTH, float)

        #yaw = 90.0
        #if 'yaw' in vnode.tags:
        #    yaw = (float(vnode.tags['yaw']) + 90.0) % 360.0
        #    # Place yaw in the range [-180.0, 180.0)
        #    if yaw < -180.0:
        #        yaw += 360.0
        #    elif yaw >= 180.0:
        #        yaw -= 360.0
        #print(yaw)

        btype = extract_tag(vnode, 'btype', '', str).lower()
        goal_ends_simulation = extract_bool_tag(vnode, 'goal_ends_simulation', False)
        rule_engine_port = extract_tag(vnode, 'rule_engine_port', None, int)
        yaw = -extract_tag(vnode, 'yaw', 0.0, float)

        log.debug(f"Initializing vehicle VID:{vid} with behavior type: {btype}...")

        #SDV Model (dynamic vehicle)
        if btype == 'sdv':
            #start state
            if 'start_cartesian' in vnode.tags:
                start_cartesian = vnode.tags['start_cartesian']
                gs_sc = start_cartesian.split(',')
                log.debug(gs_sc)
                if len (gs_sc) != 4:
                    log.error("start state in Cartesian must have 4 values [x_vel,x_acc,y_vel,y_acc].")
                    continue
                x = vnode.x
                y = vnode.y
                x_vel = float(gs_sc[0].strip())
                x_acc = float(gs_sc[1].strip())
                y_vel = float(gs_sc[2].strip())
                y_acc = float(gs_sc[3].strip())
                start_state = [x,x_vel,x_acc,y,y_vel,y_acc]     #vehicle start state in cartesian frame
                log.debug(start_state)

            if 'start_frenet' in vnode.tags:
                # assume frenet start_state is relative to the first lane of the route
                start_in_frenet = True
                start_frenet = vnode.tags['start_frenet']
                gs_sf = start_frenet.split(',')
                if len (gs_sf) != 6:
                    log.error("start state in Frenet must have 6 values [s,s_vel,s_acc,d,d_vel,d_acc].")
                    continue
                s = float(gs_sf[0])
                s_vel = float(gs_sf[1].strip())
                s_acc = float(gs_sf[2].strip())
                d = float(gs_sf[3])
                d_vel = float(gs_sf[4].strip())
                d_acc = float(gs_sf[5].strip())
                start_state = [s,s_vel,s_acc,d,d_vel,d_acc]     #vehicle start state in frenet frame
                #print(start_state)

            #route
            if 'route' not in vnode.tags:
                log.error("SDV {} requires a route .".format(vid))
                continue
            gs_route = vnode.tags['route']
            route_nodes = [ TrajNode(x = node.x, y = node.y) for node in parser.routes[gs_route].nodes ]
            route_nodes.insert(0, TrajNode(x=vnode.x,y=vnode.y)) #insert vehicle location as start of route
            #btree
            #a behavior tree file (.btree) inside the btype's folder, defaulted in btrees
            root_btree_name = vnode.tags['btree'] if 'btree' in vnode.tags else "st_standard_driver.btree"
            try:
                vehicle = SDV(  vid, name, root_btree_name, start_state, yaw,
                                lanelet_map, route_nodes,
                                start_state_in_frenet=start_in_frenet,
                                btree_locations=btree_locations,
                                btype=btype, goal_ends_simulation=goal_ends_simulation,
                                rule_engine_port=rule_engine_port,
                                length=length, width=width
                            )
                #vehicle = SDV(  vid, name, root_btree_name, start_state, yaw,
                #                lanelet_map, sim_config.lanelet_routes[vid],
                #                route_nodes,
                #                start_state_in_frenet=start_in_frenet,
                #                btree_locations=btree_locations,
                #                btype=btype
                #            )
                #reconfig btree
                if 'btconfig' in vnode.tags:
                    vehicle.btree_reconfig = vnode.tags['btconfig']
                vehicle.model = model
                sim_traffic.add_vehicle(vehicle)
                log.info("Vehicle {} initialized with SDV behavior".format(vid))
            except Exception as e:
                log.error("Failed to initialize vehicle {}".format(vid))
                raise e

        # Trajectory Vehicle (TV)
        elif btype == 'tv':

            if 'trajectory' not in vnode.tags:
                log.error("TV {} requires a trajectory .".format(vid))
                continue
            try:
                t_name = vnode.tags['trajectory']
                t_nodes = parser.trajectories[t_name].nodes
                trajectory = []
                prev_node = None
                for node in t_nodes:
                    nd = TrajNode()
                    nd.x = float(node.x)
                    nd.y = float(node.y)
                    nd.time = float(node.tags['time'])
                    if prev_node is not None:
                        dt = nd.time - prev_node.time
                        nd.x_vel = (nd.x - prev_node.x) / dt
                        nd.y_vel = (nd.y - prev_node.y) / dt
                        # the first node has unknown velocity, assume the same as the second node's
                        if prev_node.x_vel is None or prev_node.y_vel is None:
                            prev_node.x_vel = nd.x_vel
                            prev_node.y_vel = nd.y_vel
                    nd.speed = float(node.tags['speed']) if ('speed' in node.tags) else None
                    nd.yaw = float(node.tags['yaw']) if ('yaw' in node.tags) else None
                    trajectory.append(nd)
                    prev_node = nd
                vehicle = TV(vid = vid,                                     #<= must ne integer
                            name = name,                                    #vehicle name
                            start_state = start_state,                      #vehicle start state in cartesian frame [x,x_vel,x_acc, y,y_vel,y_acc]
                            yaw = yaw,
                            trajectory = trajectory,                        #a valid trajectory with at least x,y,time per node
                            length=length, width=width)
                sim_traffic.add_vehicle(vehicle)
                log.info("Vehicle {} initialized with TV behavior".format(vid))

            except Exception as e:
                log.error("Failed to initialize vehicle {}".format(vid))
                raise e

        # Path Vehicle (PV)
        elif btype == 'pv':
            if 'path' not in vnode.tags:
                log.error(f"PV {vid} requires a path")
                continue
        
            p_name = vnode.tags['path']
            p_nodes = parser.paths[p_name].nodes
            set_speed = extract_tag(vnode, "speed", None, float)
            speed_qualifier = extract_tag(vnode, 'speed_qualifier', None, str)
            if speed_qualifier is not None:
                try:
                    speed_qualifier = SpeedQualifier[speed_qualifier.upper()]
                except KeyError:
                    log.warning(f"Path vehicle {vid} has invalid speed_qualifier '{speed_qualifier}'. Using INITIAL.")
                    speed_qualifier = SpeedQualifier.INITIAL
            else:
                speed_qualifier = SpeedQualifier.INITIAL
            collision_vid = extract_tag(vnode, "collision_vehicle_vid", None, int)
            use_speed_profile = extract_bool_tag(vnode, "usespeedprofile", False)
            collision_point = None
            
            path = []
            path_length = 0.0

            for i in range(len(p_nodes)):
            # for node in p_nodes:
                if (i > 0):
                    path_length += math.hypot(p_nodes[i].x - p_nodes[i-1].x, p_nodes[i].y - p_nodes[i-1].y)
                nd = PathNode()
                nd.x = float(p_nodes[i].x)
                nd.y = float(p_nodes[i].y)
                nd.s = path_length
                # Convert from km/h to m/s
                nd.accel = extract_tag(p_nodes[i], 'agentacceleration', None, float)
                nd.time_to_accel = extract_tag(p_nodes[i], 'timetoacceleration', None, float)
                nd.speed = float(p_nodes[i].tags['agentspeed'] / 3.6) if ('agentspeed' in p_nodes[i].tags) else None
                path.append(nd)

                if extract_bool_tag(p_nodes[i], "collision_pt", False):
                    collision_point = PathNode()
                    collision_point.x = float(p_nodes[i].x)
                    collision_point.y = float(p_nodes[i].y)
                    collision_point.s = path_length


            # Set initial longitudinal velocity, path always takes precedence
            frenet_state = [0.0,0.0,0.0, 0.0,0.0,0.0]
            if path[0].speed is not None:
                frenet_state[1] = path[0].speed / 3.6
            elif 'speed' in vnode.tags:
                frenet_state[1] = float(vnode.tags['speed']) / 3.6
            else:
                log.error("PV {} has no initial speed".format(vid))
                continue
            
            vehicle = PV(vid, name, start_state, frenet_state, yaw, path, sim_traffic.debug_shdata, sim_traffic.vehicles, length=length, width=width, set_speed=set_speed, speed_qualifier=speed_qualifier, collision_vid=collision_vid, collision_point=collision_point, use_speed_profile=use_speed_profile)
            vehicle.model = model
            sim_traffic.add_vehicle(vehicle)
            log.info(f"Vehicle {vid} initialized with PV behavior")
            continue

        # External Vehicle (EV)
        elif btype == 'ev':
            bsource = extract_tag(vnode, 'bsource', 'co-simulator', str)
            vehicle = EV(vid, name, start_state, yaw, bsource)
            vehicle.model = model
            sim_traffic.add_vehicle(vehicle)
            log.info(f"Vehicle {vid} initialized as an external vehicle")

        # Neutral Vehicle
        else:
            vehicle = Vehicle(vid, name, start_state, yaw=yaw)
            vehicle.model = model
            sim_traffic.add_vehicle(vehicle)
            log.info(f"Vehicle {vid} initialized as a motionless vehicle")
        #=========

    #========= Pedestrians
    if len(parser.pedestrians.items()) > 0:
        log.debug("Scenario Setup, initializing pedestrians:")
    for pid, pnode in parser.pedestrians.items():
        pid = int(pid)
        name = pnode.tags['name']
        start_state = [pnode.x,0.0,0.0, pnode.y,0.0,0.0]                    #start state in cartesian frame [x,x_vel,x_acc, y,y_vel,y_acc]
        yaw = 90.0
        if 'yaw' in pnode.tags:
            yaw = (float(pnode.tags['yaw']) + 90.0) % 360.0
            # Place yaw in the range [-180.0, 180.0)
            if yaw < -180.0:
                yaw += 360.0
            elif yaw >= 180.0:
                yaw -= 360.0

        btype = pnode.tags['btype'].lower() if 'btype' in pnode.tags else ''
        length = extract_tag(pnode, 'length', PEDESTRIAN_LENGTH, float)
        width = extract_tag(pnode, 'width', PEDESTRIAN_WIDTH, float)

        # Trajectory Pedestrian (TP)
        if btype == 'tp':
            if 'trajectory' not in pnode.tags:
                log.error("PV {} requires a trajectory .".format(pid))
                continue
            try:
                t_name = pnode.tags['trajectory']
                t_nodes = parser.trajectories[t_name].nodes
                trajectory = []     #a valid trajectory with at least x,y,time per node
                prev_node = None
                for node in t_nodes:
                    nd = TrajNode()
                    nd.time = float(node.tags['time'])
                    nd.x = float(node.x)
                    nd.y = float(node.y)
                    if prev_node is not None:
                        dt = nd.time - prev_node.time
                        nd.x_vel = (nd.x - prev_node.x) / dt
                        nd.y_vel = (nd.y - prev_node.y) / dt
                        # the first node has unknown velocity, assume the same as the second node's
                        if prev_node.x_vel is None or prev_node.y_vel is None:
                            prev_node.x_vel = nd.x_vel
                            prev_node.y_vel = nd.y_vel
                    nd.yaw = float(node.tags['yaw']) if 'yaw' in node.tags else None
                    nd.speed = float(node.tags['speed']) if 'speed' in node.tags else None
                    trajectory.append(nd)
                    prev_node = nd
                pedestrian = TP(pid, name, start_state, yaw, trajectory, length=length, width=width)
                sim_traffic.add_pedestrian(pedestrian)
                log.info("Pedestrian {} initialized with TP behavior".format(pid))
            except Exception as e:
                log.error("Failed to initialize pedestrian {}".format(pid))
                raise e

        # Simulated pedestrian
        elif btype == 'sp':
            if 'destination' not in pnode.tags and 'route' not in pnode.tags:
                log.error("SP {} requires either a destination or route.".format(pid))
                continue
            try:
                if 'route' in pnode.tags:
                    p_route = pnode.tags['route']
                    route_nodes = parser.routes[p_route].nodes
                    sim_config.pedestrian_goal_points[pid] = [(node.x, node.y) for node in route_nodes]
                    #pedestrian_lanelet_routes = [lanelet_map.get_occupying_lanelet_by_participant(node.x, node.y, "pedestrian") for node in route_nodes]   # a valid lanelet route
                    #sim_config.lanelet_routes[pid] = lanelet_map.get_route_via(pedestrian_lanelet_routes)
                else:
                    p_dest = pnode.tags['destination']
                    dest_node = parser.locations[p_dest]
                    sim_config.pedestrian_goal_points[pid] = [(dest_node.x, dest_node.y)]

                root_btree_name = pnode.tags['btree'] if 'btree' in pnode.tags else "walk.btree" # a behavior tree file (.btree) inside btrees/sp

                pedestrian = SP(pid,
                                name,
                                start_state,
                                yaw,
                                list(sim_config.pedestrian_goal_points[pid]),
                                root_btree_name,
                                btree_locations=btree_locations,
                                btype=btype,
                                length=length, width=width)

                sim_traffic.add_pedestrian(pedestrian)
                log.info("Pedestrian {} initialized with SP behavior".format(pid))
            except Exception as e:
                log.error("Failed to initialize pedestrian {}".format(pid))
                raise e
            
        # Path pedestrian    
        else:
            if 'path' not in pnode.tags:
                log.error(f"Path Pedestrian {pid} requires a path")
                continue
            
            p_name = pnode.tags['path']
            p_nodes = parser.paths[p_name].nodes

            set_speed = extract_tag(pnode, "speed", None, float)
            speed_qualifier = extract_tag(pnode, 'speed_qualifier', None, str)
            if speed_qualifier is not None:
                try:
                    speed_qualifier = SpeedQualifier[speed_qualifier.upper()]
                except KeyError:
                    log.warning(f"Pedestrian {pid} has invalid speed_qualifier '{speed_qualifier}'. Using INITIAL.")
                    speed_qualifier = SpeedQualifier.INITIAL
            else:
                speed_qualifier = SpeedQualifier.INITIAL
            collision_vid = extract_tag(pnode, 'collision_vehicle_vid', None, int)
            use_speed_profile = extract_bool_tag(pnode, "usespeedprofile", False)
            collision_point = None

            path = []
            path_length = 0.0

            for i in range(len(p_nodes)):
            # for node in p_nodes:
                if (i > 0):
                    path_length += math.hypot(p_nodes[i].x - p_nodes[i-1].x, p_nodes[i].y - p_nodes[i-1].y)
                nd = PathNode()
                nd.x = float(p_nodes[i].x)
                nd.y = float(p_nodes[i].y)
                nd.s = path_length
                # Convert from km/h to m/s
                nd.accel = extract_tag(p_nodes[i], 'agentacceleration', None, float)
                nd.time_to_accel = extract_tag(p_nodes[i], 'timetoacceleration', None, float)
                nd.speed = float(p_nodes[i].tags['agentspeed'] / 3.6) if ('agentspeed' in p_nodes[i].tags) else None
                path.append(nd)

                if extract_bool_tag(p_nodes[i], "collision_pt", False):
                    collision_point = PathNode()
                    collision_point.x = float(p_nodes[i].x)
                    collision_point.y = float(p_nodes[i].y)
                    collision_point.s = path_length

            # Set initial longitudinal velocity, path always takes precedence
            frenet_state = [0.0,0.0,0.0, 0.0,0.0,0.0]
            if path[0].speed is not None:
                frenet_state[1] = path[0].speed / 3.6
            elif 'speed' in pnode.tags:
                frenet_state[1] = float(pnode.tags['speed']) / 3.6
            else:
                log.error(f"Path Pedestrian {pid} has no initial speed")
                continue

            pedestrian = PP(
                pid,
                p_name,
                start_state=start_state,
                frenet_state=frenet_state,
                yaw=yaw,
                path=path,
                scenario_vehicles=sim_traffic.vehicles,
                debug_shdata=sim_traffic.debug_shdata,
                set_speed=set_speed,
                reference_speed=frenet_state[1],
                speed_qualifier=speed_qualifier,
                collision_vid=collision_vid,
                collision_point=collision_point,
                use_speed_profile=use_speed_profile
            )
            sim_traffic.add_pedestrian(pedestrian)
            log.info(f"Pedestrian {pid} initialized with PP behavior")
            continue

    #========= Static Objects
    #Area based objetics are not supported yet.
    for oid, onode in parser.staticobjects.items():
        sim_traffic.add_static_object(oid, onode.x, onode.y )

    #Finished
    return True

def load_geoscenario_from_code(scenario_name:str, sim_traffic:SimTraffic, sim_config:SimConfig, lanelet_map:LaneletMap):
    """ Setup scenario directly from code
        Add more entries as more scenarios are added
    """
    log.info("Loading GeoScenario from code: {}".format(scenario_name))
    if scenario_name == '':
        log.info("No scenario was given. Loading sample scenario")
        return sample_scenario(sim_traffic, sim_config, lanelet_map)
    elif scenario_name == 'my_scenario':
        return my_scenario(sim_traffic, sim_config, lanelet_map)

def sample_scenario(sim_traffic:SimTraffic, sim_config:SimConfig, lanelet_map:LaneletMap):
    raise NotImplementedError('sample_scenario is not implemented yet. Please use a GeoScenario file (.osm) or implement your own scenario in code.')

'''
TODO: Adapt the sample scenario to current format

def sample_scenario(sim_traffic:SimTraffic, sim_config:SimConfig, lanelet_map:LaneletMap):
    """ Sample scenario using a Lanelet map
    """
    #Map
    projector = UtmProjector(lanelet2.io.Origin(49.0, 8.4, 0.0))
    map_file = "maps/ll2_mapping_example.osm"
    map_file = os.path.join(ROOT_DIR, 'scenarios',map_file)
    lanelet_map.load_lanelet_map(map_file, projector)
    sim_config.scenario_name = "MyScenario"
    sim_config.timeout = 10

    #A dynamic vehicle (SDV)
    vid = 1
    #vehicle starting position
    #Use the projector to generate the cartesian points from lat/lon
    vehicle_start = projector.forward(GPSPoint(49.0072407, 8.4570652, 329))

    try:
        #Route
        #a list of points [x,y] composing a route. At least two nodes are required.
        # Use the projector to generate the cartesian points from lat/lon
        route_start = projector.forward(GPSPoint(49.0072799, 8.4571337, 329))
        route_end = projector.forward(GPSPoint(49.0081762, 8.4582722, 329))
        route_nodes:list = [ [route_start.x, route_start.y ], [route_end.x, route_end.y] ]
        lanelets_in_route = [ lanelet_map.get_occupying_lanelet(node[0], node[1]) for node in route_nodes ]
        sim_config.lanelet_routes[vid] = lanelet_map.get_route_via(lanelets_in_route)
        sim_config.goal_points[vid] = (route_end.x, route_end.y)    #goal point. Usually, the last node in the route.
    except Exception as e:
        #Route generation can fail if there is no viable lanelet route between points
        log.error("Route generation failed for vid {}.".format(vid))
        raise e

    try:
        vehicle = SDV(vid = vid,                                     #<= must ne integer
                    name = "my_sample_vehicle",                     #vehicle name
                    root_btree_name = "drive_tree",                           #a behavior tree file (.btree) inside the btype's folder, defaulted in btrees
                    start_state = [vehicle_start.x,0.0,0.0, vehicle_start.y,0.0,0.0],          #vehicle start state in cartesian frame [x,x_vel,x_acc, y,y_vel,y_acc]
                    lanelet_map = lanelet_map,
                    lanelet_route = sim_config.lanelet_routes[vid],
                    btree_locations = btree_locations
                    )    #a valid lanelet route
    except Exception as e:
        log.error("Failed to initialize vehicle {}".format(vid))
        raise e
    sim_traffic.add_vehicle(vehicle)

    return True


    #More examples:
    # Add traffic light states
    #states = [TrafficLightColor.Red,TrafficLightColor.Green,TrafficLightColor.Yellow]   #states (alternate in a loop)
    #durations = [10.0, 12.0, 1.0]                                                       #duration for each state in seconds.
    #my_tl = TrafficLight("l_n_s", states, durations)                                    #name must match a traffic light in the lanelet map with tag 'name'
    #sim_traffic.add_traffic_light(my_tl)

    #If vehicles follow trajectories, you must build a List with each time and states
    class TNode:
        time:float
        x:float
        y:float
        yaw:float
        speed:float

    trajectory = [TNode(0.00, 0,0,0), TNode(0.01, 1,0,0.90) ] #...
    sim_traffic.add_trajectory_vehicle(vid, 'my_trajectory_vehicle', [0.0,0.0,0.0,0.0,0.0,0.0], trajectory)

    #If ghost mode is True, the vehicle will be invisible to others
    #sim_traffic.vehicles[vid].ghost_mode = True
    #If vehicle starts as inactive, it won't "exist" until is set to Active
    #sim_traffic.vehicles[vid].sim_state = Vehicle.INACTIVE
    '''

def my_scenario(sim_traffic, sim_config, lanelet_map):
    """ Build your custom scenario here
    """
    return False

#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
# ---------------------------------------------
# GeoScenario Setup
# Build directly from code (instead of GeoScenario (.osm) file)
# --------------------------------------------

from types import LambdaType
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint
from mapping.LaneletMap import *
from SimConfig import SimConfig
from SimTraffic import SimTraffic
from TrafficLight import TrafficLight as TL
from TrafficLight import TrafficLightType, TrafficLightColor
from sv.Vehicle import *
from sp.Pedestrian import *
from gsc.GSParser import GSParser
from Actor import *

def load_geoscenario_from_file(gsfile, sim_traffic:SimTraffic, sim_config:SimConfig, lanelet_map:LaneletMap, evaluation_mode = False):
    """ Setup scenario from GeoScenario file
    """
    #========= Parse GeoScenario File
    log.info("Loading GeoScenario file: {}".format(gsfile))
    full_scenario_path = os.path.join(ROOT_DIR, gsfile)
    parser = GSParser()
    if not parser.load_and_validate_geoscenario(full_scenario_path):
        log.error("Error loading GeoScenario file")
        return False
    if parser.globalconfig.tags['version'] < 2.0:
        log.error("GSServer requires GeoScenario 2.0 or newer")
        return False

    #========= Scenario config
    sim_config.scenario_name = parser.globalconfig.tags['name']
    sim_config.timeout = parser.globalconfig.tags['timeout']
    if 'plotvid' in parser.globalconfig.tags:
        sim_config.plot_vid = parser.globalconfig.tags['plotvid']

    #========= Map
    map_file = os.path.join(ROOT_DIR, 'scenarios', parser.globalconfig.tags['lanelet'])
    # use origin from gsc file to project nodes to sim frame
    altitude  = parser.origin.tags['altitude'] if 'altitude' in parser.origin.tags else 0.0
    projector = UtmProjector(lanelet2.io.Origin(parser.origin.lat, parser.origin.lon, altitude))
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
    
    #=========  Ego (External Vehicle)
    if parser.egostart is not None:
        ego_vehicle = RV(99, 'Ego', [0.0,0.0,0.0, 0.0,0.0,0.0])
        sim_traffic.vehicle(ego_vehicle)

    #========= Vehicles
    for vid, vnode in parser.vehicles.items():
        vid = int(vid)   #<= must ne integer                             
        name = vnode.tags['name']   #vehicle name
        start_state = [vnode.x,0.0,0.0, vnode.y,0.0,0.0]                #vehicle start state in cartesian frame [x,x_vel,x_acc, y,y_vel,y_acc]
        btype = vnode.tags['btype'].lower() if 'btype' in vnode.tags else ''

        #SDV Model (dynamic vehicle)
        if btype == 'sdv':

            if 'route' not in vnode.tags:
                log.error("SDV {} requires a route .".format(vid))
                continue
            try:
                myroute = vnode.tags['route']
                btree_root = vnode.tags['btree'] if 'btree' in vnode.tags else "drive_tree" #a behavior tree file (.btree) inside scenarios/btrees/
                route_nodes = parser.routes[myroute].nodes
                lanelets_in_route = [ lanelet_map.get_occupying_lanelet(node.x, node.y) for node in route_nodes ]   #a valid lanelet route
                sim_config.lanelet_routes[vid] = lanelet_map.get_route_via(lanelets_in_route)
                sim_config.goal_points[vid] = (route_nodes[-1].x, route_nodes[-1].y)
            except Exception as e:
                log.error("Route generation failed for route {}.".format(myroute))
                raise e
            try:
                vehicle = SDV(vid, name, btree_root, start_state , lanelet_map, sim_config.lanelet_routes[vid])    
                if 'btconfig' in vnode.tags:
                    vehicle.btree_reconfig = vnode.tags['btconfig'] 
                sim_traffic.add_vehicle(vehicle)
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
                for node in t_nodes:
                    nd = TrajNode()
                    nd.x = float(node.x)
                    nd.y = float(node.y)
                    nd.time = float(node.tags['time'])
                    nd.speed = float(node.tags['speed']) if ('speed' in node.tags) else None
                    nd.yaw = float(node.tags['yaw']) if ('yaw' in node.tags) else None
                    trajectory.append(nd)
                vehicle = TV(vid = vid,                                     #<= must ne integer                             
                            name = name,                                    #vehicle name
                            start_state =  start_state,                     #vehicle start state in cartesian frame [x,x_vel,x_acc, y,y_vel,y_acc]
                            trajectory = trajectory)                        #a valid trajectory with at least x,y,time per node
                sim_traffic.add_vehicle(vehicle)
            except Exception as e:
                log.error("Failed to initialize vehicle {}".format(vid))
                raise e
        
        # Path Vehicle (PV)
        elif btype == 'pv':
            log.error("Path-based vehicles are still not supported in GeoScenario Server {}".format(vid))
            continue
        
        # External Vehicle (EV)
        elif btype == 'ev':
            vehicle = EV(vid, name, start_state) #<Locaton
            sim_traffic.add_vehicle(vehicle)
        
        # Neutral Vehicle
        else:
            vehicle = Vehicle(vid, name, start_state)
            sim_traffic.add_vehicle(vehicle)
        #=========

    #========= Pedestrians
    for pid, pnode in parser.pedestrians.items():
        pid = int(pid)
        name = pnode.tags['name']
        start_state = [pnode.x,0.0,0.0, pnode.y,0.0,0.0]                    #start state in cartesian frame [x,x_vel,x_acc, y,y_vel,y_acc]
        btype = pnode.tags['btype'].lower() if 'btype' in pnode.tags else ''
        
        
        # Trajectory Pedestrian (TP)
        if btype == 'tp':
            if 'trajectory' not in pnode.tags:
                log.error("PV {} requires a trajectory .".format(pid))
                continue
            try:
                t_name = pnode.tags['trajectory']
                print(t_name)
                print(parser.trajectories)
                t_nodes = parser.trajectories[t_name].nodes
                trajectory = []     #a valid trajectory with at least x,y,time per node
                for node in t_nodes:
                    nd = TrajNode()
                    nd.time = float(node.tags['time'])
                    nd.x = float(node.x)
                    nd.y = float(node.y)
                    nd.yaw = float(node.tags['yaw']) if 'yaw' in node.tags else None
                    nd.speed = float(node.tags['speed']) if 'speed' in node.tags else None
                    trajectory.append(nd)
                pedestrian = TP(pid, name, start_state,trajectory)
                sim_traffic.add_pedestrian(pedestrian)
            except Exception as e:
                log.error("Failed to initialize pedestrian {}".format(pid))
                raise e
        # Simulated pedestrian
        elif btype == 'sp':
            if 'destination' not in pnode.tags:
                log.error("SP {} requires a destination .".format(pid))
                continue
            try:
                p_dest = pnode.tags['destination']
                btree_root = pnode.tags['btree'] if 'btree' in pnode.tags else "trees/pedestrian_trees/walk.btree" # a behavior tree file (.btree) inside scenarios/trees/pedestrian_trees
                dest_node = parser.locations[p_dest]
                #lanelets_in_route = [lanelet_map.get_occupying_lanelet(node.x, node.y) for node in route_nodes]   # a valid lanelet route
                #sim_config.lanelet_routes[pid] = lanelet_map.get_route_via(lanelets_in_route)
                sim_config.pedestrian_goal_points[pid] = (dest_node.x, dest_node.y)
                pedestrian = SP(pid, name, start_state, list(sim_config.pedestrian_goal_points[pid]), btree_root)
                sim_traffic.add_pedestrian(pedestrian)
            except Exception as e:
                log.error("Failed to initialize pedestrian {}".format(pid))
                raise e
        else:
            pedestrian = Pedestrian(pid, name, start_state)
            sim_traffic.add_pedestrian(pedestrian)
    
    #========= Static Objects
    #Area based objetics are not supported yet.
    for oid, onode in parser.staticobjects.items():
        sim_traffic.add_static_obect(oid, onode.x, onode.y )

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
                    btree_root = "drive_tree",                           #a behavior tree file (.btree) inside scenarios/btrees/
                    start_state =  [vehicle_start.x,0.0,0.0, vehicle_start.y,0.0,0.0],          #vehicle start state in cartesian frame [x,x_vel,x_acc, y,y_vel,y_acc]
                    lanelet_map = lanelet_map, 
                    lanelet_route = sim_config.lanelet_routes[vid])    #a valid lanelet route
    except Exception as e:
        log.error("Failed to initialize vehicle {}".format(vid))
        raise e
    sim_traffic.add_vehicle(vehicle)
    
    return True

    '''
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

def my_scenario():
    """ Build your custom scenario here
    """
    return False

    
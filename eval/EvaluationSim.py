import glog as log
import os
import time
import sqlite3
import utm
import string
from sv.SDVPlanner import LaneConfig
from TickSync import TickSync
from SimTraffic import SimTraffic
from TrafficLight import TrafficLight, TrafficLightColor
from util.Transformations import sim_to_frenet_position
from mapping.LaneletMap import *
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint
from SimConfig import *
from gsc.GSParser import GSParser
from util.Utils import *
from sv.Vehicle import Vehicle
import dataclasses
import csv
import difflib
from math import degrees
import numpy as np


@dataclass
class EvalScenario:
    scenario_id:str = ''
    track_id:int = 0
    vehicle_type:str = ''                           #Car, Pedestrian, Medium Vehicle, Heavy Vehicle
    scenario_type:str = ''                          #free, follow, free_follow, rlstop, glstart, yield_turnright, yield_turnleft, lcleft, lcright
    const_vehicles:list = field(default_factory=list)    #vehicles/pedestrians that must be in the scene (e.g: following, or yielding)
    start_time:float = 0.0                          #scenario start time
    end_time:float = 0.0                            #scenario end time
    direction:str = ''                              #n_s, s_n, e_w, w_e

@dataclass
class SDVConfig:
    btree_root:str = "eval_main"
    btree_reconfig:str = ""
    start_state:list = field(default_factory=list) #[0.0,0.0,0.0, 0.0,0.0,0.0]  #cartesian [xpos,xvel,xacc, upos, yvel, yacc]
    route_nodes:list = field(default_factory=list)
    vk_target_vel:float = 14.0
    vk_time:float = 3.0
    vk_time_low:float = 6.0
    follow_timegap:float = 3.0
    glstart_delay:float = 0.0
    rlstop_dist:float = 0.0


#Nodes for trajectory
@dataclass
class TrajNode:
    x:float = 0.0
    y:float = 0.0
    time:float = 0.0
    speed:float = 0.0
    angle:float = 0.0

   
def setup_evaluation(gsfile, sim_traffic:SimTraffic, sim_config:SimConfig, lanelet_map:LaneletMap, scenario_id, recalibrate):
    print("===== Setup scenario {} for evaluation. Recalibrate? {}".format(scenario_id,recalibrate))
    calibrate_behavior = True
    if recalibrate == 'n' or recalibrate == 'no':
        calibrate_behavior = False
    
    #==========================  Load Base Scenario

    full_scenario_path = os.path.join(ROOT_DIR, gsfile)
    parser = GSParser()
    if not parser.load_and_validate_geoscenario(full_scenario_path):
        log.error("Error loading GeoScenario file")
        return False, 0.0

    sim_config.scenario_name = parser.globalconfig.tags['name']
    sim_config.timeout = parser.globalconfig.tags['timeout']

    #map
    map_file = os.path.join(ROOT_DIR, 'scenarios', parser.globalconfig.tags['lanelet'])
    projector = UtmProjector(lanelet2.io.Origin(parser.origin.lat, parser.origin.lon))
    parser.project_nodes(projector)
    lanelet_map.load_lanelet_map(map_file, projector)
    sim_config.map_name = parser.globalconfig.tags['lanelet']

    # add traffic lights
    for name, tnode in parser.tlights.items():
        type = tnode.tags['type']
        # link the traffic light reg elem to the traffic light state from GS
        tl_reg_elem = lanelet_map.get_traffic_light_by_name(name)
        states = list(map(TrafficLightColor.from_str, tnode.tags['states'].split(',')))
        durations = list(map(float, str(tnode.tags['duration']).split(',')))
        sim_traffic.add_traffic_light(tl_reg_elem, name, type, states, durations)

    #========================== Load Scenario

    # Master csv to guide the experiment.
    es = load_scenario_db(scenario_id)
    
    # Database to retrieve trajectory info
    connection = sqlite3.connect('eval/uni_weber_769.db')
    c = connection.cursor()

    trajectories = {}
    #eval vehicle
    trajectories[es.track_id]  = query_track(es.track_id, c, projector)
    #load tracks for dependencies (vehicles, pedestrians)
    for cvid in es.const_vehicles:
        trajectories[cvid] = query_track(cvid, c, projector)

    #estimate scenario configuration
    config = generate_config(es, lanelet_map, sim_traffic.traffic_lights, trajectories, calibrate_behavior)

    #========================== Populate Simulation

    
    #Add Vehicles to Simulation
    for vid in trajectories:
        trajectory = trajectories[vid]
        
        veh_name = ('veh_'+ format(vid, '03d'))
        sim_traffic.add_trajectory_vehicle(vid, veh_name, [-1000.0,0.0,0.0,-1000.0,0.0,0.0], trajectory)

        if vid == es.track_id:
            #set reference trajectory vehicle as a ghost vehicle for reference
            sim_traffic.vehicles[vid].ghost_mode = True
            
            #create dynamic vehicle in same trajectory with -vid
            evid = -vid
            try:
                lanelets_in_route = [ lanelet_map.get_occupying_lanelet(node.x, node.y) for node in config.route_nodes ]
                sim_config.lanelet_routes[evid] = lanelet_map.get_route_via(lanelets_in_route)
                print(sim_config.lanelet_routes[evid])
            except Exception as e:
                log.error("Route generation failed for route {}. Can't use this vehicle for evaluation".format(evid))
                return False, 0.0
            sim_config.goal_points[evid] = (config.route_nodes[-1].x, config.route_nodes[-1].y)
            sim_traffic.add_vehicle(evid, '-'+veh_name, config.start_state,
                                    sim_config.lanelet_routes[evid], 
                                    config.btree_root, config.btree_reconfig, 
                                    False)
            #start as inactive until the original trajectory starts
            sim_traffic.vehicles[evid].sim_state = Vehicle.INACTIVE
    
    
    if calibrate_behavior:
        sim_config.scenario_name = "{}_rc".format(scenario_id)
        sim_traffic.log_file = sim_config.scenario_name
    else:
        sim_config.scenario_name = "{}_nc".format(scenario_id)
        sim_traffic.log_file = sim_config.scenario_name

    sim_config.plot_vid = -es.track_id
    sim_config.timeout = es.end_time

    return True, es.start_time



def load_scenario_db(scenario_id):
    es = None
    with open('eval/scenarios.csv', mode='r', encoding='utf-8-sig') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if row[0] == "scenario_id": #header
                continue
            if row[0]==scenario_id:
                #int(eval_vid):
                #exclusions
                if row[1] == 'x':
                    log.error("Vehicle {} cannot be used for evaluation".format(scenario_id))
                    return None
                es = EvalScenario()
                es.scenario_id = scenario_id
                es.track_id = int(row[2])
                es.vehicle_type = row[3]
                es.direction = row[4]
                es.scenario_type = row[5]
                es.scenario_ = row[5]
                #constraints
                if (row[6] != ''):
                    if (',' in row[6]):
                        es.const_vehicles = [int(cvid) for cvid in row[6].split(',')]
                    else:
                        es.const_vehicles.append(int(row[6]))
                #time bound
                if (row[7] != ''):
                    es.start_time = float(row[7])
                if (row[8] != ''):
                    es.end_time = float(row[8])
                #print("Loaded scenario:")
                #print(es)
                break;
    
    if es is None:
        log.error("Scenario for -e {} not found".format(scenario_id))
        return None
    
    return es
        
    

def query_track(vid, c, projector):
    #track
    track = []
    query = "SELECT  * \
                FROM TRAJECTORIES_0769 \
                LEFT JOIN TRAJECTORIES_0769_EXT \
                ON TRAJECTORIES_0769.TRACK_ID == TRAJECTORIES_0769_EXT.TRACK_ID AND TRAJECTORIES_0769.TIME == TRAJECTORIES_0769_EXT.TIME\
                WHERE TRAJECTORIES_0769.TRACK_ID == '{}'\
                ORDER BY TRAJECTORIES_0769.TIME".format(vid)
    res = c.execute(query)
    for row in res:
        track.append(row)
    
    trajectory = []
    for step in track:
        time = float(step[6])
        node = TrajNode()
        node.time = time
        lat,lon = utm.to_latlon(step[1],step[2],17,'N') #index 1 and 2 are utm x and y
        cart_pt = projector.forward(GPSPoint(lat, lon, 0.0))
        node.x = cart_pt.x
        node.y = cart_pt.y
        node.speed = float(step[3]) / 3.6 #NOTE: assuming speed from DB is km/h
        node.angle = float(step[7]) #degrees(float(step[7]))  #NOTE: assuming angle from DB is radians and UTM84
        node.xvel, node.yvel = speed_to_vel(node.speed, node.angle)
        #node.tan_acc = float(step[4])
        #node.lat_acc = float(step[5])
        
        trajectory.append(node)
    return trajectory

def generate_config(es, lanelet_map, traffic_lights, trajectories, calibrate_behavior):
    ''''
    Configure dynamic vehicle behavior to match empirical vehicle 
    using the original track to extract stats
    '''
    @dataclass
    class TrajStats:
        start_time:float = 0.0
        end_time:float  = 0.0
        start_speed:float = 0.0
        end_speed:float = 0.0
        max_speed:float = 0.0
        avg_speed:float = 0.0
        start_angle:float = 0.0
        start_vel_x:float  = 0.0
        start_vel_y:float = 0.0

    #Traj Stats
    ts = TrajStats()
    trajectory = trajectories[es.track_id]
    trajectory.sort(key=lambda x: x.time)       #Sorting nodes using time (if not sorted during query)
    if (es.start_time > trajectory[0].time):    #if trajectory starts before scenario, clip nodes
        trajectory = [node for node in trajectory if node.time>= es.start_time]
    ts.start_time = trajectory[0].time          #min(trajectory,key=lambda x:x.time).time
    ts.end_time = trajectory[-1].time           #max(trajectory,key=lambda x:x.time).time
    ts.start_speed = trajectory[0].speed
    ts.end_speed = trajectory[-1].speed
    ts.min_speed = min(trajectory,key=lambda x:x.speed).speed
    ts.max_speed = max(trajectory,key=lambda x:x.speed).speed
    ts.avg_speed = sum([ node.speed for node in trajectory]) / len(trajectory)
    ts.start_angle = trajectory[0].angle
    ts.start_vel_x, ts.start_vel_y = speed_to_vel(trajectory[0].speed, trajectory[0].angle)
    
    print (es.end_time)
    #Scenario
    if (es.start_time < ts.start_time):         #scenario starts only when vehicle enters the scene
        es.start_time = ts.start_time
    if es.end_time == 0.0:                      #if not defined, scenario ends when trajectory ends
        es.end_time = ts.end_time

    config = SDVConfig()

    #Standard config
    config.route_nodes = [ trajectory[0], trajectory[-1]]
    config.start_state = [ trajectory[0].x, 0.0, 0.0, trajectory[0].y, 0.0, 0.0 ] 
    config.btree_root = "eval_main"
    config.btree_reconfig = "m_vkeeping=MVelKeepConfig(vel=MP(14.0,10,3), time=MP(3.0,20,6), time_lowvel=MP(6.0,20,3))"

    #Recalibration Config
    if calibrate_behavior:
        config.start_state = [ trajectory[0].x, ts.start_vel_x,0.0, trajectory[0].y, ts.start_vel_y,0.0 ]

        if es.scenario_type == 'free':
            config.vk_target_vel = format(ts.avg_speed, '.2f')
            #config.vk_target_vel = format(ts.end_speed, '.2f')
            config.vk_time = 3.0
            config.vk_time_low = 6.0
            #todo: lane offset reconfig
            #config.start_state = [  trajectory[0].x,ts.start_vel_x,0.0, trajectory[0].y,ts.start_vel_y,0.0 ] 

        elif es.scenario_type == 'follow':
            config.vk_target_vel = ts.avg_speed
            config.vk_time = 3.0
            config.follow_timegap = 2.0 #todo estimate

        elif es.scenario_type == 'free_follow':
            config.vk_target_vel = ts.avg_speed
            config.vk_time = 3.0
            config.follow_timegap = 2.0 #todo estimate

        elif es.scenario_type == 'rlstop':
            config.vk_target_vel = ts.avg_speed
            config.vk_time = 3.0
            config.rlstop_dist = find_stop_distance(lanelet_map, trajectory)

        elif es.scenario_type == 'glstart':
            print("GLSTART")
            config.vk_target_vel = ts.max_speed
            config.vk_time = 5.0
            config.vk_time_low = 6.0
            config.glstart_delay = find_gl_delay(lanelet_map, trajectory, traffic_lights)
            config.start_state = [ trajectory[0].x, 0.0, 0.0, trajectory[0].y, 0.0, 0.0 ] 
            

        elif es.scenario_type == 'yield_turnleft':    
            pass
        elif es.scenario_type == 'yield_turnright':    
            pass
        elif es.scenario_type == 'lcleft':    
            pass
        elif es.scenario_type == 'lcright':    
            pass

        config.btree_reconfig = "m_vkeeping=MVelKeepConfig(vel=MP({},10,3), time=MP({},20,6), time_lowvel=MP({},20,3) )".format(
                    config.vk_target_vel, config.vk_time, config.vk_time_low)
        config.btree_reconfig += ";m_stop_redlight=(MStopConfig( type=3 , distance={}) )".format(config.rlstop_dist)
    
    #Summary: 
    print("======= Experiment Summary ===")
    print(es)
    print(ts)
    print(config)
    print("===============================")

    return config



def find_stop_distance(laneletmap:LaneletMap, trajectory): #, traffic_light_states):
    """ Find stop distance from stop line.
    """
    stop_node = None
    resume_node = None
    stop_distance = None

    #find where trajectory stops
    for node in trajectory:
        if node.speed <= 0.01:
            stop_node = node
            #print ("Stop Node")
            #print (node)
            break
    
    #Find distance to stop line
    cur_ll = laneletmap.get_occupying_lanelet(stop_node.x,stop_node.y)
    #regulatory elements acting on this lanelet
    reg_elems = cur_ll.regulatoryElements
    reg_elem_stats = {}
    for re in reg_elems:
        if isinstance(re, lanelet2.core.TrafficLight):
            stop_linestring = re.parameters['ref_line']
            stop_distance = distance_point_line(stop_linestring[0][0].x,stop_linestring[0][0].y, 
                                stop_linestring[0][-1].x, stop_linestring[0][-1].y,
                                stop_node.x, stop_node.y)

    if stop_distance is None:
        print ("Can't find stop distance")        
        return 0.0

    return stop_distance

def find_follow_gap(laneletmap:LaneletMap, trajectory):
    pass

def find_gl_delay(laneletmap:LaneletMap, trajectory, traffic_lights):
    delay = 0.0
    #Find traffic light
    mytl = None
    cur_ll = laneletmap.get_occupying_lanelet(trajectory[0].x,trajectory[0].y)
    #regulatory elements acting on this lanelet
    reg_elems = cur_ll.regulatoryElements
    reg_elem_stats = {}
    for re in reg_elems:
        if isinstance(re, lanelet2.core.TrafficLight):
            for re_tl in re.trafficLights: 
                tlname = re_tl.attributes['name']
                for id, tl in traffic_lights.items():
                    if tl.name == tlname:
                        mytl = tl
    if mytl is None:
        print("Traffic Light not found in current lanelet")
        return delay

    #find where trajectory resumes driving after the green light
    for node in trajectory:
        if node.speed > 0.1:
            state,gl_time = mytl.state_in(node.time)
            #print(state, gl_time)
            if state == TrafficLightColor.Green:
                delay = format(gl_time,'.3f')
                break
    
    print("Delay after green light is {}".format(delay))
    return delay
  

'''
    cur_ll = laneletmap.get_occupying_lanelet(x,y)
    middle_lane_width = LaneletMap.get_lane_width(cur_ll, x, y)
    middle_lane_config = LaneConfig(0, 30, middle_lane_width / 2, middle_lane_width / -2)
    
    # Get regulatory elements acting on this lanelet
    reg_elems = cur_ll.regulatoryElements
    reg_elem_stats = {}
    for re in reg_elems:
        if isinstance(re, lanelet2.core.TrafficLight):
            # lanelet2 traffic lights must have a corresponding state from the main process
            #if re.id not in traffic_light_states:
            #    continue
            stop_linestring = re.parameters['ref_line']
            # choose the closest point on the stop line as the stop position
            stop_pos = min(
                sim_to_frenet_position(reference_path, stop_linestring[0][0].x, stop_linestring[0][0].y),
                sim_to_frenet_position(reference_path, stop_linestring[0][-1].x, stop_linestring[0][-1].y),
                key=lambda p: p[0])
            #reg_elem_stats.append(TrafficLightState(color=traffic_light_states[re.id], stop_position=stop_pos))
            reg_elem_stats.stop_pos=stop_pos
            print(stop_pos)
            #distance_2d(stop_pos.x, stop_pos.y, )
            #reg_elem_stats.distance_to_stop_pos=stop_pos
    
    return #middle_lane_config, stop_pos, distance_to_stop
'''




""" 
   
    
    #All Vehicles IDs
    #vehicles_ids = []
    #query = "SELECT TRACK_ID FROM TRACKS WHERE (TYPE == 'Car' OR TYPE == 'Medium Vehicle' OR TYPE == 'Heavy Vehicle') "
    #query += "AND ENTRY_GATE == 'g_n_en_st_r'" 
    #if LIMIT_QUERY:
    #    query +=" LIMIT {}".format(LIMIT_QUERY)
    #res = c.execute(query)
    #for row in res:
    #    vehicles_ids.append(row[0])
    #print (len(vehicles_ids))
    #Pedestrians IDs
    #dependencies that affect a vehicle.
    #v_constraints = {}
    # 2, 4, 5-10, 12, 14 no constraints
    #13 not on db, but follows 6
    #15 not on db, but follows 4
    #constraints[11] = [] #['769 011 000 01 02'] ['769 011 000 02 02']  #01 wait for oncoming 02 proceed turn 03 track speed
    #v_constraints[20] = [11] #['769 011 020 04 01'] follow lead and track speed, but it does not follow 11
    #v_constraints[23] = [11] #['769 011 023 03 01']
    #v_constraints[49] = [58] # ['769 058 049 03 02'] 3 track speed, but 58 wait for oncoming 49 and 52
    #v_constraints[52] = [58] #['769 058 052 03 01'] 3 track speed
    #v_constraints[58] = [100] # ['769 058 000 05 02'] 5 decelerate-to-stop ['769 058 000 02 02'] 02 proceed turn ['769 058 000 0101']  01 wait for oncoming ['769 100 058 0202'] 02 proceed turn 
    #checked up to 60
    # 
    # 
    # 

def add_route_vehicle(simvid, parser, lanelet_map, sim_traffic,sim_config):
    #Dynamic vehicles:
    vnode = parser.vehicles[simvid]
    if 'route' in vnode.tags:
        myroute = vnode.tags['route']
        btree_root = "drive_tree" #default
        if 'btree' in vnode.tags:
            btree_root = vnode.tags['btree']
        try:
            # NOTE may not want to prepend vehicle node, in case the user starts the vehicle in the middle of the path
            route_nodes = parser.routes[myroute].nodes
            lanelets_in_route = [ lanelet_map.get_occupying_lanelet(node.x, node.y) for node in route_nodes ]
            sim_config.lanelet_routes[simvid] = lanelet_map.get_route_via(lanelets_in_route)
        except Exception as e:
            log.error("Route generation failed for route {}.".format(myroute))
            raise e

        sim_config.goal_points[simvid] = (route_nodes[-1].x, route_nodes[-1].y)
        sim_traffic.add_vehicle(simvid, vnode.tags['name'], [vnode.x,0.0,0.0, vnode.y,0.0,0.0],
                            sim_config.lanelet_routes[simvid], btree_root)
    

def add_trajectory_vehicle(simvid, parser, lanelet_map, sim_traffic, sim_config, static_to_dynamic = False, evaluation_mode = False):
    #Vehicles following predefined trajectory
    vnode = parser.vehicles[simvid]
    if 'path' in vnode.tags:
        try:
            mypath = vnode.tags['path']
            path_nodes = parser.paths[mypath].nodes
            trajectory = [ (node.x, node.y, node.tags['time']) for node in path_nodes ] 
        except Exception as e:
            log.error("Trajectory generation failed for path {}.".format(mypath))
            raise e

        if not static_to_dynamic:
            sim_traffic.add_trajectory_vehicle(simvid, vnode.tags['name'], [-1000.0,0.0,0.0,-1000.0,0.0,0.0], trajectory)
        
        else:
            #create dynamic vehicle in same trajectory
            btree_root = "drive_tree" #default
            myroute = mypath
            try:
                route_nodes = [ path_nodes[0], path_nodes[-1]]
                speed = path_nodes[0].tags['speed']
                lanelets_in_route = [ lanelet_map.get_occupying_lanelet(node.x, node.y) for node in route_nodes ]
                print("route_nodes {} \nstart speed {} \nlanelets in route {} ".format(route_nodes,speed,lanelets_in_route))
                sim_config.lanelet_routes[simvid] = lanelet_map.get_route_via(lanelets_in_route)
            except Exception as e:
                log.error("Route generation failed for route {}.".format(myroute))
                raise e
            sim_config.goal_points[simvid] = (route_nodes[-1].x, route_nodes[-1].y)
            sim_traffic.add_vehicle(simvid, vnode.tags['name'], [vnode.x,0.0,0.0, vnode.y,0.0,0.0],                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
                                sim_config.lanelet_routes[simvid], btree_root)
            
            if evaluation_mode:
                #start as inactive until the original trajectory starts
                sim_traffic.vehicles[simvid].sim_state = Vehicle.INACTIVE   
                #add original trajectory vehicle as a ghost vehicle for reference
                sim_traffic.add_trajectory_vehicle(-simvid, vnode.tags['name'], [-1000.0,0.0,0.0,-1000.0,0.0,0.0], trajectory)
                sim_traffic.vehicles[-simvid].ghost_mode = True """
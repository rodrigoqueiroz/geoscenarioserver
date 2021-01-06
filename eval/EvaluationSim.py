import glog as log
import os
import time
import sqlite3
import utm
import string

from TickSync import TickSync
from SimTraffic import SimTraffic
from TrafficLight import TrafficLightColor
from mapping.LaneletMap import *
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint
from SimConfig import *
from gsc.GSParser import GSParser
from sv.SV import Vehicle
import dataclasses
import csv
import difflib

from eval.VehicleConstraints import *

@dataclass
class EvalScenario:
    track_id = 0
    vehicle_type = ''            #Car, Pedestrian, Medium Vehicle, Heavy Vehicle
    scenario_type = ''   #straigh, follow, stop_rl, start_gl, start_rl, wait_oncoming_turn_left, wait_oncoming_turn_right, lane_change_left, lane_change_right
    const_vehicles = []    #vehicles that must be in the scene (e.g: following, or wait for oncoming vehicles)
    start_time = 0.0        #some tracks contains more than one scenario. clip trajectory to starting time
    direction = ''       #n_s, s_n, e_w, w_e
    
    

@dataclass
class TrajStats:
    start_time = 0.0
    end_time = 0.0
    max_vel = 0.0
    avg_vel = 0.0
    #estimates
    target_vel = 0.0
    time_to_target_vel = 0.0
    follow_distance = 0.0
    follow_time_gap = 0.0
    start_gl_delay = 0.0
    decel = 0.0


#Nodes for trajectory
@dataclass
class TrajNode:
    x = 0.0
    y = 0.0
    time = 0.0
    speed = 0.0
    angle = 0.0

   
        

def setup_evaluation(gsfile, sim_traffic, sim_config, lanelet_map, eval_vid):
    """ Setup scenario for Evaluation
    """

    #==== Config
    
    #vehicles that can't be used for evaluation
    #reasons: start in the middle of intersection, or outside lanelets, or traverse not modelled lanelets
    exclusions = [1,8,11,16,17,18,19,20,22,23,27,33,45,46,63,69,85]
    not_found = [3,33,37] #on manual tagging
    
    if eval_vid in exclusions:
        log.error("Vehicle cannot be used for evaluation")
        return False

    #===== Load Base Scenario
    full_scenario_path = os.path.join(ROOT_DIR, gsfile)
    parser = GSParser()
    if not parser.load_and_validate_geoscenario(full_scenario_path):
        log.error("Error loading GeoScenario file")
        return False

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

    #========================== Load Scenario csv
    # Using master spreadsheet to guide the experiment. 
    # The databse is used to retrieve trajectory info

    
    es = load_scenario_db(eval_vid)
    #get all relevant track ids
    track_ids = []
    track_ids.append(es.track_id)
    for cvid in es.const_vehicles:
        track_ids.append(cvid)

    #==========================DATABASE

    #connect
    conn = sqlite3.connect('eval/uni_weber_769.db')
    c = conn.cursor()
 
    #Get trajectories for all tracks of interest   
    trajectories = {}
    traj_stats = {}
    print("DB Tracks query")
    for id in track_ids:
        trajectories[id], traj_stats[id] = query_track(id,c,projector, es.start_time)
    print("Done")

    #========================== estimate scenario confguration
    #if traj_stats[es.track_id].start_time >  es.start_time
    #    es.start_time = traj_stats[es.track_id].start_time

    if es.scenario_type == 'straight':
        #traj_stats[es.track_id].target_vel
        pass

    elif es.scenario_type == 'follow':
        #traj_stats[es.track_id].follow_distance = 0
        #traj_stats[es.track_id].follow_time_gap = 0
        pass

    elif es.scenario_type == 'stop_rl':
        #decel
        pass

    elif es.scenario_type == 'start_gl':
        #find gl time
        #start_gl_delay
        pass

    elif es.scenario_type == 'wait_concoming_turn_left':    
        pass
    elif es.scenario_type == 'wait_concoming_turn_right':    
        pass
    elif es.scenario_type == 'lane_change_left':    
        pass
    elif es.scenario_type == 'lane_change_right':    
        pass

    #==========================POPULATE SIM

    
    #Add Vehicles to Simulation
    for vid in trajectories:
        trajectory = trajectories[vid]
        ts = traj_stats[vid]
        veh_name = ('veh_'+ format(vid, '03d'))
        sim_traffic.add_trajectory_vehicle(vid, veh_name, [-1000.0,0.0,0.0,-1000.0,0.0,0.0], trajectory)

        if vid == es.track_id:

            #set reference trajectory vehicle as a ghost vehicle for reference
            sim_traffic.vehicles[vid].ghost_mode = True
            
            #create dynamic vehicle in same trajectory with -vid
            evid = -vid
            btree_root = "drive_tree_eval" #default
            reconfig = {}
            reconfig['keep_velocity'] = "MVelKeepConfig(vel=MP({},10,6)), time=MP({},10,3)".format(ts.target_vel, ts.time_to_target_vel)
            #reconfig['']
            #reconfig['']
            print ("reconfig")
            print (reconfig)
            
            try:
                route_nodes = [ trajectory[0], trajectory[-1]]
                speed = 10
                lanelets_in_route = [ lanelet_map.get_occupying_lanelet(node.x, node.y) for node in route_nodes ]
                print("route_nodes {} \nstart speed {} \nlanelets in route {} ".format(route_nodes,speed,lanelets_in_route))
                sim_config.lanelet_routes[evid] = lanelet_map.get_route_via(lanelets_in_route)
            except Exception as e:
                log.error("Route generation failed for route {}. Can't use this vehicle for evaluation".format(evid))
                continue
                
            sim_config.goal_points[evid] = (route_nodes[-1].x, route_nodes[-1].y)
            sim_traffic.add_vehicle(evid, '-'+veh_name, [trajectory[0].x,0.0,0.0, trajectory[0].y,0.0,0.0],                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
                                sim_config.lanelet_routes[evid], btree_root)
            #start as inactive until the original trajectory starts
            sim_traffic.vehicles[evid].sim_state = Vehicle.INACTIVE
            
    return True



def load_scenario_db(eval_vid):
    with open('eval/scenarios_from_db.csv', mode='r', encoding='utf-8-sig') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if row[0] == "track_id":
                continue
            vid = int(row[0])
            if vid==int(eval_vid):
                es = EvalScenario()
                es.track_id = vid
                es.vehicle_type = row[1]
                es.direction = row[2]
                es.scenario_type = row[3]
                if (row[4] != ''):
                    if ('-' in row[4]):
                        es.const_vehicles = [int(cvid) for cvid in row[4].split('-')]
                    else:
                        es.const_vehicles.append(int(row[4]))
                if (row[5] != ''):
                    es.start_time = float(row[5])
                print(es)
    
    if es is not None:
        return es
    else:
        log.error("Scenario for -e {} not found".format(eval_vid))
        return None
    

def query_track(vid, c, projector,clip_time = 0.0):
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
    all_vel = []
    print (clip_time)
    for step in track:
        time = float(step[6])
        if time >= clip_time:
            node = TrajNode()
            node.time = time
            lat,lon = utm.to_latlon(step[1],step[2],17,'N') #index 1 and 2 are utm x and y
            cart_pt = projector.forward(GPSPoint(lat, lon, 0.0))
            node.x = cart_pt.x
            node.y = cart_pt.y
            node.vel = float(step[3])
            #node.tan_acc = float(step[4])
            #node.lat_acc = float(step[5])
            node.angle = float(step[7])
            trajectory.append(node)
            #trajectory.sort(key=lambda x: x.time) #if not sorted during query
            #stat
            all_vel.append(node.vel)
            
    
    ts = TrajStats()
    ts.avg_vel = sum(all_vel) / len(all_vel)
    ts.max_vel = max(all_vel)
    ts.start_time = min(trajectory,key=lambda x:x.time)
    ts.end_time = max(trajectory,key=lambda x:x.time)
    #todo: extract estimates from trajectory
    ts.target_vel = ts.avg_vel 
    ts.time_to_target_vel = 3.0
    ts.decel = 9.0
    ts.lane_offset = 0.0
    print(ts.avg_vel)
    return trajectory, ts

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
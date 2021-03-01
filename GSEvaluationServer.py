#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# GeoScenario Server for Evaluation only
# Controls the problem setup and traffic simulation loop
# --------------------------------------------


import os
import sqlite3
import utm
import string
import dataclasses
import csv
import glog as log
from math import degrees
import numpy as np
from argparse import ArgumentParser
from TickSync import TickSync
from SimTraffic import SimTraffic
from TrafficLight import TrafficLight, TrafficLightColor
from dash.Dashboard import *
from gsc.GSParser import GSParser
from sv.Vehicle import Vehicle
from Actor import TrajNode
from sv.ManeuverConfig import *
from sv.SDVPlanner import LaneConfig
from util.Transformations import sim_to_frenet_position
from util.Utils import *
from mapping.LaneletMap import *
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint
from SimConfig import *
from ScenarioSetup import *


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
    #vk_time_low:float = 6.0
    follow_timegap:float = 3.0
    glstart_delay:float = 0.0
    rlstop_dist:float = 0.0


@dataclass
class TrajStats:
    start_time:float = 0.0
    start_yaw:float = 0.0
    start_speed:float = 0.0
    start_vel_x:float  = 0.0
    start_vel_y:float = 0.0
    min_speed:float = 0.0
    max_speed:float = 0.0
    avg_speed:float = 0.0
    avg_driving_speed:float = 0.0
    stop_time:float = 0.0
    end_time:float  = 0.0
    end_speed:float = 0.0
    
def start_server(args, es, calibrate = False):
    log.info('GeoScenario Evaluation Server START')
    lanelet_map = LaneletMap()
    sim_config = SimConfig()
    sim_traffic = SimTraffic(lanelet_map, sim_config)
    
    #Scenario SETUP
    res, time = setup_evaluation_scenario(args.gsfile, sim_traffic, sim_config, lanelet_map, es, calibrate)
    if not res:
        return

    sync_global = TickSync(rate=sim_config.traffic_rate, realtime=True, block=True, verbose=False, label="EX", sim_start_time = float(time))
    sync_global.set_timeout(sim_config.timeout)

    #GUI / Debug screen
    dashboard = Dashboard(sim_traffic, sim_config)

    #SIM EXECUTION START
    log.info('SIMULATION START')
    sim_traffic.start()
    show_dashboard = SHOW_DASHBOARD and not args.no_dash
    dashboard.start(show_dashboard)

    while sync_global.tick():
        if show_dashboard and not dashboard._process.is_alive(): # might/might not be wanted
            break
        #try:
        #Update Traffic
        sim_status = sim_traffic.tick(
        sync_global.tick_count,
        sync_global.delta_time,
        sync_global.sim_time
        )
        if sim_status < 0:
            break
        #except Exception as e:
        #    log.error(e)
        #    break
        
    sim_traffic.stop_all()
    dashboard.quit()
    #SIM END
    log.info('SIMULATION END')
    log.info('GeoScenario Evaluation Server SHUTDOWN')

   
def setup_evaluation_scenario(gsfile, sim_traffic:SimTraffic, sim_config:SimConfig, lanelet_map:LaneletMap, es, calibrate_behavior):
    print("===== Setup scenario {} for evaluation. Recalibrate? {}".format(es.scenario_id,calibrate_behavior))
    
    #==========================  Load Base Scenario
    #if not parser.load_and_validate_geoscenario(full_scenario_path):
    if not load_geoscenario_from_file(gsfile,sim_traffic,sim_config,lanelet_map):
        log.error("Error loading GeoScenario file")
        return False, 0.0

    #========================== Load Tracks
    # Database to retrieve trajectory info
    connection = sqlite3.connect('eval/uni_weber_769.db')
    c = connection.cursor()

    trajectories = {}
    trajectories[es.track_id]  = query_track(es.track_id, c, lanelet_map.projector)
    #load tracks for dependencies (vehicles, pedestrians)
    for cvid in es.const_vehicles:
        trajectories[cvid] = query_track(cvid, c, lanelet_map.projector)

    #========================== Estimate scenario configuration
    config = generate_config(es, lanelet_map, sim_traffic.traffic_lights, trajectories, calibrate_behavior)

    #========================== Populate Simulation
    #Add Vehicles to Simulation
    for vid in trajectories:
        trajectory = trajectories[vid]
        
        name = ('veh_'+ format(vid, '03d'))
        vehicle = TV(vid, name, [-1000.0,0.0,0.0,-1000.0,0.0,0.0], trajectory, False)
        sim_traffic.add_vehicle(vehicle)

        if vid == es.track_id:
            #set reference trajectory vehicle to run in ghost mode (not visible in traffic)
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

            vehicle = SDV(evid, '-'+name, config.btree_root, config.start_state , 
                                lanelet_map, sim_config.lanelet_routes[evid])
            #vehicle = Vehicle(evid, '-'+name, config.start_state)
            vehicle.btree_reconfig = config.btree_reconfig
            sim_traffic.add_vehicle(vehicle)
            #start as inactive until the original trajectory starts
            sim_traffic.vehicles[evid].sim_state = ActorSimState.INACTIVE
    
    if calibrate_behavior:
        sim_config.scenario_name = "{}_rc".format(es.scenario_id)
        sim_traffic.log_file = sim_config.scenario_name
    else:
        sim_config.scenario_name = "{}_nc".format(es.scenario_id)
        sim_traffic.log_file = sim_config.scenario_name

    sim_config.plot_vid = -es.track_id
    sim_config.timeout = es.end_time

    return True, es.start_time

def load_all_scenarios():
    scenarios = {}
    with open('eval/scenarios.csv', mode='r', encoding='utf-8-sig') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if row[0] == "scenario_id": #skip header
                continue
            if row[0] == "": #skip empty
                continue
            if row[1] == 'x': #skip exclusions
                continue
            es = EvalScenario()
            es.scenario_id = row[0]
            es.track_id = int(row[2])
            es.vehicle_type = row[3]
            es.direction = row[4]
            es.scenario_type = row[5]
            #constraints
            if (row[6] != ''):
                if (',' in row[6]):
                    es.const_vehicles = [int(cvid) for cvid in row[6].split(',')]
                else:
                    es.const_vehicles.append(int(row[6]))
            #start time
            if (row[7] != ''):
                es.start_time = float(row[7])
            if (row[8] != ''):
                    es.end_time = float(row[8])
            scenarios[es.scenario_id] = es
    return scenarios
        
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
        node.yaw = float(step[7]) #degrees(float(step[7]))  #NOTE: assuming angle from DB is radians and UTM84
        node.x_vel, node.y_vel = speed_to_vel(node.speed, node.yaw)
        #node.tan_acc = float(step[4])
        #node.lat_acc = float(step[5])
        
        trajectory.append(node)
    return trajectory

def generate_config(es:EvalScenario, lanelet_map:LaneletMap, traffic_lights, trajectories, calibrate_behavior):
    ''''
    Configure dynamic vehicle behavior to match empirical vehicle 
    using the original track to extract stats
    '''
    trajectory = trajectories[es.track_id]
    trajectory.sort(key=lambda x: x.time)       #Sorting nodes using time (if not sorted during query)
    if (es.start_time > trajectory[0].time):    #if trajectory starts before scenario, clip nodes
        trajectory = [node for node in trajectory if node.time>= es.start_time]
    #Traj Stats
    ts = generate_traj_stats(trajectory) 
    #scenario starts only when vehicle enters the scene
    if (es.start_time < ts.start_time):         
        es.start_time = ts.start_time
    es.end_time = ts.end_time
    #Config
    config = SDVConfig()
    #Standard
    config.route_nodes = [ trajectory[0], trajectory[-1]]
    config.start_state = [ trajectory[0].x, ts.start_vel_x,0.0, trajectory[0].y, ts.start_vel_y,0.0 ]
    config.btree_root = "trees/eval_main.btree"
    config.btree_reconfig = ""

    print("Both start at  x_sdv {} x_tv{}".format(config.start_state[0],trajectory[0].x))

    #scenario specific
    if es.scenario_type == 'free':
        config.btree_root = "trees/eval_drive.btree"
        if calibrate_behavior:
            config.vk_target_vel = format(ts.avg_speed, '.2f')
            config.vk_time = 3.0
            config.btree_reconfig+= "m_vkeeping=MVelKeepConfig(vel=MP({},10,3))".format(config.vk_target_vel)

    elif es.scenario_type == 'follow' or  es.scenario_type == 'free_follow':
        config.btree_root = "trees/eval_drive.btree"
        lead_id = es.const_vehicles[0] #assuming one lead vehicle for the entire scenario
        lead_ts = generate_traj_stats(trajectories[lead_id])
        es.end_time = lead_ts.end_time #scenario ends when lead vehicle exists
        if calibrate_behavior:
            config.vk_time = 3.0
            config.vk_target_vel = ts.avg_driving_speed
            mingap, avggap, maxgap = find_follow_gap(lanelet_map,trajectory, trajectories[lead_id],ts)
            config.follow_timegap = avggap  #3.0
            config.btree_reconfig = "m_follow_lead=MFollowConfig(time_gap={})".format(config.follow_timegap)
            config.btree_reconfig+= ";m_vkeeping=MVelKeepConfig(vel=MP({},10,3))".format(config.vk_target_vel)

    elif es.scenario_type == 'rlstop':
        es.end_time = ts.stop_time
        if calibrate_behavior:
            config.vk_target_vel = ts.avg_speed
            config.vk_time = 3.0
            config.rlstop_dist = find_stop_distance(lanelet_map, trajectory)
            config.btree_reconfig = "m_stop_redlight=(MStopConfig( type=3 , distance={}) )".format(config.rlstop_dist)
            config.btree_reconfig += ";m_vkeeping=MVelKeepConfig(vel=MP({},10,3))".format(config.vk_target_vel)

    elif es.scenario_type == 'glstart':
        if calibrate_behavior:
            #config.vk_target_vel = ts.avg_speed
            config.vk_target_vel = ts.avg_driving_speed
            config.vk_time = 3.0
            delay = find_gl_delay(lanelet_map, trajectory, traffic_lights)
            if delay < 0.33:
                config.glstart_delay = 0
            else:
                config.glstart_delay = delay
            config.start_state = [ trajectory[0].x, 0.0, 0.0, trajectory[0].y, 0.0, 0.0 ] 
            config.btree_reconfig = "c_wait=wait,args=(time={})".format(config.glstart_delay)
            config.btree_reconfig += ";m_vkeeping=MVelKeepConfig(vel=MP({},10,3))".format(config.vk_target_vel)
            #to consider:
            # the final speed on the riginal track may not be accurate. getting an average on the last quartil can help
            # the limiting factor on the acceleration could be lifted to allow steeper startup
            
    
    #Summary: 
    print("======= Experiment Summary ===")
    print(es)
    print(ts)
    print(config)
    print("===============================")

    return config

def generate_traj_stats(trajectory):
    ts = TrajStats()
    ts.start_time = trajectory[0].time          #min(trajectory,key=lambda x:x.time).time
    ts.end_time = trajectory[-1].time           #max(trajectory,key=lambda x:x.time).time
    ts.start_speed = trajectory[0].speed
    ts.end_speed = trajectory[-1].speed
    ts.min_speed = min(trajectory,key=lambda x:x.speed).speed
    ts.max_speed = max(trajectory,key=lambda x:x.speed).speed
    ts.avg_speed = sum([ node.speed for node in trajectory]) / len(trajectory)
    ts.start_yaw = trajectory[0].yaw
    ts.start_vel_x, ts.start_vel_y = speed_to_vel(trajectory[0].speed, trajectory[0].yaw)
    for node in trajectory:
        if node.speed <= 0.01:
            ts.stop_time = node.time
            break
    
    #average for higher speeds (stopping times)
    nodes_driving = []
    for node in trajectory:
        if node.speed > 4:
            nodes_driving.append(node)
    ts.avg_driving_speed = sum([ node.speed for node in nodes_driving]) / len(nodes_driving)
    return ts

def find_stop_distance(laneletmap:LaneletMap, trajectory): #, traffic_light_states):
    """ Find stop distance from stop line.
    """
    stop_node = None
    stop_distance = None

    #find where trajectory stops
    for node in trajectory:
        if node.speed <= 0.01:
            stop_node = node
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
            if stop_distance > VEHICLE_RADIUS:
                stop_distance = stop_distance - VEHICLE_RADIUS
            #else:
            #    stop_distance

    if stop_distance is None:
        print ("Can't find stop distance")        
        return 0.0

    return stop_distance

def find_follow_gap(laneletmap:LaneletMap, t, tlead,  ts:TrajStats):
    #Assuming both are ordered, but the times do not match.
    #Samples for both trajectories can be colected on different times.
    #Lead not necessarrily in leading role the whole time.
    #Vehicles can also become lead coming from different route

    print("Find follow gap")

    follow_zone = 30
    n_samples = 100
    min_time = ts.start_time
    max_time = ts.end_time
    
    t_norm = {}
    tlead_norm = {}
    
    #run a sim over the trajectory time to capture both vehicles at the same time
    for stime in np.linspace(min_time,max_time, n_samples,dtype = float, endpoint=False):
        #find closest node to stime
        node = None
        for i in range(len(t)):
            if  stime >= t[i].time:
                node = t[i]
        if not node:
            continue

        #find closest lead node if available
        node_lead = None
        if stime < tlead[-1].time: #if lead still active
            for i in range(len(tlead)):
                if stime >= tlead[i].time:
                    node_lead = tlead[i]
        if not node_lead:
            continue     

        time = format(stime,'.2f')
        t_norm[time] = node
        tlead_norm[time] = node_lead
        
    
    all_tgs = []
    for time,node in t_norm.items():
        diff = distance_2p(tlead_norm[time].x,tlead_norm[time].y,node.x,node.y)
        dist_between_vehicles = diff - (VEHICLE_RADIUS*2)
        time_gap = dist_between_vehicles / abs(node.speed) if node.speed != 0 else float('inf')
        #focus on a reasonable time gap. above the threshold is not relevant
        if 1 < time_gap < 6:
            all_tgs.append(time_gap)
        #else:
        #    print(time_gap)

    mingap = format(min(all_tgs), '.2f')
    avggap = format(sum([ tg for tg in all_tgs]) / len(all_tgs), '.2f')
    maxgap = format(max(all_tgs), '.2f')
    print("Follow gap: min{} avg{} max{}".format(mingap, avggap, maxgap))
    return mingap, avggap, maxgap
    
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
                delay = float(format(gl_time,'.3f'))
                break
    
    print("Delay after green light is {}".format(delay))
    return delay
  
if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-n", "--no_dash", dest="no_dash", action="store_true", help="run without the dashboard")
    parser.add_argument("-s", "--scenario", dest="gsfile", metavar="FILE", default="", help="GeoScenario file")
    parser.add_argument("-e", "--eval", dest="eval_id", default="", help="Evaluation scenario ID")
    parser.add_argument("-t", "--type", dest="eval_type", default="", help="Type for batch evaluation")
    parser.add_argument("-rc", "--recalibrate", dest="recalibrate", default="y", help="[y/n/b] Recalibrate behavior to match reference vehicle (b for both)")
    parser.add_argument("-c", "--compare", dest="compare", default="y", help="[y/n/e] Compare trajectories? e=for exclusivelly")

    
    args = parser.parse_args()
    
    CLIENT_SHM = False
    WRITE_TRAJECTORIES = True

    # Master csv to guide all experiments.
    scenarios = load_all_scenarios()

    #Run single scenario
    if args.eval_id != "":
        try:
            if args.recalibrate == 'b':
                start_server(args, scenarios[args.eval_id], False)
                start_server(args, scenarios[args.eval_id], True)
            elif args.recalibrate == 'n':
                start_server(args, scenarios[args.eval_id], False)
            else: #default
                start_server(args, scenarios[args.eval_id], True)

        except Exception as e:
            print("ERROR. Can not run simulation for scenario{}".format(args.eval_id))
            raise e
    #Run all scenarios from type
    elif args.eval_type != "":
        for eval_id in scenarios:
            if scenarios[eval_id].scenario_type == args.eval_type:
                try:
                    if args.recalibrate == 'b':
                        start_server(args, scenarios[eval_id], False)
                        start_server(args, scenarios[eval_id], True)
                    elif args.recalibrate == 'n':
                        start_server(args, scenarios[eval_id], False)
                    else: #default
                        start_server(args, scenarios[eval_id], True)
                except Exception as e:
                    print("ERROR. Can not run simulation for scenario{}".format(eval_id))
    
   

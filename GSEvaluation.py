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
from sv.ManeuverConfig import *
from sv.SDVPlanner import LaneConfig
from util.Transformations import sim_to_frenet_position
from util.Utils import *
from mapping.LaneletMap import *
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint
from SimConfig import *


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

@dataclass
class TrajNode:
    x:float = 0.0
    y:float = 0.0
    time:float = 0.0
    speed:float = 0.0
    angle:float = 0.0

@dataclass
class TrajStats:
    start_time:float = 0.0
    start_angle:float = 0.0
    start_speed:float = 0.0
    start_vel_x:float  = 0.0
    start_vel_y:float = 0.0
    min_speed:float = 0.0
    max_speed:float = 0.0
    avg_speed:float = 0.0
    stop_time:float = 0.0
    end_time:float  = 0.0
    end_speed:float = 0.0
    
def start_server(args, es):
    log.info('GeoScenario Evaluation Server START')
    lanelet_map = LaneletMap()
    sim_config = SimConfig()
    sim_traffic = SimTraffic(lanelet_map, sim_config)
    
    #Scenario SETUP
    res, time = setup_evaluation_scenario(args.gsfile, sim_traffic, sim_config, lanelet_map, es, args.recalibrate)
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
        try:
            #Update Traffic
            sim_status = sim_traffic.tick(
                sync_global.tick_count,
                sync_global.delta_time,
                sync_global.sim_time
            )
            if sim_status < 0:
                break
        except Exception as e:
            log.error(e)
            break
        
    sim_traffic.stop_all()
    dashboard.quit()
    #SIM END
    log.info('SIMULATION END')
    log.info('GeoScenario Evaluation Server SHUTDOWN')

   
def setup_evaluation_scenario(gsfile, sim_traffic:SimTraffic, sim_config:SimConfig, lanelet_map:LaneletMap, es, recalibrate):
    print("===== Setup scenario {} for evaluation. Recalibrate? {}".format(es.scenario_id,recalibrate))
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
        tltype = tnode.tags['type']
        # link the traffic light reg elem to the traffic light state from GS
        tl_reg_elem = lanelet_map.get_traffic_light_by_name(name)
        states = list(map(TrafficLightColor.from_str, tnode.tags['states'].split(',')))
        durations = list(map(float, str(tnode.tags['duration']).split(',')))
        sim_traffic.add_traffic_light(tl_reg_elem, name, tltype, states, durations)

    #========================== Load Scenario

    
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
        node.angle = float(step[7]) #degrees(float(step[7]))  #NOTE: assuming angle from DB is radians and UTM84
        node.xvel, node.yvel = speed_to_vel(node.speed, node.angle)
        #node.tan_acc = float(step[4])
        #node.lat_acc = float(step[5])
        
        trajectory.append(node)
    return trajectory

def generate_config(es:EvalScenario, lanelet_map:LaneletMap, traffic_lights, trajectories, calibrate_behavior):
    ''''
    Configure dynamic vehicle behavior to match empirical vehicle 
    using the original track to extract stats
    '''


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
    for node in trajectory:
        if node.speed <= 0.01:
            ts.stop_time = node.time
            break
    #Scenario
    #scenario starts only when vehicle enters the scene
    if (es.start_time < ts.start_time):         
        es.start_time = ts.start_time
    #if not defined, ends trajectory ends or when vehicle stops
    #if es.end_time == 0.0:                      
    if es.scenario_type == 'rlstop':
        es.end_time = ts.stop_time
    else:
        es.end_time = ts.end_time

    config = SDVConfig()

    #Standard config
    config.route_nodes = [ trajectory[0], trajectory[-1]]
    config.start_state = [ trajectory[0].x, ts.start_vel_x,0.0, trajectory[0].y, ts.start_vel_y,0.0 ]
    config.btree_root = "eval_main"
    config.btree_reconfig = "m_vkeeping=MVelKeepConfig(vel=MP(14.0,10,3), time=MP(3.0,20,6), time_lowvel=MP(6.0,20,3))"

    #Recalibration
    if calibrate_behavior:

        if es.scenario_type == 'free':
            config.vk_target_vel = format(ts.avg_speed, '.2f')
            #config.vk_target_vel = format(ts.end_speed, '.2f')
            config.vk_time = 3.0
            config.vk_time_low = 6.0
            #TODO: lane offset reconfig
            #config.start_state = [  trajectory[0].x,ts.start_vel_x,0.0, trajectory[0].y,ts.start_vel_y,0.0 ] 

        elif es.scenario_type == 'follow':
            config.vk_target_vel = ts.avg_speed
            config.vk_time = 3.0
            config.follow_timegap = 2.0
            for cvid in es.const_vehicles:
                find_follow_gap(lanelet_map,trajectory, trajectories[cvid],ts)

        elif es.scenario_type == 'free_follow':
            config.vk_target_vel = ts.avg_speed
            config.vk_time = 3.0
            config.follow_timegap = 2.0 #todo estimate
            for cvid in es.const_vehicles:
                find_follow_gap(lanelet_map,trajectory, trajectories[cvid],ts)

        elif es.scenario_type == 'rlstop':
            config.vk_target_vel = ts.avg_speed
            config.vk_time = 3.0
            config.rlstop_dist = find_stop_distance(lanelet_map, trajectory)

        elif es.scenario_type == 'glstart':
            print("GLSTART")
            config.vk_target_vel = ts.avg_speed
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
        for i in range(len(t)):
            if  stime >= t[i].time:
                node = t[i]
        
        #find closest lead node if available
        node_lead = None
        if stime < tlead[-1].time: #if lead still active
            for i in range(len(tlead)):
                if stime >= tlead[i].time:
                    node_lead = tlead[i]
            
        time = format(stime,'.2f')
        t_norm[time] = node
        tlead_norm[time] = node_lead
        
    #debug. print all diffs
    for key,value in t_norm.items():
        print(key - value.time)

    #todo: check if is acually following

    '''    
        tlead_norm[time] = node_lead
            if t[i].time < stime:

            j_start = 0 #leverage last found index to speed up search
            for j in range(j_start,len(tlead)-1):
                dif = abs(tlead[j].time - t[i].time)
                dif_next =  abs(tlead[j+1].time - t[i].time)
                if dif < dif_next:
                    #j is closest in time
                    t[i].lead_index = j
                    t[i].time_dif = dif
                    t[i].speed_dif = dif

                    j_start = j
    
    #estimate the follow gap (distance and time)
    
    for i in np.linspace(min_time,max_time, n_samples,dtype = int, endpoint=False):
        sim_trajectory(t)
        sim_trajectory(j)
        
                #for node in self.trajectory:
                for i in range(len(self.trajectory)):
                    node = self.trajectory[i]
                    if node.time < sim_time:
                        continue
                    #closest after current sim time
                    #TODO: interpolate taking the difference between closest node time and sim time
                    #print("closest node time {} >= simtime {}".format(node_time,sim_time))
                    
                    self.vehicle_state.set_X([node.x, node.xvel, xacc])
                    self.vehicle_state.set_Y([node.y, node.yvel, xacc])
                    break
            #After trajectory
            if sim_time > end_time:
                #vanish. Need to optimize this by setting vehicles as not visible and removing all calculations with it
                self.vehicle_state.set_X([-9999, 0, 0])
                self.vehicle_state.set_Y([-9999,0,0])
                if self.sim_state is Vehicle.ACTIVE or self.sim_state is Vehicle.INVISIBLE:
                    log.warn("vid {} is now INACTIVE".format(self.vid))
                    self.sim_state = Vehicle.INACTIVE
                    #workaround for evaluation only
                    #if -self.vid in self.simtraffic.vehicles:
                    #    self.simtraffic.vehicles[-self.vid].sim_state = Vehicle.INACTIVE 
    '''
    

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
  
if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-n", "--no_dash", dest="no_dash", action="store_true", help="run without the dashboard")
    parser.add_argument("-s", "--scenario", dest="gsfile", metavar="FILE", default="", help="GeoScenario file")
    parser.add_argument("-e", "--eval", dest="eval_id", metavar="FILE", default="", help="GeoScenario file")
    parser.add_argument("-t", "--type", dest="eval_type", default="", help="Type for batch evaluation")
    parser.add_argument("-rc", "--recalibrate", dest="recalibrate", default="y", help="[y/n] Recalibrate behavior to match reference vehicle")
    parser.add_argument("-c", "--compare", dest="compare", default="y", help="[y/n/e] Compare trajectories? e=for exclusivelly")

    
    args = parser.parse_args()
    
    CLIENT_SHM = False
    WRITE_TRAJECTORIES = True

    # Master csv to guide all experiments.
    scenarios = load_all_scenarios()

    #Run single scenario
    if args.eval_id != "":
        try:
            start_server(args, scenarios[args.eval_id])
        except Exception as e:
            print("ERROR. Can not run simulation for scenario{}".format(args.eval_id))
            raise e
    #Run all scenarios from type
    elif args.eval_type != "":
        for eval_id in scenarios:
            if scenarios[eval_id].scenario_type == args.eval_type:
                try:
                    start_server(args, scenarios[eval_id])
                except Exception as e:
                    print("ERROR. Can not run simulation for scenario{}".format(eval_id))
    
   

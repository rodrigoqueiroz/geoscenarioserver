#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
#slarter@uwaterloo.ca
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
from sp.Pedestrian import Pedestrian
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
    agent_type:str = ''                           #Car, Pedestrian, Medium Vehicle, Heavy Vehicle
    scenario_type:str = ''                          #free, follow, free_follow, rlstop, glstart, yield_turnright, yield_turnleft, lcleft, lcright
    const_agents:list = field(default_factory=list)    #vehicles/pedestrians that must be in the scene (e.g: following, or yielding)
    start_time:float = 0.0                          #scenario start time
    end_time:float = 0.0                            #scenario end time
    direction:str = ''                              #n_s, s_n, e_w, w_e


@dataclass
class SPConfig:
    btree_root:str = "eval_walk.btree"
    btree_reconfig:str = ""
    start_state:list = field(default_factory=list) #[0.0,0.0,0.0, 0.0,0.0,0.0]  #cartesian [xpos,xvel,xacc, ypos, yvel, yacc]
    route_nodes:list = field(default_factory=list)
    # vk_target_vel:float = 14.0
    # vk_time:float = 3.0
    # vk_time_low:float = 6.0
    # follow_timegap:float = 3.0
    # glstart_delay:float = 0.0
    # rlstop_dist:float = 0.0


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
    avg_walking_speed:float = 0.0
    avg_speed_exit:float = 0.0
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


    #SIM EXECUTION START
    log.info('SIMULATION START')
    sim_traffic.start()

     #GUI / Debug screen
    dashboard = Dashboard(sim_traffic, sim_config)
    if sim_config.show_dashboard:
        dashboard.start()
    else:
        log.warn("Dashboard will not start")

    while sync_global.tick():
        if sim_config.show_dashboard and not dashboard._process.is_alive(): # might/might not be wanted
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
    base_btree_location = os.path.join(ROOT_DIR, "evaluation/eval_scenarios/") #default btree folders location
    btree_locations = [base_btree_location]
    log.info ("Btree search locations set (in order) as: " + str(btree_locations))
    if not load_geoscenario_from_file(gsfile,sim_traffic,sim_config,lanelet_map,"",btree_locations):
        log.error("Error loading GeoScenario file")
        return False, 0.0

    #========================== Load Tracks
    # Database to retrieve trajectory info
    connection = sqlite3.connect('evaluation/uni_weber_769.db')
    c = connection.cursor()

    trajectories = {}
    trajectories[es.track_id]  = query_track(es.track_id, c, lanelet_map.projector)
    #load tracks for dependencies (vehicles, pedestrians)
    for cid in es.const_agents:
        trajectories[cid] = query_track(cid, c, lanelet_map.projector)

    #========================== Estimate scenario configuration
    config = generate_config(es, lanelet_map, sim_traffic.traffic_lights, trajectories, calibrate_behavior)

    #========================== Populate Simulation
    #Add Vehicles to Simulation
    for id in trajectories:
        agent_type = trajectories[id]['agent_type']
        trajectory = trajectories[id]['trajectory']

        if agent_type == "Pedestrian":
            name = ('ped_'+ format(id, '03d'))
            pedestrian = TP(id, name, [-1000.0,0.0,0.0,-1000.0,0.0,0.0], trajectory, False)
            sim_traffic.add_pedestrian(pedestrian)

            if id == es.track_id:
                #set reference trajectory pedestrian to run in ghost mode (not visible in traffic)
                sim_traffic.pedestrians[id].ghost_mode = True

                #create dynamic simulated pedestrian in same trajectory with -id
                epid = -id
                try:
                    sim_config.pedestrian_goal_points[epid] = [(node.x, node.y) for node in config.route_nodes] # route
                    #sim_config.pedestrian_goal_points[epid] = [(config.route_nodes[-1].x, config.route_nodes[-1].y)] # destination

                    pedestrian = SP(epid,
                                    '-'+name,
                                    config.start_state,
                                    list(sim_config.pedestrian_goal_points[epid]),
                                    config.btree_root,
                                    btree_locations=btree_locations,
                                    btype="sp")

                    pedestrian.btree_reconfig = config.btree_reconfig
                    sim_traffic.add_pedestrian(pedestrian)
                    #start as inactive until the original trajectory starts
                    sim_traffic.pedestrians[epid].sim_state = ActorSimState.INACTIVE
                except Exception as e:
                    log.error("Route generation failed for route {}. Can't use this pedestrian for evaluation".format(epid))
                    return False, 0.0
        else:
            name = ('veh_'+ format(id, '03d'))
            vehicle = TV(id, name, [-1000.0,0.0,0.0,-1000.0,0.0,0.0], trajectory, False)
            sim_traffic.add_vehicle(vehicle)

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
    with open('evaluation/pedestrian_scenarios.csv', mode='r', encoding='utf-8-sig') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if row[0] == "scenario_id": #skip header
                continue
            if row[0] == "": #skip empty
                continue
            if row[1] == 'x' or row[1] == '?' or row[1] == '': #skip exclusions, or empty selection
                continue
            if row[5] == '': #skip empty scenario types
                continue
            es = EvalScenario()
            es.scenario_id = row[0]
            es.track_id = int(row[2])
            es.agent_type = row[3]
            es.direction = row[4]
            es.scenario_type = row[5]
            #constraints
            if (row[6] != ''):
                if (',' in row[6]):
                    es.const_agents = [int(cid) for cid in row[6].split(',')]
                else:
                    es.const_agents.append(int(row[6]))
            #start time
            if (row[7] != ''):
                es.start_time = float(row[7])
            if (row[8] != ''):
                    es.end_time = float(row[8])
            scenarios[es.scenario_id] = es
    return scenarios

def query_track(id, c, projector):
    query = "SELECT T.TRACK_ID, T.X, T.Y, T.SPEED, T.TAN_ACC, T.LAT_ACC, T.TIME, T.ANGLE, TK.TYPE \
            FROM TRAJECTORIES_0769 T \
            LEFT JOIN TRACKS TK ON T.TRACK_ID = TK.TRACK_ID \
            WHERE T.TRACK_ID = '{}' \
            ORDER BY T.TIME".format(id)
    c.execute(query)
    res = c.fetchall()

    trajectory = []
    for row in res:
        time = float(row[6])
        node = TrajNode()
        node.time = time
        lat,lon = utm.to_latlon(row[1],row[2],17,'T') #index 1 and 2 are utm x and y
        cart_pt = projector.forward(GPSPoint(lat, lon, 0.0))
        node.x = cart_pt.x
        node.y = cart_pt.y
        node.speed = float(row[3]) / 3.6 #NOTE: assuming speed from DB is km/h
        node.yaw = float(row[7]) #degrees(float(row[7]))  #NOTE: assuming angle from DB is radians and UTM84
        node.x_vel, node.y_vel = speed_to_vel(node.speed, node.yaw)
        #node.tan_acc = float(row[4])
        #node.lat_acc = float(row[5])

        trajectory.append(node)

    agent_type = res[0][8]

    return {'agent_type': agent_type, 'trajectory': trajectory}

def generate_config(es:EvalScenario, lanelet_map:LaneletMap, traffic_lights, trajectories, calibrate_behavior):
    ''''
    Configure dynamic pedestrian behavior to match empirical pedestrian
    using the original track to extract stats
    '''
    trajectory = trajectories[es.track_id]['trajectory']
    trajectory.sort(key=lambda x: x.time)       #Sorting nodes using time (if not sorted during query)
    if (trajectory[0].time < es.start_time):    #if trajectory starts before scenario, clip nodes
        trajectory = [node for node in trajectory if node.time >= es.start_time]
    if (trajectory[-1].time > es.end_time):    #if trajectory ends before scenario, clip nodes
        trajectory = [node for node in trajectory if node.time <= es.end_time]
    #Traj Stats
    ts = generate_traj_stats(trajectory)
    #scenario starts only when vehicle enters the scene
    if (es.start_time < ts.start_time):
        es.start_time = ts.start_time
    es.end_time = ts.end_time
    #Config
    config = SPConfig()
    config.btree_root = 'eval_walk.btree'
    #Standard
    #config.route_nodes = [ trajectory[0], trajectory[-1]]
    config.route_nodes = trajectory[::10] # every 10 nodes in trajectory
    config.start_state = [ trajectory[0].x, ts.start_vel_x,0.0, trajectory[0].y, ts.start_vel_y,0.0 ]
    config.btree_reconfig = ""

    print("Both start at x_sp {} x_tp {}".format(config.start_state[0],trajectory[0].x))

    #scenario specific
    if es.scenario_type == 'free':
        config.btree_root = "eval_drive.btree"
        if calibrate_behavior:
            config.vk_target_vel = format(ts.avg_speed_exit, '.2f')
            config.d_target = find_lateral_pos(lanelet_map,trajectory)
            config.vk_time = 3.0
            config.btree_reconfig+= "m_vkeeping=MVelKeepConfig( vel=MP({},10,3), lat_target=LT({}) )".format(config.vk_target_vel,config.d_target)

    elif es.scenario_type == 'follow' or  es.scenario_type == 'free_follow':
        config.btree_root = "eval_drive.btree"
        lead_id = es.const_agents[0] #assuming one lead vehicle for the entire scenario
        lead_ts = generate_traj_stats(trajectories[lead_id])
        es.end_time = lead_ts.end_time #scenario ends when lead vehicle exists
        if calibrate_behavior:
            config.d_target = find_lateral_pos(lanelet_map,trajectory)
            if abs(config.d_target) > 1.0:
                config.off_lane = 0
            else:
                config.off_lane = 1
            config.vk_time = 3.0
            config.vk_target_vel = format(ts.avg_walking_speed, '.2f')
            mingap, avggap, maxgap = find_follow_gap(lanelet_map,trajectory, trajectories[lead_id],ts)
            config.follow_timegap = avggap  #3.0
            #config.btree_reconfig = "m_follow_lead=MFollowConfig( time_gap={},lat_target=LT({}),feasibility_constraints['off_lane'] = {} )".format(
            config.btree_reconfig = "m_follow_lead=MFollowConfig( time_gap={},lat_target=LT({},3) )".format(
                config.follow_timegap,
                config.d_target)
                #config.off_lane)
            config.btree_reconfig+= ";m_vkeeping=MVelKeepConfig( vel=MP({},10,3), lat_target=LT({},3) )".format(
                config.vk_target_vel,
                config.d_target)
                #config.off_lane)

    elif es.scenario_type == 'rlstop':
        es.end_time = ts.stop_time
        if calibrate_behavior:
            config.d_target = find_lateral_pos(lanelet_map,trajectory)
            config.vk_target_vel = ts.start_speed
            config.vk_time = 3.0
            config.rlstop_dist = find_stop_distance(lanelet_map, trajectory)
            config.btree_reconfig = "m_stop_redlight=MStopConfig( target=3 , distance={}, lat_target=LT({}) )".format(config.rlstop_dist, config.d_target)
            config.btree_reconfig += ";m_vkeeping=MVelKeepConfig( vel=MP({},10,3), lat_target=LT({}) )".format(config.vk_target_vel,config.d_target)


    elif es.scenario_type == 'glstart':
        if calibrate_behavior:
            config.d_target = find_lateral_pos(lanelet_map,trajectory)
            #config.vk_target_vel = ts.avg_speed
            config.vk_target_vel = format(ts.avg_speed_exit, '.2f')
            config.vk_time = 3.0
            delay = find_gl_delay(lanelet_map, trajectory, traffic_lights)
            if delay < 0.33:
                config.glstart_delay = 0
            else:
                config.glstart_delay = delay
            config.start_state = [ trajectory[0].x, 0.0, 0.0, trajectory[0].y, 0.0, 0.0 ]
            config.btree_reconfig = "c_wait=wait,args=(time={})".format(config.glstart_delay)
            config.btree_reconfig += ";m_vkeeping=MVelKeepConfig( vel=MP({},10,3), lat_target=LT({}) )".format(config.vk_target_vel,config.d_target)
            #to consider:
            # the final speed on the riginal track may not be accurate. getting an average on the last quartil can help
            # the limiting factor on the acceleration could be increased to allow steeper startup



    #Summary:
    print("======= Experiment Summary =======")
    print(es)
    print(ts)
    print(config)
    print("==================================")

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
    nodes_walking = []
    for node in trajectory:
        if node.speed > 0.5:
            nodes_walking.append(node)
    if len(nodes_walking) > 0:
        ts.avg_walking_speed = sum([ node.speed for node in nodes_walking]) / len(nodes_walking)
    else:
        ts.avg_walking_speed = 0.0

    #average speed by the end of track (last quarter)
    start = int(3 * len(trajectory)/4)
    ts.avg_speed_exit = sum([ trajectory[i].speed for i in range(start,len(trajectory)) ]) / (len(trajectory)-start)

    return ts

def find_lateral_pos(lanelet_map:LaneletMap, trajectory):
    route_nodes = [ trajectory[0], trajectory[-1]]
    lanelets_in_route = [ lanelet_map.get_occupying_lanelet(node.x, node.y) for node in route_nodes ]
    lanelet_route = lanelet_map.get_route_via(lanelets_in_route)
    global_path = lanelet_map.get_global_path_for_route(lanelet_route)
    _, d_vector = sim_to_frenet_frame(global_path, [trajectory[0].x,0.0,0.0], [trajectory[0].y,0.0,0.0])
    d_target = d_vector[0]
    print("D TARGET IS:")
    print(d_target)
    return d_target


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
    parser.add_argument("-c", "--compare", dest="compare", default="y", help="[y/n/e] Compare trajectories? e=for exclusively")
    parser.add_argument("-a", "--all", dest="eval_all", action="store_true", help="Batch evaluation for all trajectories")


    args = parser.parse_args()

    CLIENT_SHM = False
    WRITE_TRAJECTORIES = True

    # Master csv to guide all experiments.
    scenarios = load_all_scenarios()

    if args.eval_all:
         for id in scenarios:
            try:
                if args.recalibrate == 'b':
                    start_server(args,scenarios[id], False)
                    start_server(args,scenarios[id], True)
                elif args.recalibrate == 'n':
                    start_server(args,scenarios[id], False)
                else:
                    start_server(args,scenarios[id], True)

            except Exception as e:
                print("ERROR. Can not run evaluation for scenario{}".format(id))
    #Run single scenario
    elif args.eval_id != "":
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
                #try:
                if args.recalibrate == 'b':
                    start_server(args, scenarios[eval_id], False)
                    start_server(args, scenarios[eval_id], True)
                elif args.recalibrate == 'n':
                    start_server(args, scenarios[eval_id], False)
                else: #default
                    start_server(args, scenarios[eval_id], True)
                #except Exception as e:
                #    print("ERROR. Can not run simulation for scenario{}".format(eval_id))

from argparse import ArgumentParser
import csv
import numpy as np
from math import sqrt, exp
import random
from dataclasses import dataclass, field
from matplotlib import pyplot as plt
from frechetdist import frdist

@dataclass
class EvalScenario:
    scenario_id:str = ''
    track_id:int = 0
    vehicle_type:str = ''                           #Car, Pedestrian, Medium Vehicle, Heavy Vehicle
    scenario_type:str = ''                          #free, follow, free_follow, rlstop, glstart, yield_turnright, yield_turnleft, lcleft, lcright
    const_vehicles:list = field(default_factory=list)    #vehicles/pedestrians that must be in the scene (e.g: following, or yielding)
    start_time:float = 0.0                          #scenario start time
    direction:str = ''                              #(straight) n_s, s_n, e_w, w_e (left turns) s_w, w_n (right turn) s_e, w_s, (not supported) e_n, n_w

def evaluate(args):
    scenarios = load_all_scenarios()

    if args.eval_id != "":
        try:
            evaluate_scenario(scenarios[args.eval_id])
        except Exception as e:
            print("ERROR. Can not run evaluation for scenario{}".format(args.eval_id))
            raise e

    elif args.eval_type != "":
        for id in scenarios:
            if scenarios[id].scenario_type == args.eval_type:
                try:
                    evaluate_scenario(scenarios[id])
                except Exception as e:
                    print("ERROR. Can not run evaluation for scenario{}".format(id))

def evaluate_scenario(es:EvalScenario):
    #calibrated
    #empirical
    traj_q_rc = load_trajectory_log( 'trajlog/{}_rc_{}.csv'.format(es.scenario_id, es.track_id) )
    #synthetic
    traj_p_rc = load_trajectory_log( 'trajlog/{}_rc_{}.csv'.format(es.scenario_id, -es.track_id) )
    filename = 'plots/{}/{}_rc_{}'.format(es.scenario_type, es.scenario_id, es.track_id)
    title = 'Frechet distance RC v{}'.format(es.track_id)
    score_rc = calc_frechet(traj_p_rc, traj_q_rc, "synthetic", "empirical", trim=7.0, samples = 300, filename = filename, title = title)
    
    #not calibrated
    #empirical
    traj_p_nc = load_trajectory_log( 'trajlog/{}_nc_{}.csv'.format(es.scenario_id, -es.track_id) )
    #synthetic
    traj_q_nc = load_trajectory_log( 'trajlog/{}_nc_{}.csv'.format(es.scenario_id, es.track_id) )
    filename = 'plots/{}/{}_nc_{}'.format(es.scenario_type, es.scenario_id, es.track_id)
    title = 'Frechet distance NC v{}'.format(es.track_id)
    score_nc = calc_frechet(traj_p_nc, traj_q_nc,"SDV Model", "empirical", trim=7.0, samples = 300, filename = filename, title = title)
    
    #calc_frechet(traj_p_rc, traj_q_rc, 1, 2, trim=1.0, samples = None, filename = plotfilename, title = title)
    
    #Write results to file
    with open('results.csv', mode='r',encoding='utf-8-sig')as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        lines = list(csv_reader)
        found = False
        for l in lines:
            if l[0] == es.scenario_id:
                l[1] = es.scenario_type
                l[2] = score_nc
                l[3] = score_rc
                found = True
        if not found:
            lines.append([es.scenario_id,es.scenario_type,score_nc,score_rc])
    
    with open('results.csv', mode='w',encoding='utf-8-sig')as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',')
        csv_writer.writerows(lines)
    

def load_all_scenarios():
    scenarios = {}
    with open('scenarios.csv', mode='r', encoding='utf-8-sig') as csv_file:
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
    

def load_scenario_db(scenario_id):
    es = None
    with open('scenarios.csv', mode='r', encoding='utf-8-sig') as csv_file:
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
                #constraints
                if (row[6] != ''):
                    if (',' in row[6]):
                        es.const_vehicles = [int(cvid) for cvid in row[6].split(',')]
                    else:
                        es.const_vehicles.append(int(row[6]))
                #start time
                if (row[7] != ''):
                    es.start_time = float(row[7])
                print("Loaded scenario:")
                print(es)
                break;
    
    if es is None:
        print("Scenario for -e {} not found".format(scenario_id))
        return None
    
    return es

def load_trajectory_log(filename):
    #vid, type, sim_state, tick_count, sim_time, delta_time,
    #state_vector[x, y, z, x_vel, y_vel, x_acc, y_acc, yaw, steer ],
    #frenet_state_vector[s, d, s_vel, d_vel, s_acc, d_acc]

    trajectory = []
    with open(filename, mode='r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            node = {}
            node['x'] = float(row[6])
            node['y'] = float(row[7])
            node['time'] = float(row[4])
            node['xvel'] = float(row[9])
            node['yvel'] = float(row[10])
            node['speed'] = sqrt(node['xvel']**2 + node['yvel'] **2)
            #acc = sqrt(xacc**2 + yacc**2)
            trajectory.append(node)
        return trajectory

def calc_frechet(traj_p, traj_q, label_p, label_q, trim = 1.0, samples = None, filename = '', title = ''):
    
    #trim ending of trajectory in %
    if trim < 1.0:
        print("trim trajector in {} of {}".format(trim, len(traj_p)))
        traj_p = traj_p[:int(len(traj_p)*trim)]
        traj_q = traj_q[:int(len(traj_q)*trim)]
        print(len(traj_p))
    
    if samples is not None:
        #sample indexes to preserve original order
        #check for maximum sample size
        sample_size = min([len(traj_p),len(traj_q),samples])
        #todo: sample time. using index only works if both are collected with in same simulation with same number of points collected
        #indexes = sorted(random.sample(range(len(traj_p)), sample_size))
        max_index = len(traj_p)-1
        indexes = np.linspace(0, max_index, sample_size,dtype = int, endpoint=False)
        P = [ [traj_p[i]['x'], traj_p[i]['y']] for i in indexes]
        Q = [ [traj_q[i]['x'], traj_q[i]['y']] for i in indexes]
    else:
        size = min([len(traj_p),len(traj_q)]) #max size
        P = [ [ node['x'],node['y'] ] for node in traj_p[:size]] 
        Q = [ [ node['x'],node['y'] ] for node in traj_q[:size]] 
    
    #Frechet distance
    score = frdist(P,Q)
    score = format(score, '.3f')
    print("{}: F = {}".format(title, score))  

    #
    if filename != '':
        plt.cla()
        plt.plot(   [node[0] for node in Q],    [node[1] for node in Q], 'r-',label=label_q)
        plt.plot(   [node[0] for node in P],    [node[1] for node in P], 'b-',label=label_p)
        plt.suptitle("{}\n{}".format(title,score ))
        plt.legend(loc="upper left")
        plt.axis('equal')
        plt.savefig(filename+'.png')
        #plt.show()

    #vel
    plt.cla()
    plt.plot(   [node['time'] for node in traj_q],    [node['speed'] for node in traj_q], 'r-',label=label_q)
    plt.plot(   [node['time'] for node in traj_p],    [node['speed'] for node in traj_p], 'b-',label=label_p)
    #plt.ylim(0,30)
    #plt.xlim(0,50)
    plt.suptitle("Speed [m/s]")
    plt.legend(loc="upper left")
    plt.savefig(filename+'_vel.png')
    
    #P=[[1,1], [2,1], [2,2]]
    #Q=[[2,2], [0,1], [2,4]]

    return score

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-s", "--scenario", dest="eval_id", default="", help="Scenario Id for evaluation")
    parser.add_argument("-t", "--type", dest="eval_type", default="", help="Type for batch evaluation")
    args = parser.parse_args()
    evaluate(args)
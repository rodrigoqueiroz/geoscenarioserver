from argparse import ArgumentParser
import csv
import numpy as np
from math import sqrt, exp, trunc, sin, cos, hypot
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
    
    #metrics:
    ed:float = 0.0 
    ed_nc:float = 0.0
    ed_change:float = 0.0
    ed_vector:list = field(default_factory=list)

    fd:float = 0.0 
    fd_nc:float = 0.0
    fd_change:float = 0.0
    



def evaluate_trajectory(traj_file):
    P = load_trajectory_log( 'trajlog/{}.csv'.format(traj_file) )   #empirical
    
    d_vector = []
    ds_vector = []
    n = len(P)
    print(n)
    for i in range(n):
        d = distance_2p( P[i]['x'], P[i]['y'], 0, 0) 
        d_vector.append([ P[i]['time'] , d ]) 
        ds_vector.append([ P[i]['time'] ,  P[i]['s'] ]) 
        
    
    plt.cla()
    #plt.plot(  [node[0] for node in d_vector] , [node[1] for node in d_vector],'k-',label="d_0")
    plt.plot(  [node[0] for node in ds_vector] , [node[1] for node in ds_vector],'b-',label="d_s")
    plt.show()    
    


def evaluate_scenario(es:EvalScenario):
    
    
    #calibrated
    traj_l = None
    if es.scenario_type == 'follow':
        lead_id = es.const_vehicles[0] #assuming single lead
        traj_l = load_trajectory_log( 'trajlog/{}_rc_{}.csv'.format(es.scenario_id, lead_id) )    #lead
    traj_e = load_trajectory_log( 'trajlog/{}_rc_{}.csv'.format(es.scenario_id, es.track_id) )   #empirical
    traj_s = load_trajectory_log( 'trajlog/{}_rc_{}.csv'.format(es.scenario_id, -es.track_id) )  #synthetic
    #not calibrated
    traj_s_nc = load_trajectory_log( 'trajlog/{}_nc_{}.csv'.format(es.scenario_id, -es.track_id) ) #synthetic nc
    traj_e_nc = load_trajectory_log( 'trajlog/{}_nc_{}.csv'.format(es.scenario_id, es.track_id) )  #empirical nc
    
    #Frechet distance
    es.fd = get_frechet(traj_s,traj_e) 
    es.fd_nc = get_frechet(traj_s_nc,traj_e_nc)
    es.ed, es.ed_vector = get_euclideandistance(traj_s,traj_e) 
    es.ed_nc, _ = get_euclideandistance(traj_s_nc,traj_e_nc) 
    
    traj_plot_combined(es, traj_s, traj_s_nc, traj_e)
    speed_plot_combined(es, traj_s, traj_s_nc, traj_e, traj_l)
    ed_plot(es)
    
    #not calibrated
    #traj_s_nc = load_trajectory_log( 'trajlog/{}_nc_{}.csv'.format(es.scenario_id, -es.track_id) ) #synthetic
    #traj_e_nc = load_trajectory_log( 'trajlog/{}_nc_{}.csv'.format(es.scenario_id, es.track_id) )
    #S_nc, E_nc = get_samples2(traj_s_nc, traj_e_nc, trim=8.0, samples = 500) #extract samples
    #es.fd_nc = get_frechet(S_nc,E_nc) #Frechet distance
    #es.ed_nc,_ = get_euclideandistance(traj_s_nc,traj_e_nc) #Frechet distance
    #save_traj_plot(S_nc,E_nc, es, es.fd_nc, es.ed_nc, 'nc')
    #save_speed_plot(S_nc,E_nc, L, es, 'nc')
    
    es.ed_change = (es.ed - es.ed_nc)/es.ed_nc*100
    es.fd_change = (es.fd - es.fd_nc)/es.fd_nc*100

    #print("Experiment {} Frechet distance = {} (nc) {} (rc) Euclidean distance = {} (nc) {} (rc)".format(
    #    es.scenario_id, es.fd_nc,es.fd_rc,es.ed_nc,es.ed_rc))
    
    
    
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
                    print("Vehicle {} cannot be used for evaluation".format(scenario_id))
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
     #'id', 'type','sim_state', 'tick_count', 'sim_time', 'delta_time',
     #'x', 'x_vel', 'x_acc', 'y',  'y_vel', 'y_acc', 's', 's_vel', 's_acc','d', 'd_vel', 'd_acc', 'angle'

    trajectory = []
    with open(filename, mode='r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if row[0] == 'id':
                continue
            node = {}
            node['time'] = float(row[4])
            node['x'] = float(row[6])
            node['xvel'] = float(row[7])
            node['y'] = float(row[9])
            node['yvel'] = float(row[10])
            node['speed'] = sqrt(node['xvel']**2 + node['yvel'] **2)
            node['s'] = float(row[12])
            trajectory.append(node)
        return trajectory


def get_samples2(traj_s, traj_e, trim = 1.0, samples = None):
    
    E, S, L = None, None, None

    #trim ending of trajectory in %
    #if trim < 1.0:
    #    print("trim trajector in {} of {}".format(trim, len(traj_p)))
    #    traj_p = traj_p[:int(len(traj_p)*trim)]
    #    traj_q = traj_q[:int(len(traj_q)*trim)]
    #    if traj_l:
    #        traj_l = traj_l[:int(len(traj_l)*trim)]
        #print(len(traj_p))
    
    if samples is not None:
        #sample indexes to preserve original order
        #check for maximum sample size
        sample_size = min([len(traj_e),samples])
        #todo: sample time. using index only works if both are collected with in same simulation with same number of points collected
        #indexes = sorted(random.sample(range(len(traj_p)), sample_size))
        max_index = len(traj_e)-1
        indexes = np.linspace(0, max_index, sample_size,dtype = int, endpoint=False)
        E = [ traj_e[i]  for i in indexes]
        S = [ traj_s[i]  for i in indexes]
    else:
        size = min([len(traj_s),len(traj_e)]) #max size
        E = [ node for node in traj_e[:size]] 
        S = [ node for node in traj_s[:size]] 
    #    if traj_l:
    #        L =  [ node for node in traj_l[:size]]
    return E, S, L

def get_samples(traj_p, traj_q, traj_l = None, trim = 1.0, samples = None):
    
    L = None

    #trim ending of trajectory in %
    if trim < 1.0:
        print("trim trajector in {} of {}".format(trim, len(traj_p)))
        traj_p = traj_p[:int(len(traj_p)*trim)]
        traj_q = traj_q[:int(len(traj_q)*trim)]
        if traj_l:
            traj_l = traj_l[:int(len(traj_l)*trim)]
        #print(len(traj_p))
    
    if samples is not None:
        #sample indexes to preserve original order
        #check for maximum sample size
        sample_size = min([len(traj_p),len(traj_q),samples])
        #todo: sample time. using index only works if both are collected with in same simulation with same number of points collected
        #indexes = sorted(random.sample(range(len(traj_p)), sample_size))
        max_index = len(traj_p)-1
        indexes = np.linspace(0, max_index, sample_size,dtype = int, endpoint=False)
        P = [ traj_p[i]  for i in indexes]
        Q = [ traj_q[i]  for i in indexes]
        if traj_l:
            L = [ traj_l[i]  for i in indexes]
    else:
        size = min([len(traj_p),len(traj_q)]) #max size
        P = [ node for node in traj_p[:size]] 
        Q = [ node for node in traj_q[:size]] 
        if traj_l:
            L =  [ node for node in traj_l[:size]]
    return P, Q, L

def get_frechet(traj_p,traj_q, trim = 1.0, samples = None):
    L = None
    #trim ending of trajectory in %
    print("trim trajectory in {} of {}".format(trim, len(traj_p)))
    P = traj_p[:int(len(traj_p)*trim)]
    Q = traj_q[:int(len(traj_q)*trim)]
    if samples is not None:
        #sample indexes to preserve original order
        #check for maximum sample size
        sample_size = min([len(P),len(Q),samples])
        #todo: sample time. using index only works if both are collected with in same simulation with same number of points collected
        #indexes = sorted(random.sample(range(len(traj_p)), sample_size))
        max_index = len(P)-1
        indexes = np.linspace(0, max_index, sample_size,dtype = int, endpoint=False)
        SP = [ P[i]  for i in indexes]
        SQ = [ Q[i]  for i in indexes]
    else:
        size = min([len(P),len(Q)]) #max size
        SP = [ node for node in P[:size]] 
        SQ = [ node for node in Q[:size]] 

    Plist = [ [ node['x'], node['y']] for node in SP]
    Qlist = [ [ node['x'], node['y']] for node in SQ]
    score = frdist(Plist,Qlist)
    return score

def get_euclideandistance(P,Q = None):
    score = 0
    d_vector = []
    
    n = len(P)
    for i in range(n):
        d = distance_2p( P[i]['x'], P[i]['y'], Q[i]['x'],Q[i]['y']) 
        d_vector.append([ P[i]['time'] , d ])
        score += d
    score = score/n
    return score, d_vector

def distance_2p(x1,y1,x2,y2):
    #dist = hypot(x2 - x1, y2 - y1)
    dist = sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
    return dist



def traj_plot_combined(es:EvalScenario, traj_s, traj_s_nc, traj_e):
    plt.cla()
    fig=plt.figure()
    ax=fig.add_subplot(111)
    ax.plot(   [node['x'] for node in traj_e],    
                [node['y'] for node in traj_e], 
                'r-', label="emp")

    ax.plot(   [node['x'] for node in traj_s_nc],    
                [node['y'] for node in traj_s_nc], 
                'b--',label="synth Ed={} m Fd={} m (A)".format(format(es.ed_nc, '.2f'),format(es.fd_nc, '.2f')))

    ax.plot(   [node['x'] for node in traj_s],    
                [node['y'] for node in traj_s], 
                'b-',label="synth Ed={} m Fd={} m (B)".format(format(es.ed, '.2f'),format(es.fd, '.2f')))
    
    box = ax.get_position()
    ax.set_position([box.x0, box.y0 + box.height * 0.1, box.width, box.height * 0.9])
    ax.legend(loc="upper center",bbox_to_anchor=(0.5, -0.05))
    plt.suptitle('Experiment {} ({}) \nTrajectory'.format(es.scenario_id, es.scenario_type))
    plt.ylabel('y (m)')
    plt.xlabel('x (m)')
    
    plt.axis('equal')
    filename = 'plots/{}/{}_trajectory'.format(es.scenario_type, es.scenario_id)
    plt.savefig(filename+'.png')
    plt.close(fig)
    

def speed_plot_combined(es:EvalScenario, traj_s, traj_s_nc, traj_e, traj_l, ymin=0, ymax=20):
    plt.cla()
    fig=plt.figure()
    ax=fig.add_subplot(111)

    if traj_l:
        ax.plot(   [node['time'] for node in traj_l],    
                    [node['speed'] for node in traj_l], 
                    'k--',label="lead")
    ax.plot(   [node['time'] for node in traj_e],    
                [node['speed'] for node in traj_e], 
                'r-',label="emp")
    ax.plot(   [node['time'] for node in traj_s_nc],    
                [node['speed'] for node in traj_s_nc], 
                'b--',label="synth A")
    ax.plot(   [node['time'] for node in traj_s],    
                [node['speed'] for node in traj_s], 
                'b-',label="synth B")
    plt.ylabel('speed/n(m/s)')
    plt.xlabel('time (s)')
    
    xmin = min([node['time'] for node in traj_e])
    xmax = max([node['time'] for node in traj_e])
    
    ax.set_xbound(lower=xmin, upper=xmax)
    ax.set_ybound(lower=ymin, upper=ymax)
    #plt.xlim(xmin,xmax)
    #plt.ylim(0,30)
    #plt.yticks(range(0, 30))
    #plt.axis.set_aspect('auto')
    plt.suptitle('Experiment {} ({}) \nSpeed (m/s)'.format(es.scenario_id, es.scenario_type))
    plt.legend(loc="upper left")
    filename = 'plots/{}/{}_speed'.format(es.scenario_type, es.scenario_id)
    plt.savefig(filename+'.png')
    plt.close(fig)

def ed_plot(es):
    filename = 'plots/{}/{}_edvector'.format(es.scenario_type, es.scenario_id)
    title = 'Experiment {} ({}) \n ED {} m'.format(es.scenario_id, es.scenario_type, format(es.ed, '.2f'))
    plt.cla()
    plt.plot(  [node[0] for node in es.ed_vector] , [node[1] for node in es.ed_vector],'k-',label="ed")
    plt.suptitle(title)
    plt.legend(loc="upper left")
    plt.savefig(filename+'.png')
    #plt.show()    
    
def update_results_table(es):
    #Write results to file
    with open('results.csv', mode='r',encoding='utf-8-sig')as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        lines = list(csv_reader)
        found = False
        for l in lines:
            if l[0] == es.scenario_id:
                l[1] = es.scenario_type
                l[2] = format(es.ed_nc, '.2f')
                l[3] = format(es.ed, '.2f')
                l[4] = format(es.ed_change, '.2f')
                l[5] = format(es.fd_nc, '.2f')
                l[6] = format(es.fd, '.2f')
                l[7] = format(es.fd_change, '.2f')
                found = True
        if not found:
            lines.append([es.scenario_id,
                        es.scenario_type,
                        format(es.ed_nc, '.2f'),
                        format(es.ed, '.2f'), 
                        format(es.ed_change, '.2f'),
                        format(es.fd_nc, '.2f'),
                        format(es.fd, '.2f'), 
                        format(es.fd_change, '.2f') ])
                                
    with open('results.csv', mode='w',encoding='utf-8-sig') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',')
        csv_writer.writerows(lines)

    return lines

def generate_boxplots(lines):
    i_ed_nc = 2
    i_ed = 3
    i_fd = 5
    i_fd_nc = 6
    datasize = len(lines)-1
    #all scenarios
    data_fd_nc =    [ float(lines[i][i_fd_nc]) for i in range(len(lines)) if lines[i][0] != 'sid' ]
    data_fd =       [ float(lines[i][i_fd]) for i in range(len(lines)) if lines[i][0] != 'sid' ]
    data_ed_nc =    [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][0] != 'sid' ]
    data_ed =       [ float(lines[i][i_ed]) for i in range(len(lines)) if lines[i][0] != 'sid' ]
    
    data_ed_type = [
            [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][1] == 'free' ],
            [ float(lines[i][i_ed])    for i in range(len(lines)) if lines[i][1] == 'free' ],
            [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][1] == 'follow' ],
            [ float(lines[i][i_ed])    for i in range(len(lines)) if lines[i][1] == 'follow' ],
            [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][1] == 'glstart' ],
            [ float(lines[i][i_ed])    for i in range(len(lines)) if lines[i][1] == 'glstart' ],
            [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][1] == 'rlstop' ],
            [ float(lines[i][i_ed])    for i in range(len(lines)) if lines[i][1] == 'rlstop' ]
    ]
    
    data_fd_type = [
            [ float(lines[i][i_fd_nc]) for i in range(len(lines)) if lines[i][1] == 'free' ],
            [ float(lines[i][i_fd])    for i in range(len(lines)) if lines[i][1] == 'free' ],
            [ float(lines[i][i_fd_nc]) for i in range(len(lines)) if lines[i][1] == 'follow' ],
            [ float(lines[i][i_fd])    for i in range(len(lines)) if lines[i][1] == 'follow' ],
            [ float(lines[i][i_fd_nc]) for i in range(len(lines)) if lines[i][1] == 'glstart' ],
            [ float(lines[i][i_fd])    for i in range(len(lines)) if lines[i][1] == 'glstart' ],
            [ float(lines[i][i_fd_nc]) for i in range(len(lines)) if lines[i][1] == 'rlstop' ],
            [ float(lines[i][i_fd])    for i in range(len(lines)) if lines[i][1] == 'rlstop' ]
    ]
    
    #boxplot
    #plt.cla()
    title = "Euclidean distance (ED) and Fréchet distance (FD) distribution.\n{} scenarios".format(datasize)
    data = [data_ed_nc, data_ed, data_fd_nc, data_fd]
    filename = 'boxplot_combined.png'    
    fig = plt.figure() 
    ax = fig.add_subplot(111)
    bp = ax.boxplot(data) 
    ax.set_xticklabels(['ED (a)', 'ED (b)','FD (a)', 'FD (b)' ]) 
    ax.get_xaxis().tick_bottom() 
    ax.get_yaxis().tick_left() 
    plt.title(title) 
    plt.savefig(filename)
    plt.close(fig)
    

    #boxplot combined ED
    title = "Euclidean distance (ED) distribution per scenario type"
    filename = 'boxplot_type_ed.png'    
    #plt.cla()
    fig = plt.figure() 
    ax = fig.add_subplot(111)
    bp = ax.boxplot(data_ed_type) 
    ax.set_xticklabels(['follow (a)', 'follow (b)',
                        'free (a)','free (b)',
                        'glstart (a)','glstart (b)',
                        'rlstop (a)', 'rlstop (b)']) 
    ax.get_xaxis().tick_bottom() 
    ax.get_yaxis().tick_left() 
    plt.title(title) 
    plt.savefig(filename)
    plt.close(fig)
    #plt.show()

    #boxplot combined FD
    title = "Fréchet distance (FD) distribution per scenario type"
    filename = 'boxplot_type_fd.png'    
    #plt.cla()
    fig = plt.figure() 
    ax = fig.add_subplot(111)
    bp = ax.boxplot(data_fd_type) 
    ax.set_xticklabels(['follow (a)', 'follow (b)',
                        'free (a)','free (b)',
                        'glstart (a)','glstart (b)',
                        'rlstop (a)', 'rlstop (b)']) 
    ax.get_xaxis().tick_bottom() 
    ax.get_yaxis().tick_left() 
    plt.title(title) 
    plt.savefig(filename)
    plt.close(fig)
    #plt.show()

def save_traj_plot(P, Q, es, fd_score, ed_score, config_str):
    filename = 'plots/{}/{}_{}'.format(es.scenario_type, es.scenario_id, config_str)
    title = 'Experiment {} ({}) \n FD ={} m, ED {} m'.format(es.scenario_id,config_str, format(fd_score, '.3f'),format(ed_score, '.3f'))
    plt.cla()
    plt.plot(   [node['x'] for node in Q],    [node['y'] for node in Q], 'r-',label="emp")
    plt.plot(   [node['x'] for node in P],    [node['y'] for node in P], 'b-',label="synth")
    plt.suptitle(title)
    plt.legend(loc="upper left")
    plt.axis('equal')
    plt.savefig(filename+'.png')
    #plt.show()

def save_speed_plot(P, Q, L, es, config_str):
    filename = 'plots/{}/{}_{}_speed'.format(es.scenario_type, es.scenario_id, config_str)
    title = 'Experiment {} ({}) \nSpeed (m/s)'.format(es.scenario_id,config_str)

    #vel
    plt.cla()
    if L:
        plt.plot(   [node['time'] for node in L],    [node['speed'] for node in L], 'k--',label="lead")
    plt.plot(   [node['time'] for node in Q],    [node['speed'] for node in Q], 'r-',label="emp")
    plt.plot(   [node['time'] for node in P],    [node['speed'] for node in P], 'b-',label="synth")
    plt.ylabel('speed (m/s)')
    plt.xlabel('time (s)')
    #plt.ylim(0,20)
    #plt.xlim(0,50)
    plt.suptitle(title)
    plt.legend(loc="upper left")
    plt.savefig(filename+'.png')

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-s", "--scenario", dest="eval_id", default="", help="Scenario Id for evaluation")
    parser.add_argument("-t", "--type", dest="eval_type", default="", help="Type for batch evaluation")
    parser.add_argument("-tf", "--traj_file", dest="traj_file", default="", help="Single Trajectory analysis")
    parser.add_argument("-a", "--all", dest="eval_all", action="store_true", help="Batch evaluation for all trajectories")
    args = parser.parse_args()
    
    scenarios = load_all_scenarios()

    #All scenarios
    if args.eval_all:
        for id in scenarios:
            try:
                evaluate_scenario(scenarios[id])
                results = update_results_table(scenarios[id])
            except Exception as e:
                print("ERROR. Can not run evaluation for scenario{}".format(id))
        generate_boxplots(results)
    #All scenarios from type
    elif args.eval_type != "":
        for id in scenarios:
            if scenarios[id].scenario_type == args.eval_type:
                try:
                    evaluate_scenario(scenarios[id])
                    results = update_results_table(scenarios[args.eval_id])
                except Exception as e:
                    print("ERROR. Can not run evaluation for scenario{}".format(id))
        generate_boxplots(results)
    #One scenario
    elif args.eval_id != "":
        try:
            evaluate_scenario(scenarios[args.eval_id])
            results = update_results_table(scenarios[args.eval_id])
            generate_boxplots(results)
        except Exception as e:
            print("ERROR. Can not run evaluation for scenario{}".format(args.eval_id))
            raise e

    #evaluate single trajectory
    elif args.traj_file != "":
        evaluate_trajectory(args.traj_file)
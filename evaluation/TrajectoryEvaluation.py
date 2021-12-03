from argparse import ArgumentParser
import csv
import numpy as np
from math import sqrt, exp, trunc, sin, cos, hypot
import random
from dataclasses import dataclass, field
from matplotlib import pyplot as plt
from frechetdist import frdist
import time
from pathlib import Path
from scipy.signal import butter, filtfilt
from scipy.spatial.distance import directed_hausdorff

TIMESTR = time.strftime("%m%d_%H%M")

@dataclass
class EvalScenario:
    scenario_id:str = ''
    video_id:str = ''
    scenario_length:str = ''
    map_location:str = ''
    track_id:int = 0
    agent_type:str = ''                                 #Car, Pedestrian, Medium Vehicle, Heavy Vehicle
    scenario_type:str = ''                              #free, follow, free_follow, rlstop, glstart, yield_turnright, yield_turnleft, lcleft, lcright
    const_agents:list = field(default_factory=list)     #vehicles/pedestrians that must be in the scene (e.g: following, or yielding)
    start_time:float = 0.0                              #scenario start time
    direction:str = ''                                  #(straight) n_s, s_n, e_w, w_e (left turns) s_w, w_n (right turn) s_e, w_s, (not supported) e_n, n_w

    #metrics:
    time:float = 0.0
    ed:float = 0.0
    ed_vector:list = field(default_factory=list)
    ed_mean:float = 0.0
    ed_median:float = 0.0
    ed_max:float = 0.0
    ed_change:float = 0.0

    ade:float = 0.0

    fd:float = 0.0
    fd_change:float = 0.0

    hd:float = 0.0

    curvature_score:float = 0.0



def evaluate_trajectory(video_id, map_location, traj_file, scenario_length):
    P = load_trajectory_log('evaluation/traj_log/{}/{}/{}/{}.csv'.format(map_location, scenario_length, video_id, traj_file) )   #empirical

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



def evaluate_scenario(es:EvalScenario, map_lines):
    print("Evaluate scenario {}".format(es.scenario_id))

    traj_s = load_trajectory_log('evaluation/traj_log/{}/{}/{}/{}_nc_{}.csv'.format(es.map_location, es.scenario_length, es.video_id, es.scenario_id, -es.track_id)) #synthetic nc
    traj_e = load_trajectory_log('evaluation/traj_log/{}/{}/{}/{}_nc_{}.csv'.format(es.map_location, es.scenario_length, es.video_id, es.scenario_id, es.track_id))  #empirical nc

    es.time = float(traj_e[-1]['time']) - float(traj_e[0]['time'])

    # Euclidean distance
    es.ed_vector, es.ed = get_euclideandistance(traj_s, traj_e)
    ed_array = [x[1] for x in es.ed_vector]
    es.ed_mean = np.mean(ed_array)
    es.ed_median = np.median(ed_array)
    es.ed_max = np.max(ed_array)

    # get list of trajectory nodes without time dependency
    s_list = [np.array([node['x'], node['y']]) for node in traj_s]
    e_list = [np.array([node['x'], node['y']]) for node in traj_e]

    # Average Displacement Error
    es.ade = get_avgdisplacementerror(s_list, e_list)

    # Frechet distance
    es.fd = get_frechet(s_list, e_list)

    # Hausdorff distance
    es.hd = get_hausdorff(s_list, e_list)

    path_curvatures = get_curvature_of_paths(traj_s, traj_e)
    plot_curvatures(es, path_curvatures)
    es.curvature_score = get_curvature_matching_score(path_curvatures)

    traj_plot_combined(es, traj_s, traj_e, map_lines)
    #speed_plot_combined(es, traj_s, traj_s_nc, traj_e, traj_l)
    ed_plot(es)

    #not calibrated
    #traj_s_nc = load_trajectory_log( 'trajlog/{}_nc_{}.csv'.format(es.scenario_id, -es.track_id) ) #synthetic
    #traj_e_nc = load_trajectory_log( 'trajlog/{}_nc_{}.csv'.format(es.scenario_id, es.track_id) )
    #S_nc, E_nc = get_samples2(traj_s_nc, traj_e_nc, trim=8.0, samples = 500) #extract samples
    #es.fd_nc = get_frechet(S_nc,E_nc) #Frechet distance
    #es.ed_nc,_ = get_euclideandistance(traj_s_nc,traj_e_nc) #Frechet distance
    #save_traj_plot(S_nc,E_nc, es, es.fd_nc, es.ed_nc, 'nc')
    #save_speed_plot(S_nc,E_nc, L, es, 'nc')

    #es.ed_change = (es.ed - es.ed_nc)/es.ed_nc*100
    #es.fd_change = (es.fd - es.fd_nc)/es.fd_nc*100



    #print("Experiment {} Frechet distance = {} (nc) {} (rc) Euclidean distance = {} (nc) {} (rc)".format(
    #    es.scenario_id, es.fd_nc,es.fd_rc,es.ed_nc,es.ed_rc))

def load_all_scenarios(video_id, map_location, scenario_length):
    scenarios = {}
    with open('evaluation/pedestrian_scenarios/{}/{}/pedestrian_scenarios_{}.csv'.format(map_location, scenario_length, video_id), mode='r', encoding='utf-8-sig') as csv_file:
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
            es.video_id = video_id
            es.map_location = map_location
            es.scenario_length = scenario_length
            es.track_id = int(row[2])
            es.agent_type = row[3]
            es.direction = row[4]
            es.scenario_type = row[5].split(";")
            #constraints
            if (row[6] != ''):
                if (';' in row[6]):
                    es.const_agents = [int(cid) for cid in row[6].split(';')]
                else:
                    es.const_agents.append(int(row[6]))
            #start time
            if (row[7] != ''):
                es.start_time = float(row[7])
            if (row[8] != ''):
                    es.end_time = float(row[8])
            scenarios[es.scenario_id] = es
    return scenarios

def load_scenario_db(scenario_id):
    es = None
    with open('evaluation/pedestrian_scenarios.csv', mode='r', encoding='utf-8-sig') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if row[0] == "scenario_id": #header
                continue
            if row[0]==scenario_id:
                #int(eval_vid):
                #exclusions
                if row[1] == 'x':
                    print("Pedestrian {} cannot be used for evaluation".format(scenario_id))
                    return None
                es = EvalScenario()
                es.scenario_id = scenario_id
                es.track_id = int(row[2])
                es.agent_type = row[3]
                es.direction = row[4]
                es.scenario_type = row[5].split(";")
                #constraints
                if (row[6] != ''):
                    if (',' in row[6]):
                        es.const_agents = [int(cvid) for cvid in row[6].split(';')]
                    else:
                        es.const_agents.append(int(row[6]))
                #start time
                if (row[7] != ''):
                    es.start_time = float(row[7])
                print("Loaded scenario:")
                print(es)
                break

    if es is None:
        print("Scenario for -e {} not found".format(scenario_id))
        return None

    return es

def load_trajectory_log(filename):
     #'id', 'agent_type', 'type', 'sim_state', 'tick_count', 'sim_time', 'delta_time',
     #'x', 'x_vel', 'x_acc', 'y', 'y_vel', 'y_acc', 's', 's_vel', 's_acc', 'd', 'd_vel', 'd_acc', 'yaw'

    trajectory = []
    with open(filename, mode='r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        next(csv_reader, None)  # skip the header
        for row in csv_reader:
            node = {}
            node['time'] = float(row[5])
            node['x'] = float(row[7])
            node['xvel'] = float(row[8])
            node['y'] = float(row[10])
            node['yvel'] = float(row[11])
            node['speed'] = sqrt(node['xvel']**2 + node['yvel'] **2)
            node['s'] = float(row[13])
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

def get_frechet(P, Q, trim = 1.0, samples=494):
    #trim ending of trajectory in %
    P = P[:int(len(P)*trim)]
    Q = Q[:int(len(Q)*trim)]

    if samples is not None:
        #sample indexes to preserve original order
        #check for maximum sample size
        sample_size = min([len(P), len(Q), samples])
        #todo: sample time. using index only works if both are collected with in same simulation with same number of points collected
        #indexes = sorted(random.sample(range(len(traj_p)), sample_size))
        max_index = min([len(P), len(Q)]) - 1
        indexes = np.linspace(0, max_index, sample_size, dtype=int, endpoint=False)
        SP = [P[i] for i in indexes]
        SQ = [Q[i] for i in indexes]
    else:
        size = min([len(P),len(Q)]) #max size
        SP = [node for node in P[:size]]
        SQ = [node for node in Q[:size]]

    score = frdist(SP, SQ)
    return score


def get_hausdorff(P, Q):
    size = min([len(P),len(Q)]) #max size
    SP = [node for node in P[:size]]
    SQ = [node for node in Q[:size]]

    return max(directed_hausdorff(SP, SQ)[0], directed_hausdorff(SQ, SP)[0])


def get_curvature_of_paths(traj_s, traj_e):
    curvature_vals = []

    for path in [traj_s, traj_e]:
        x_vals = [node['x'] for node in path]
        y_vals = [node['y'] for node in path]
        time_vals = [node['time'] for node in path]
        dt = path[1]['time'] - path[0]['time']
        two_dt = 2*dt

        x_grad = [0]
        y_grad = [0]
        x_grad2 = [0]
        y_grad2 = [0]
        curvature = []

        for i in range(1, len(x_vals)-1):
            x_grad.append((x_vals[i+1] - x_vals[i-1]) / (time_vals[i+1] - time_vals[i-1]))
        x_grad[0] = (x_vals[1] - x_vals[0]) / (time_vals[1] - time_vals[0])
        x_grad.append((x_vals[-1] - x_vals[-2]) / (time_vals[-1] - time_vals[-2]))

        for i in range(1, len(y_vals)-1):
            y_grad.append((y_vals[i+1] - y_vals[i-1]) / (time_vals[i+1] - time_vals[i-1]))
        y_grad[0] = (y_vals[1] - y_vals[0]) / (time_vals[1] - time_vals[0])
        y_grad.append((y_vals[-1] - y_vals[-2]) / (time_vals[-1] - time_vals[-2]))

        for i in range(1, len(x_grad)-1):
            x_grad2.append((x_grad[i+1] - x_grad[i-1]) / (time_vals[i+1] - time_vals[i-1]))
        x_grad2[0] = (x_grad[1] - x_grad[0]) / (time_vals[1] - time_vals[0])
        x_grad2.append((x_grad[-1] - x_grad[-2]) / (time_vals[-1] - time_vals[-2]))

        for i in range(1, len(y_grad)-1):
            y_grad2.append((y_grad[i+1] - y_grad[i-1]) / (time_vals[i+1] - time_vals[i-1]))
        y_grad2[0] = (y_grad[1] - y_grad[0]) / (time_vals[1] - time_vals[0])
        y_grad2.append((y_grad[-1] - y_grad[-2]) / (time_vals[-1] - time_vals[-2]))

        for i in range(len(y_vals)):
            if x_grad[i] == 0 and y_grad[i] == 0:
                curvature.append(0)
            else:
                curvature.append((x_grad[i]*y_grad2[i] - y_grad[i]*x_grad2[i]) / ((x_grad[i]**2 + y_grad[i]**2)**1.5))

        curvature_vals.append(low_pass_filter(curvature))

    return curvature_vals

def low_pass_filter(vals):
    # Filter requirements
    fs = 30.0       # sample rate, Hz
    cutoff = 1      # desired cutoff frequency of the filter, Hz
    nyq = 0.5 * fs  # Nyquist Frequency
    order = 4       # polynomial order of the signal (number of crosswalks taken * 2)

    normal_cutoff = cutoff / nyq

    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, vals)

    return y


def get_curvature_matching_score(path_curvatures):
    path_length_t = min(len(path_curvatures[0]), len(path_curvatures[1]))
    curvature_score = abs(path_curvatures[1][0] - path_curvatures[0][0])

    for i in range(1, path_length_t):
        curvature_diff = abs(path_curvatures[1][i] - path_curvatures[0][i])
        if curvature_diff > curvature_score:
            curvature_score = curvature_diff

    return curvature_score


def plot_curvatures(es:EvalScenario, curvature):
    for scenario_tag in es.scenario_type:
        Path("evaluation/plots/{}/{}/{}/{}".format(es.map_location, es.scenario_length, scenario_tag, es.video_id)).mkdir(parents=True, exist_ok=True)
        plt.cla()

        fig, ax = plt.subplots()

        ax.plot(range(len(curvature[0])),
                curvature[0],
                'b-', label="Model pedestrian curvature")

        ax.plot(range(len(curvature[1])),
                curvature[1],
                'r-', label="Empirical pedestrian curvature")

        box = ax.get_position()
        ax.set_position([box.x0, box.y0 + box.height * 0.2, box.width, box.height * 0.8])
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.15))
        plt.suptitle('Curvature of model and empirical paths')
        plt.ylabel('y')
        plt.xlabel('x')

        filename = 'evaluation/plots/{}/{}/{}/{}/{}_curvature'.format(es.map_location, es.scenario_length, scenario_tag, es.video_id, es.scenario_id)
        plt.savefig(filename+'.png')
        plt.close(fig)

def get_euclideandistance(P, Q = None):
    d_vector = []

    n = min(len(P), len(Q))
    dt = 0
    total = 0

    for i in range(n):
        d = distance_2p(P[i]['x'], P[i]['y'], Q[i]['x'], Q[i]['y'])
        d_vector.append([P[i]['time'], d])
        if i < (n-1):
            dt = P[i+1]['time'] - P[i]['time']
            total += d*dt
    score = total / (P[-1]['time'] - P[0]['time'])
    return d_vector, score


def get_avgdisplacementerror(P, Q):
    n = min([len(P),len(Q)]) #max size

    ADE = sum([np.linalg.norm(P[i] - Q[i])**2 for i in range(n)]) / n
    return ADE


def distance_2p(x1, y1, x2, y2):
    dist = sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist


def traj_plot_combined(es:EvalScenario, traj_s, traj_e, map_lines):
    # create directory for plots if it does not exist
    for scenario_tag in es.scenario_type:
        Path("evaluation/plots/{}/{}/{}/{}".format(es.map_location, es.scenario_length, scenario_tag, es.video_id)).mkdir(parents=True, exist_ok=True)

        plt.cla()
        fig=plt.figure()

        ax=fig.add_subplot(111)

        # plot road
        for line in map_lines:
            ax.plot(line[0], line[1], color='#cccccc', zorder=0)

        ax.plot(   [node['x'] for node in traj_e],
                    [node['y'] for node in traj_e],
                    'r-', label="Empirical pedestrian")

        ax.plot(   [node['x'] for node in traj_s],
                    [node['y'] for node in traj_s],
                    'b-',label="SP avg ed={} m, max ed={} m".format(format(es.ed_mean, '.2f'),format(es.ed_max, '.2f')))

        #fig.subplots_adjust(bottom=0.05)
        box = ax.get_position()
        ax.set_position([box.x0, box.y0 + box.height * 0.2, box.width, box.height * 0.8])
        ax.legend(loc="upper center",bbox_to_anchor=(0.5, -0.15))
        plt.suptitle('Experiment {} ({}) \nTrajectory'.format(es.scenario_id, scenario_tag))
        plt.ylabel('y (m)')
        plt.xlabel('x (m)')

        plt.axis('equal')
        filename = 'evaluation/plots/{}/{}/{}/{}/{}_trajectory'.format(es.map_location, es.scenario_length, scenario_tag, es.video_id, es.scenario_id)
        plt.savefig(filename+'.png')
        plt.close(fig)


def speed_plot_combined(es:EvalScenario, traj_s, traj_s_nc, traj_e, traj_l, ymin=-1, ymax=20):
    for scenario_tag in es.scenario_type:
        plt.cla()
        fig=plt.figure()
        ax=fig.add_subplot(111)

        if traj_l:
            ax.plot(   [node['time'] for node in traj_l],
                        [node['speed'] for node in traj_l],
                        'k--',label="Lead")
        ax.plot(   [node['time'] for node in traj_e],
                    [node['speed'] for node in traj_e],
                    'r-',label="Empirical")
        ax.plot(   [node['time'] for node in traj_s_nc],
                    [node['speed'] for node in traj_s_nc],
                    'b--',label="SDV (a)")
        ax.plot(   [node['time'] for node in traj_s],
                    [node['speed'] for node in traj_s],
                    'b-',label="SDV (b)")
        plt.ylabel('speed (m/s)')
        plt.xlabel('time (s)')

        xmin = min([node['time'] for node in traj_e])
        xmax = max([node['time'] for node in traj_e])

        ax.set_xbound(lower=xmin, upper=xmax)
        ax.set_ybound(lower=ymin, upper=ymax)
        #plt.xlim(xmin,xmax)
        #plt.ylim(0,30)
        #plt.yticks(range(0, 30))
        #plt.axis.set_aspect('auto')
        plt.suptitle('Experiment {} ({}) \nSpeed (m/s)'.format(es.scenario_id, scenario_tag))
        plt.legend(loc="upper left")
        filename = 'evaluation/plots/{}/{}/{}/{}/{}_speed'.format(es.map_location, es.scenario_length, scenario_tag, es.video_id, es.scenario_id)
        plt.savefig(filename+'.png')
        plt.close(fig)


def ed_plot(es):
    for scenario_tag in es.scenario_type:
        Path("evaluation/plots/{}/{}/{}/{}".format(es.map_location, es.scenario_length, scenario_tag, es.video_id)).mkdir(parents=True, exist_ok=True)
        filename = 'evaluation/plots/{}/{}/{}/{}/{}_edvector'.format(es.map_location, es.scenario_length, scenario_tag, es.video_id, es.scenario_id)
        title = 'Experiment {} ({}) \n ED {} m'.format(es.scenario_id, scenario_tag, format(es.ed_mean, '.2f'))
        plt.cla()
        plt.plot([node[0] for node in es.ed_vector], [node[1] for node in es.ed_vector], 'k-', label="ed")
        plt.suptitle(title)
        plt.legend(loc="upper left")
        plt.savefig(filename+'.png')


def update_results_table(es:EvalScenario):
    print("Update results for scenario {}".format(es.scenario_id))
    #Write results to file
    with open('evaluation/results/{}/results_{}.csv'.format(es.map_location, es.scenario_length), mode='r', encoding='utf-8-sig') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        lines = list(csv_reader)

        found = False
        for l in lines:
            if l[0] == es.video_id and l[1] == es.scenario_id:
                l[2] = ';'.join(es.scenario_type)
                l[3] = es.time
                #nc
                l[4] = format(es.ed_mean, '.2f')
                l[5] = format(es.ed_median, '.2f')
                l[6] = format(es.ed_max, '.2f')
                l[7] = format(es.ed, '.2f')

                l[8] = format(es.ade, '.2f')
                l[9] = format(es.fd, '.2f')
                l[10] = format(es.hd, '.2f')
                #l[8] = format(es.ed_change, '.2f')
                #l[9] = format(es.fd_nc, '.2f')
                #l[10] = format(es.fd, '.2f')
                #l[11] = format(es.fd_change, '.2f')
                found = True

        if not found:
            lines.append([es.video_id,
                        es.scenario_id,
                        ';'.join(es.scenario_type),
                        es.time,
                        format(es.ed_mean, '.2f'),
                        format(es.ed_median, '.2f'),
                        format(es.ed_max, '.2f'),
                        format(es.ed, '.2f'),

                        format(es.ade, '.2f'),
                        format(es.fd, '.2f'),
                        format(es.hd, '.2f')
                        #format(es.ed_change, '.2f')
                        #format(es.fd_nc, '.2f'),
                        #format(es.fd, '.2f'),
                        #format(es.fd_change, '.2f')
                        ])

    with open('evaluation/results/{}/results_{}.csv'.format(es.map_location, es.scenario_length), mode='w',encoding='utf-8-sig') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',')
        csv_writer.writerows(lines)

    return lines

def generate_boxplots(lines):
    print("generating boxplots")
    i_ed_nc = 6
    i_ed = 10
    #i_fd = 5
    #i_fd_nc = 6
    datasize = len(lines)-1
    #all scenarios
    #data_fd_nc =    [ float(lines[i][i_fd_nc]) for i in range(len(lines)) if lines[i][0] != 'sid' ]
    #data_fd =       [ float(lines[i][i_fd]) for i in range(len(lines)) if lines[i][0] != 'sid' ]
    data_ed_nc =    [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][0] != 'sid' ]
    data_ed =       [ float(lines[i][i_ed]) for i in range(len(lines)) if lines[i][0] != 'sid' ]

    data_ed_type = [
            [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][1] == 'free' ],
            [ float(lines[i][i_ed])    for i in range(len(lines)) if lines[i][1] == 'free' ],
            [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][1] == 'follow' ],
            [ float(lines[i][i_ed])    for i in range(len(lines)) if lines[i][1] == 'follow' ],
            [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][1] == 'free/follow' ],
            [ float(lines[i][i_ed])    for i in range(len(lines)) if lines[i][1] == 'free/follow' ],
            [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][1] == 'green light' ],
            [ float(lines[i][i_ed])    for i in range(len(lines)) if lines[i][1] == 'green light' ],
            [ float(lines[i][i_ed_nc]) for i in range(len(lines)) if lines[i][1] == 'red light' ],
            [ float(lines[i][i_ed])    for i in range(len(lines)) if lines[i][1] == 'red light' ]
    ]
    """
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
    """

    #boxplot
    #plt.cla()
    title = "Euclidean distance (ED) distribution.\n{} scenarios".format(datasize)
    data = [data_ed_nc, data_ed ] #, data_fd_nc, data_fd]
    filename = 'evaluation/results/boxplot_combined.png'
    fig = plt.figure()
    ax = fig.add_subplot(111)
    bp = ax.boxplot(data)
    ax.set_xticklabels(['ED (a)', 'ED (b)']) #,'FD (a)', 'FD (b)' ])
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()
    plt.title(title)
    plt.savefig(filename)
    plt.close(fig)


    #boxplot combined ED
    title = "Euclidean distance (ED) distribution per scenario type"
    filename = 'evaluation/results/boxplot_type_ed.png'
    #plt.cla()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    bp = ax.boxplot(data_ed_type)

    ax.set_xticklabels(['follow (a)', 'follow (b)',
                        'free (a)','free (b)',
                        'free/follow (a)','free/follow (a)',
                        'green light (a)','green light (b)',
                        'red light (a)', 'red light (b)'],rotation = 'vertical')

    #ax.get_xaxis().set_rotation(45)
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()
    fig.subplots_adjust(bottom=0.3)
    plt.title(title)
    plt.savefig(filename)
    plt.close(fig)
    #plt.show()
    """
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
    """

def save_traj_plot(P, Q, es, fd_score, ed_score, config_str):
    for scenario_tag in es.scenario_type:
        filename = 'evaluation/plots/{}/{}/{}/{}/{}_{}'.format(es.map_location, es.scenario_length, scenario_tag, es.video_id, es.scenario_id, config_str)
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
    for scenario_tag in es.scenario_type:
        filename = 'evaluation/plots/{}/{}/{}/{}/{}_{}_speed'.format(es.map_location, es.scenario_length, scenario_tag, es.video_id, es.scenario_id, config_str)
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

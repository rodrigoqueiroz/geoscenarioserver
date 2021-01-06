from argparse import ArgumentParser
import csv
import numpy as np
from math import sqrt, exp
import random
import matplotlib
from matplotlib import pyplot as plt
from frechetdist import frdist



def evaluate(args):
    vid = 10
    vid = 6
    vid = 4
    vid = 26

    if args.eval_vid != "":
        vid = int(args.eval_vid)
    
    trajectories = {}
    #empirical
    trajectories[vid] = load_trajectory_log(vid)
    #synthetic
    trajectories[-vid] = load_trajectory_log(-vid)

    print(len(trajectories))
    calc_frechet(trajectories,vid,-vid, 100, True)
    #plot_vel(trajectories)
    plt.show()

def load_trajectory_log(vid):
    trajectory = []
    with open('traj_log/v_{}.csv'.format(vid), mode='r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if vid == int(row[0]):
                node = {}
                node['x'] = float(row[6])
                node['y'] = float(row[7])
                node['time'] = float(row[4])
                trajectory.append(node)
        return trajectory

def calc_frechet(trajectories, vid_p, vid_q, sample_size = 100, showplot = False):
    traj_p = trajectories[vid_p]
    traj_q = trajectories[vid_q]
    
    #entire list
    P_all= [ [ node['x'],node['y'] ] for node in traj_p] 
    Q_all= [ [ node['x'],node['y'] ] for node in traj_q] 
    max_size = min([len(traj_p),len(traj_q)])
    score_all = frdist(P_all[:max_size],Q_all[:max_size])

    #sample indexes to preserve original order
    #check for maximum sample size
    sample_size = min([len(traj_p),len(traj_q),sample_size])
    #todo: sample time. using indez only works if both are collected with in same simulation with same number of points collected
    #indexes = sorted(random.sample(range(len(traj_p)), sample_size))
    indexes = np.linspace(0, len(traj_p)-1, sample_size,dtype = int, endpoint=False)
    
    
    P = [ [traj_p[i]['x'], traj_p[i]['y']] for i in indexes]
    Q = [ [traj_q[i]['x'], traj_q[i]['y']] for i in indexes]
    score = frdist(P,Q)
    print("Frechet distance between {} and {} is {} with {} samples, and {} with all".format(vid_p,vid_q,score,sample_size,score_all )) 

    #check if implementation makes sense
    #alt_indexes = sorted(random.sample(range(len(traj_p)), sample_size))
    #alt_indexes = np.linspace( int(len(traj_p)/2), len(traj_p)-1, sample_size,dtype = int, endpoint=False)
    #P_alt = [ [traj_p[i]['x'], traj_p[i]['y']] for i in alt_indexes]
    #alt_score = frdist(P,P_alt)
    #print("Frechet distance between full and half is {}".format(alt_score)) 
    
    #plt.plot(   [node[0] for node in P_all],    [node[1] for node in P_all],'b-')
    #plt.plot(   [node[0] for node in Q_all],    [node[1] for node in Q_all],'r-')
    
    plt.plot(   [node[0] for node in P],    [node[1] for node in P], 'b-',label=str(vid_p)+' empirical')
    plt.plot(   [node[0] for node in Q],    [node[1] for node in Q], 'r-',label=str(vid_q)+' synthetic')

    plt.suptitle('Frechet distance: {}'.format(score))
    plt.legend(loc="upper left")
    
    #TP = [ [node['x'] , node['y']] for node in trajectories[vid_p]
    #TQ = [ [ float(step[6]) , float(step[7])] for step in trajectories[vid_q]
    #if (len(traj_p)> sample_size):
        #sample indexes to preserve original order
        

    #for step in trajectories[vid]:
        #TP.append([ float(step[6]) , float(step[7]) ])
    
    #    if (len(TP)> 50):
            #sample indexes to preserve original order
        #    discrete_trajectories[vid] = [TP[i] for i in sorted(random.sample(range(len(TP)), sample_size))]
            #discrete_trajectories[vid] = random.sample(T, 50)
    
   # for step in trajectories[-vid]:
   #     TQ.append([ float(step[6]) , float(step[7]) ])
   #     sample_size = 50
   #     if (len(TQ)> 50):
            #sample indexes to preserve original order
   #         discrete_trajectories[-vid] = [TQ[i] for i in sorted(random.sample(range(len(TQ)), sample_size))]
            #discrete_trajectories[vid] = random.sample(T, 50)
    """   
    P = discrete_trajectories[vid_p]
    Q = discrete_trajectories=[-vid_q]
    score = frdist(P,Q)
    print("Frechet distance between {} and {} is {}".format(vid,-vid,score)) 
    XP = []
    YP = []
    for step in P:
        XP.append( step[0] )
        YP.append( step[1] )
    plt.plot(XP,YP, 'b-')
    XQ = []
    YQ = []
    for step in Q:
        XQ.append( step[0] )
        YQ.append( step[1] )
    plt.plot(XQ,YQ, 'r-')
             """
    
    #P=[[1,1], [2,1], [2,2]]
    #Q=[[2,2], [0,1], [2,4]]
    

    
def plot_pos(trajectory, vid):
    print("Plot Pos")
    #Plot Vel
    fig = plt.figure()
    plt.cla()
    if -vid in trajectories:
        trajectoryP = trajectories[vid]
        trajectoryQ = trajectories[-vid]
        XP = []
        YQ = []
        for step in traj:
            X.append( float(step[6]) )
            Y.append( float(step[7]) )
        #print(data)
        plt.plot(X,Y, '-')

def plot_vel(trajectories):
    print("Plot Vel")
    fig = plt.figure()
    plt.cla()
    for vid,traj in trajectories.items():
        if (-vid in trajectories):
            X = []
            Y = []
            start_time = 0.0
            for step in traj:
                time = float(step[5])
                if start_time ==0.0:
                    start_time = time
                adjusted_time = time - start_time
                xvel = float(step[9])
                yvel = float(step[10])
                vel = sqrt(xvel**2 + yvel**2)
                X.append(adjusted_time)
                Y.append(vel)   
            plt.plot(X,Y, '-')
        
        
    


def plot_acc(trajectories):
    print("Plot Acc")
    #Plot Vel
    fig = plt.figure()
    plt.cla()
    for vid,traj in trajectories.items():
        X = []
        Y = []
        for step in traj:
            time = float(step[5])
            xacc = float(step[11]) 
            yacc = float(step[12])
            acc = sqrt(xacc**2 + yacc**2)
            X.append( time )
            Y.append( acc )
        #print(data)
        plt.plot(X,Y, '-')





if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-e", "--eval", dest="eval_vid", default="", help="Vehicle Id for evaluation")
    args = parser.parse_args()
    evaluate(args)
#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# UTILS
# --------------------------------------------
from math import sqrt, exp, trunc, sin, cos, hypot
import random
from matplotlib import pyplot as plt
import uuid
import numpy as np
from scipy.stats import truncnorm
from itertools import tee
from util.Transformations import normalize


#Iterates an iter by pairs
def pairwise(iterable):
    i, j = tee(iterable, 2)
    next(j, None)
    return zip(i, j)

#Truncates a vector of numbers
def truncate_vector(numbers, digits):
    for i in range(len(numbers)):
        numbers[i] = truncate(numbers[i], digits)
    return numbers

#Truncates a numbers
def truncate(num, digits):
    stp = 10.0 ** digits
    return trunc(stp * num) / stp

#logistic: returns a value between 0 and 1 within the range 0 to infinity
def logistic(x):
    return 2.0 / (1 + exp(-x)) - 1.0

#Returns a function of time f with given coefficients for a polynomial
def to_equation(coefficients):
    def f(t):
        total = 0.0
        for i, coef in enumerate(coefficients):
            total += coef * t ** i
        return total
    return f

#Calculates derivative of a polynomial and returns coefficients.
def differentiate(coefficients):
    der_coef = []
    for deg, prev_coef in enumerate(coefficients[1:]):
        der_coef.append((deg+1) * prev_coef)
    return der_coef


def get_truncated_normal(mean, sd, lo, up):
    return truncnorm.rvs(
        (lo - mean) / sd, (up - mean) / sd, loc=mean, scale=sd)

def kalman(x, z, P, F, H, Q, R):
    F_t = np.transpose(F)
    H_t = np.transpose(H)
    # prediction step
    x_pred = F @ x
    P_pred = F @ (P @ F_t) + Q
    # Kalman gain
    K = P_pred @ (H_t / (R + H @ (P_pred @ H_t))) # R * vel_pred^2? yes
    # update estimation
    x_new = x_pred + K @ (z - H @ x_pred)
    P_new = P_pred - K @ (H @ P_pred)

    return x_new, P_new

def speed_to_vel(speed, angle):
    x_vel = speed*cos(angle)
    y_vel = speed*sin(angle)
    return x_vel, y_vel

def distance_2p(x1,y1,x2,y2):
    dist = hypot(x2 - x1, y2 - y1)
    return dist

#Calculates distance from line (x1,y1,x2,y2) to point (xp, yp)
def distance_point_line(x1,y1,x2,y2, xp, yp):
    p1=np.array([x1,y1])
    p2=np.array([x2,y2])
    p3=np.array([xp,yp])
    d=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
    return abs(d)

def linear_samples(nsamples,lo,up):
    if (lo >= up):
        return tuple([lo])
    return np.linspace(lo,up, nsamples)

def uniform_samples(nsamples,lo,up):
    samples = []
    for x in range(nsamples):
        samples.append(random.uniform(lo, up))
    return samples

def normal_samples(nsamples, mean, sd, lo = None,up = None):
    samples = []
    for x in range(nsamples):
        if lo is None and up is None:
            s = random.gauss(mean,sd)
        else:
            #same as random.gauss(), but with bounds:
            s = get_truncated_normal(mean,sd,lo,up)
        samples.append(s)
    return samples

def distance_point_to_wall(pt, wall):
    p0 = wall[0]
    p1 = wall[1]
    wall_vec = p1-p0
    pt_p0 = pt-p0

    t = np.dot(wall_vec, pt_p0) / np.dot(wall_vec, wall_vec)
    cross = p0 + t*wall_vec

    if t <= 0.0:
        dist = np.linalg.norm(pt_p0)
    elif t >= 1.0:
        dist = np.linalg.norm(pt-p1)
    else:
        dist = np.linalg.norm(cross-pt)
    
    niW = normalize(pt-cross)

    return dist, niW

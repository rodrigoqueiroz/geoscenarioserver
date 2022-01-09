#!/usr/bin/env python3
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


def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm > 0 else v

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

def distance_point_to_border(pt, border):
    ''' get shortest distance from pt to vector (border)
        and unit vector perpendicular to border
    '''
    p0 = border[0]
    p1 = border[1]
    border_vec = p1-p0
    pt_p0 = pt-p0

    t = np.dot(border_vec, pt_p0) / np.dot(border_vec, border_vec)
    cross = p0 + t*border_vec

    if t <= 0.0:
        dist = np.linalg.norm(pt_p0)
    elif t >= 1.0:
        dist = np.linalg.norm(pt-p1)
    else:
        dist = np.linalg.norm(cross-pt)

    norm = normalize(pt-cross)

    return dist, norm

def get_lanelet_entry_exit_points(lanelet):
    ''' given a directed lanelet, return the points
        midway between the endpoints at each end
    '''
    entrance_pt_left = np.array([lanelet.leftBound[0].x, lanelet.leftBound[0].y])
    entrance_pt_right = np.array([lanelet.rightBound[0].x, lanelet.rightBound[0].y])
    exit_pt_left = np.array([lanelet.leftBound[-1].x, lanelet.leftBound[-1].y])
    exit_pt_right = np.array([lanelet.rightBound[-1].x, lanelet.rightBound[-1].y])

    entrance_pt = (entrance_pt_left + entrance_pt_right) / 2
    exit_pt = (exit_pt_left + exit_pt_right) / 2

    return entrance_pt, exit_pt


    ''' return True if line segments L1=(p1,q1) and L2=(p2,q2) intersect
    '''
def line_segments_intersect(L1, L2):
    # find orientations
    o1 = orientation(L1[0], L1[1], L2[0])
    o2 = orientation(L1[0], L1[1], L2[1])
    o3 = orientation(L2[0], L2[1], L1[0])
    o4 = orientation(L2[0], L2[1], L1[1])

    # general case
    if ((o1 != o2) and (o3 != o4)):
        return True

    # special cases

    # p2 colinear to and lies on L1
    if (o1 == 0 and colinear_point_on_line_segment(L2[0], L1)):
        return True

    # q2 colinear to and lies on L1
    if (o2 == 0 and colinear_point_on_line_segment(L2[1], L1)):
        return True

    # p1 colinear to and lies on L2
    if (o3 == 0 and colinear_point_on_line_segment(L1[0], L2)):
        return True

    # q1 colinear to and lies on L2
    if (o4 == 0 and colinear_point_on_line_segment(L1[1], L2)):
        return True

    # line segments to not intersect
    return False


def orientation(p1, p2, p3):
    ''' return the orientation of the ordered triplet of points (p1, p2, p3)
    '''
    val = (float(p2[1] - p1[1]) * (p3[0] - p2[0])) - (float(p2[0] - p1[0]) * (p3[1] - p2[1]))

    if val > 0:
        # clockwise
        return 1
    elif val < 0:
        # counterclockwise
        return 2
    else:
        # colinear
        return 0

def colinear_point_on_line_segment(pt, L):
    ''' given point pt colinear to line L, return True if pt is on L
    '''
    if ((pt[0] <= max(L[0][0], L[1][0])) and (pt[0] >= min(L[0][0], L[1][0])) and
            (pt[1] <= max(L[0][1], L[1][1])) and (pt[1] >= min(L[0][1], L[1][1]))):
        return True

    return False

def angle_btwn_vectors(a, b):
    ''' return the angle in radians between vectors a and b
        (0 <= angle <= pi)
    '''
    return np.arccos(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))

def point_in_rectangle(P, A, B, C, D):
    ''' Checks if the point pt is in the rectangle defined by
        points A, B, C, D in a counterclockwise orientation
    '''
    left_of_AB = (B[0]-A[0])*(P[1]-A[1]) - (B[1]-A[1])*(P[0]-A[0]) > 0
    left_of_BC = (C[0]-B[0])*(P[1]-B[1]) - (C[1]-B[1])*(P[0]-B[0]) > 0
    left_of_CD = (D[0]-C[0])*(P[1]-C[1]) - (D[1]-C[1])*(P[0]-C[0]) > 0
    left_of_DA = (A[0]-D[0])*(P[1]-D[1]) - (A[1]-D[1])*(P[0]-D[0]) > 0

    '''
    Could not directly return (left_of_AB and left_of_BC and left_of_CD and left_of_DA)
    for some reason so I had to split into two different bools (first_two and last_two).

    (left_of_AB and left_of_BC and left_of_CD and left_of_DA) apparently evaluates
    to an empty list instead of True or False
    '''
    first_two = left_of_AB and left_of_BC
    last_two = left_of_CD and left_of_DA
    inside_rect = first_two and last_two

    return inside_rect

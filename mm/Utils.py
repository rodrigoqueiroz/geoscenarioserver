#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# UTILS
# --------------------------------------------
#TODO: refactor

from math import sqrt, exp
from matplotlib import pyplot as plt
import uuid
import numpy as np

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

#Calculates the closest distance to any vehicle during a trajectory.
def nearest_approach_to_any_vehicle(traj, vehicles):
    closest = 999999
    for v in vehicles.values():
        d = nearest_approach(traj,v)
        if d < closest:
            closest = d
    return closest

def nearest_approach(traj, vehicle):
    closest = 999999
    s_,d_,T = traj
    s = to_equation(s_)
    d = to_equation(d_)
    for i in range(100):
        t = float(i) / 100 * T
        cur_s = s(t)
        cur_d = d(t)
        targ_s, _, _, targ_d, _, _ = vehicle.state_in(t)
        dist = sqrt((cur_s-targ_s)**2 + (cur_d-targ_d)**2)
        if dist < closest:
            closest = dist
    return closest



def get_f_and_N_derivatives(coeffs, N=3):
    functions = [to_equation(coeffs)]
    for i in range(N):
        coeffs = differentiate(coeffs)
        functions.append(to_equation(coeffs))
    return functions


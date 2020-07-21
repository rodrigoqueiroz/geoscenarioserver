#!/usr/bin/env python
#d43sharm@uwaterloo.ca
#rqueiroz@uwaterloo.ca
# ---------------------------------------------
# Transformations between Cartesian and Frenet frames.
# --------------------------------------------

import numpy as np
from math import copysign
from lanelet2.core import BasicPoint2d, ConstLineString2d, ConstLineString3d
from lanelet2.geometry import toArcCoordinates


class OutsideRefPathException(Exception):
    pass


def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm > 0 else v


def vector_interp(a, b, t):
    return a + t * (b - a)


def sim_to_frenet_position(ref_path:ConstLineString3d, x, y):
    path_ls = ConstLineString2d(ref_path)
    # toArcCoordinates does not interpolate beyond or before ref_path
    # arc_coords.distance used as d may be discontinuous moving across points
    arc_coords = toArcCoordinates(path_ls, BasicPoint2d(x, y))
    return arc_coords.length, arc_coords.distance


def tangent_of_path_at(path, i):
    """ Returns the average of the vectors path[i] - path[i-1] and path[i+1] - path[i].
        If i points to the first or last point in path, the vector to the next or
        previous point respectively is returned.
    """
    p = path[i]
    
    op = np.array([0,0]) if i == 0 else np.array([p.x - path[i-1].x, p.y - path[i-1].y])
    pq = np.array([0,0]) if i == len(path) - 1 else np.array([path[i+1].x - p.x, path[i+1].y - p.y])
    
    return normalize(op + pq)


def sim_to_frenet_frame(ref_path:ConstLineString3d, x_vector, y_vector):
    """ Transforms position and speed from cartesian to frenet frame based of ref_path
        
        @param ref_path:    ConstLineString3d. Change to something general? Enforce types in python?
    """
    x, x_vel, x_acc = x_vector
    y, y_vel, y_acc = y_vector
    velocity = np.array([x_vel, y_vel])
    acc = np.array([x_acc, y_acc])

    path_ls = ConstLineString2d(ref_path)
    # toArcCoordinates does not interpolate beyond or before ref_path
    # arc_coords.distance used as d may be discontinuous moving across points
    arc_coords = toArcCoordinates(path_ls, BasicPoint2d(x, y))
    s = arc_coords.length
    d = arc_coords.distance

    # velocity
    unit_tangent = None
    arclen = 0
    for i in range(len(ref_path)-1):
        p = ref_path[i]
        q = ref_path[i+1]
        
        pq = np.array([q.x - p.x, q.y - p.y])
        dist = np.linalg.norm(pq)

        if arclen <= s <= arclen + dist: # if s lies between p and q
            # interpolate tangent between p and q
            percent_delta_s = (s - arclen) / dist
            tangent_p = tangent_of_path_at(ref_path, i)
            tangent_q = tangent_of_path_at(ref_path, i+1)
            unit_tangent = normalize(vector_interp(tangent_p, tangent_q, percent_delta_s)) # tangent_p + percent_delta_s * (tangent_q - tangent_p) )
            unit_normal = np.array([-1 * unit_tangent[1], unit_tangent[0]])
            # should be in direction of unit normal, pointing inwards
            # curvature is negative if pointing away from unit normal
            kappa_vector = (tangent_q - tangent_p) / dist
            kappa = copysign(np.linalg.norm(kappa_vector), np.dot(kappa_vector, unit_normal))
            break

        arclen += dist

    if unit_tangent is None:
        print("point is outside the reference path.")
        raise OutsideRefPathException()
    
    s_vel = np.dot(velocity, unit_tangent)
    d_vel = np.dot(velocity, unit_normal)
    # using d/dt(tangent(s(t))) = kappa*normal*s_vel and ddt(normal(s(t))) = -kappa*tangent*s_vel
    s_acc = np.dot(acc, unit_tangent) + np.dot(velocity, kappa*s_vel*unit_normal)
    d_acc = np.dot(acc, unit_normal) - np.dot(velocity, kappa*s_vel*unit_tangent)
    
    return [s, s_vel, s_acc], [d, d_vel, d_acc]


def frenet_to_sim_frame(ref_path:ConstLineString3d, s_vector, d_vector):
    """ Transforms position and speed from frenet frame based on ref_path to cartesian.
        @param ref_path:    iterable of lanelet2.core.Point3d. Change to something general?
    """
    s, s_vel, s_acc = s_vector
    d, d_vel, d_acc = d_vector

    point_on_ls = None
    unit_tangent = None
    arclen = 0
    for i in range(len(ref_path)-1):
        p = ref_path[i]
        q = ref_path[i+1]

        pq = np.array([q.x - p.x, q.y - p.y])
        dist = np.linalg.norm(pq)

        if s < 0 or arclen <= s <= arclen + dist: # if s lies between p and q
            # r(s)
            delta_s = s - arclen
            unit_pq = normalize(pq)
            # we are still travelling along pq, but facing in the direction of an interpolated tangent
            point_on_ls = np.array([p.x, p.y]) + unit_pq * delta_s
            
            # interpolate tangent between p and q
            tangent_p = tangent_of_path_at(ref_path, i)
            tangent_q = tangent_of_path_at(ref_path, i+1)
            percent_delta_s = delta_s / dist
            unit_tangent = normalize(vector_interp(tangent_p, tangent_q, percent_delta_s))
            # unit normal turns tangent counter-clockwise according to frenet conventions
            unit_normal = np.array([-1 * unit_tangent[1], unit_tangent[0]])
            
            # should be in direction of unit normal, pointing inwards
            # curvature is negative if pointing away from unit normal
            kappa_vector = (tangent_q - tangent_p) / dist
            kappa = copysign(np.linalg.norm(kappa_vector), np.dot(kappa_vector, unit_normal))
            break
        
        arclen += dist
    
    if point_on_ls is None:
        print("s {} d {} is outside the reference path.".format(s, d))
        raise OutsideRefPathException()
    
    position = point_on_ls + unit_normal*d
    vel = s_vel * unit_tangent + d_vel * unit_normal
    # using d/dt(tangent(s(t))) = kappa*normal*s_vel and ddt(normal(s(t))) = -kappa*tangent*s_vel
    acc = s_acc*unit_tangent + s_vel*s_vel*kappa*unit_normal + d_acc*unit_normal - d_vel*s_vel*kappa*unit_tangent
    return [position[0], vel[0], acc[0]], [position[1], vel[1], acc[1]]

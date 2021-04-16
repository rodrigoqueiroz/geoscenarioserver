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
from typing import List
from scipy import interpolate

from util.Utils import distance_2p

class OutsideRefPathException(Exception):
    def __init__(self, message):
        Exception(message)


def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm > 0 else v


def vector_interp(a, b, t):
    return a + t * (b - a)


def frenet_to_sim_position(ref_path:ConstLineString3d, s:float, d:float, s_start:float):
    sim_position = None
    arclen = s_start

    if s >= s_start:
        for i in range(len(ref_path) - 1):
            p = ref_path[i]
            q = ref_path[i + 1]

            pq = np.array([q.x - p.x, q.y - p.y])
            dist = np.linalg.norm(pq)

            if arclen <= s <= arclen + dist: # if s lies between p and q
                unit_tangent = normalize(pq)
                # rotate tangent counter-clockwise for unit normal
                unit_normal = np.array([-1 * unit_tangent[1], unit_tangent[0]])

                # we are still travelling along pq, but facing in the direction of an interpolated tangent
                point_on_ref_path = np.array([p.x, p.y]) + unit_tangent * (s - arclen)
                sim_position = point_on_ref_path + unit_normal * d
                break

            arclen += dist

    if sim_position is None:
        raise OutsideRefPathException("s {} d {} is outside the reference path.".format(s, d))
    return sim_position


def sim_to_frenet_position(ref_path:ConstLineString3d, x:float, y:float, s_start:float, max_dist_from_path:float=5.0):
    # TODO: find a better way of checking if the point is on ref_path
    point_is_on_path = False
    for point in ref_path:
        if distance_2p(point.x, point.y, x, y) <= max_dist_from_path:
            point_is_on_path = True
            break

    if not point_is_on_path:
        raise OutsideRefPathException("x {} y {} is outside the reference path.".format(x, y))

    path_ls = ConstLineString2d(ref_path)
    # toArcCoordinates does not interpolate beyond or before ref_path
    # arc_coords.distance used as d may be discontinuous moving across points
    arc_coords = toArcCoordinates(path_ls, BasicPoint2d(x, y))

    return (arc_coords.length + s_start, arc_coords.distance)


def path_frame_at_arclength(ref_path:ConstLineString3d, s:float, s_start:float):
    """ Returns the unit tangent, unit normal, and kappa (curvature) at arclength s along
        the reference path.
    """
    ret = None
    arclen = s_start

    if s >= s_start:
        for i in range(len(ref_path) - 1):
            p = ref_path[i]
            q = ref_path[i + 1]

            pq = np.array([q.x - p.x, q.y - p.y])
            dist = np.linalg.norm(pq)

            if arclen <= s <= arclen + dist: # if s lies between p and q
                # interpolate tangent between p and q
                tangent_p = tangent_of_path_at(ref_path, i)
                tangent_q = tangent_of_path_at(ref_path, i+1)
                percent_delta_s = (s - arclen) / dist
                unit_tangent = normalize(vector_interp(tangent_p, tangent_q, percent_delta_s))
                # rotate tangent counter-clockwise for unit normal
                unit_normal = np.array([-1 * unit_tangent[1], unit_tangent[0]])
                # should be parallel to unit normal, pointing inwards
                # curvature is negative if pointing away from unit normal
                kappa_vector = (tangent_q - tangent_p) / dist
                kappa = copysign(np.linalg.norm(kappa_vector), np.dot(kappa_vector, unit_normal))

                ret = (unit_tangent, unit_normal, kappa)
                break

            arclen += dist

    if ret is None:
        raise OutsideRefPathException("s {} is outside the reference path.".format(s))
    return ret


def tangent_of_path_at(path:ConstLineString3d, i:int):
    """ Returns the average of the vectors path[i] - path[i-1] and path[i+1] - path[i].
        If i points to the first or last point in path, the vector to the next or
        previous point respectively is returned.
    """
    p = path[i]

    op = np.array([0,0]) if i == 0 else np.array([p.x - path[i-1].x, p.y - path[i-1].y])
    pq = np.array([0,0]) if i == len(path) - 1 else np.array([path[i+1].x - p.x, path[i+1].y - p.y])

    return normalize(op + pq)


def sim_to_frenet_frame(ref_path:ConstLineString3d, x_vector:List[float], y_vector:List[float], s_start:float):
    """ Transforms position and speed from cartesian to frenet frame based of ref_path

        @param ref_path:    ConstLineString3d. Change to something general? Enforce types in python?
    """
    x, x_vel, x_acc = x_vector
    y, y_vel, y_acc = y_vector
    velocity = np.array([x_vel, y_vel])
    acc = np.array([x_acc, y_acc])

    # position
    s, d = sim_to_frenet_position(ref_path, x, y, s_start)

    unit_tangent, unit_normal, kappa = path_frame_at_arclength(ref_path, s, s_start)

    s_vel = np.dot(velocity, unit_tangent)
    d_vel = np.dot(velocity, unit_normal)
    # using ddt(tangent(s(t))) = kappa*normal*s_vel and ddt(normal(s(t))) = -kappa*tangent*s_vel
    s_acc = np.dot(acc, unit_tangent) + np.dot(velocity, kappa*s_vel*unit_normal)
    d_acc = np.dot(acc, unit_normal) - np.dot(velocity, kappa*s_vel*unit_tangent)

    return [s, s_vel, s_acc], [d, d_vel, d_acc]


def frenet_to_sim_frame(ref_path:ConstLineString3d, s_vector:List[float], d_vector:List[float], s_start:float):
    """ Transforms position and speed from frenet frame based on ref_path to cartesian.
        @param ref_path:    iterable of lanelet2.core.Point3d. Change to something general?
    """
    s, s_vel, s_acc = s_vector
    d, d_vel, d_acc = d_vector

    position = frenet_to_sim_position(ref_path, s, d, s_start)
    unit_tangent, unit_normal, kappa = path_frame_at_arclength(ref_path, s, s_start)

    # position = point_on_ref_path + unit_normal*d
    vel = s_vel * unit_tangent + d_vel * unit_normal
    # using d/dt(tangent(s(t))) = kappa*normal*s_vel and ddt(normal(s(t))) = -kappa*tangent*s_vel
    acc = s_acc*unit_tangent + s_vel*s_vel*kappa*unit_normal + d_acc*unit_normal - d_vel*s_vel*kappa*unit_tangent

    return [position[0], vel[0], acc[0]], [position[1], vel[1], acc[1]]

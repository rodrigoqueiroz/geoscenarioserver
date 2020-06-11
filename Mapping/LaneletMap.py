#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# LaneletMap
# Class to parse a Lanelet 2 map and interface with lanelet2 library
# --------------------------------------------

import lanelet2
from lanelet2.core import getId, BasicPoint2d, Point3d, Point2d, BoundingBox2d, LineString3d, LineString2d, ConstLineString2d, ConstLineString3d, Lanelet
from lanelet2.geometry import distance, boundingBox2d, inside, toArcCoordinates, project, length2d
from lanelet2.projection import UtmProjector

from matplotlib import pyplot as plt
from itertools import tee
import numpy as np


def get_test_ll():
    left = LineString3d(getId(), [Point3d(getId(), 1, y, 0) for y in range(0, 5)])
    right = LineString3d(getId(), [Point3d(getId(), 2, y, 0) for y in range(0, 5)])
    return Lanelet(getId(), left, right)

def pairwise(iterable):
    i, j = tee(iterable, 2)
    next(j, None)
    return zip(i, j)

def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm > 0 else v


class OutsideRefPathException(Exception):
    pass


class LaneletMap(object):
    def __init__(self):
        self.example_map = "/home/divit/lanelet2_standalone/lanelet2_maps/res/mapping_example.osm"
        projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
        # map is collection of primitive layers
        self.lanelet_map, errors = lanelet2.io.loadRobust(self.example_map, projector)
        assert not errors


    def get_occupying_lanelet_in_route(self, s, lanelet_route):
        running_length = 0
        for ll_id in lanelet_route:
            ll = self.lanelet_map.laneletLayer[ll_id]
            ll_length = length2d(ll)
            if running_length <= s <= running_length + ll_length:
                return ll
            
            running_length += ll_length
        
        # s is outside the route
        return self.lanelet_map.laneletLayer[lanelet_route[-1]]

    # TODO: change this to get_occupying_lanelet_in_route(s, lanelet_route)
    # this function conceptually doesn't work - its all just heuristics
    def get_occupying_lanelet(self, x, y, lanelet_route=None):
        point = BasicPoint2d(x, y)
        
        # get all intersecting lanelets using a trivial bounding box
        searchbox = BoundingBox2d(point, point)
        intersecting_lls = self.lanelet_map.laneletLayer.search(searchbox)

        if len(intersecting_lls) == 0:
            print("Lanelet Error: vehicle not part of any lanelet.")
            return None
        elif len(intersecting_lls) > 1:
            # filter results for lanelets containing the point
            intersecting_lls = list(filter(lambda ll: inside(ll, point), intersecting_lls))
            print([ll.id for ll in intersecting_lls])

            if len(intersecting_lls) > 1:
                # use routing or some other information to determine which lanelet we are in
                if lanelet_route is not None:
                    intersecting_lls = list(filter(lambda ll: ll.id in lanelet_route, intersecting_lls))
                    assert len(intersecting_lls) == 1

        return intersecting_lls[0]

    def get_global_path_for_route(self, lanelet_route, x = None, y = None, meters_ahead=float("inf")):
        """ This looks 100m ahead of the beginning of the current lanelet. Change?
            x, y only used to determine the starting lanelet, allowed to be a little outdated.
            NOTE: lookahead isn't implemented yet, since change in path results in change in frenet coords
            IDEALLY we don't want to request for this very often - only when we generate a new trajectory
            @param lanelet_route:   list of consecutive lanelets to grab path from
            @return:    list of lanelet2.core.Point3d
        """
        cur_ll = self.lanelet_map.laneletLayer[lanelet_route[0]]
        path = [ Point3d(0, cur_ll.centerline[0].x, cur_ll.centerline[0].y, 0.0) ]
        path_length = 0
        for ll_id in lanelet_route:
            for p, q in pairwise(self.lanelet_map.laneletLayer[ll_id].centerline):
                dist = distance(p, q)
                if path_length + dist <= meters_ahead:
                    path.append(Point3d(0, q.x, q.y, 0.0))
                    path_length += dist
                else:
                    return ConstLineString3d(0, path)
        
        path_ls = ConstLineString3d(0, path)
        return path_ls

    def plot_lanelets(self, lanelet_ids):
        for ll_id in lanelet_ids:
            LaneletMap.plot_ll(self.lanelet_map.laneletLayer[ll_id])

    @staticmethod
    def plot_ll(lanelet):
        """ Plots the bounds of the lanelet on the current pyplot
        """
        xs = [pt.x for pt in lanelet.rightBound] + [pt.x for pt in lanelet.leftBound]
        ys = [pt.y for pt in lanelet.rightBound] + [pt.y for pt in lanelet.leftBound]
        
        plt.plot(xs, ys, 'ro')
        plt.plot(
            [pt.x for pt in lanelet.centerline],
            [pt.y for pt in lanelet.centerline],
            'go')
    

    @staticmethod
    def get_lane_width(lanelet, x, y):
        """ Two ways to do this: project the point onto leftbound and rightbound and add their distances
            OR intersect the centerline normal at s with leftbound and rightbound.
            Either way the width would be discontinuous as you move along s.
            TODO: do we actually need average/min lane width over some lookahead time?
            @param s:   length along the lanelet centerline
        """
        # x, y = LaneletMap.frenet_to_sim_frame(lanelet.centerline, s, 0)
        point_on_centerline = BasicPoint2d(x, y)
        # project on left and right bounds
        point_on_leftbound = project(ConstLineString2d(lanelet.leftBound), point_on_centerline)
        point_on_rightbound = project(ConstLineString2d(lanelet.rightBound), point_on_centerline)
        return distance(point_on_leftbound, point_on_rightbound)
    
    
    @staticmethod
    def sim_to_frenet_frame(ref_path:ConstLineString3d, x_vector, y_vector):
        """
            @param ref_path:    ConstLineString3d. Change to something general? Enforce types in python?
        """
        x, x_vel, x_acc = x_vector
        y, y_vel, y_acc = y_vector
        velocity = np.array([x_vel, y_vel])

        path_ls = ConstLineString2d(ref_path)
        # toArcCoordinates does not interpolate beyond or before ref_path
        arc_coords = toArcCoordinates(path_ls, BasicPoint2d(x, y))

        # velocity
        unit_tangent = None
        arclen = 0
        for p, q in pairwise(ref_path):
            pq = np.array([q.x - p.x, q.y - p.y])
            dist = np.linalg.norm(pq)

            if arclen <= arc_coords.length <= arclen + dist: # if s lies between p and q
                unit_tangent = normalize(pq)
                break
            arclen += dist

        if unit_tangent is None:
            print("point is outside the reference path.")
            raise OutsideRefPathException()
        
        unit_normal = np.array([-1 * unit_tangent[1], unit_tangent[0]])
        vel_frenet = np.array([
            np.dot(velocity, unit_tangent),
            np.dot(velocity, unit_normal)])
         # ensure speed is right
        return [arc_coords.length, vel_frenet[0], x_acc], [arc_coords.distance, vel_frenet[1], y_acc]


    @staticmethod
    def frenet_to_sim_frame(ref_path, s_vector, d_vector):
        """
            @param ref_path:    iterable of lanelet2.core.Point3d. Change to something general?
        """
        s, s_vel, s_acc = s_vector
        d, d_vel, d_acc = d_vector

        point_on_ls = None
        # kappa = None
        unit_tangent = None
        arclen = 0
        for p, q in pairwise(ref_path):
            pq = np.array([q.x - p.x, q.y - p.y])
            dist = np.linalg.norm(pq)

            if s < 0 or arclen <= s <= arclen + dist: # if s lies between p and q
                # r(s)
                delta_s = s - arclen
                print(delta_s)
                unit_tangent = normalize(pq)
                point_on_ls = np.array([p.x, p.y]) + unit_tangent * delta_s

                # kappa
                # if i+2 < len(ref_path):
                #     # take next unit_tangent for kappa calc
                #     r = ref_path[i+2]
                #     tangent_qr = normalize(np.array([r.x - q.x, r.y - q.y]))
                #     dT = tangent_qr - unit_tangent
                #     kappa = np.linalg.norm(dT / dist)
                # elif i-1 >= 0:
                #     # take previous tangent for calculation
                #     op = np.array([p.x - o.x, p.y - o.y])
                #     o = ref_path[i-1]
                #     tangent_op = normalize(op)
                #     dT = unit_tangent - tangent_op
                #     kappa = np.linalg.norm(dT / np.linalg.norm(op))
                break
            
            arclen += dist
            # print((p.x, p.y, q.x, q.y))
        
        if point_on_ls is None:
            print("s {} d {} is outside the reference path.".format(s, d))
            raise OutsideRefPathException()
        
        unit_normal = np.array([-1 * unit_tangent[1], unit_tangent[0]])

        point_in_cart = point_on_ls + unit_normal*d
        # one_minus_kappa_d_2 = (1 - kappa * d) ** 2
        # v = np.sqrt( one_minus_kappa_d_2 * s_vel**2 + d_vel**2 )
        # theta_r = np.arctan2(unit_tangent[1], unit_tangent[0])
        # theta_x = np.arcsin(d_vel/v) + theta_r
        # vel = v * np.array([np.cos(theta_x), np.sin(theta_x)])
        vel = s_vel * unit_tangent + d_vel * unit_normal
        return [point_in_cart[0], vel[0], s_acc], [point_in_cart[1], vel[1], d_acc]

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

# this might not be worth using over indexes
def tripletwise(iterable):
    i, j, k = tee(iterable, 3)
    next(j, None)
    next(k, None)
    next(k, None)
    return zip(i, j, k)

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
        # update cache for every new ref path
        self.tangents_cache = []


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

        if len(self.tangents_cache) == 0:
            i = 0
            for p, q, r in tripletwise(path_ls):
                pq = np.array([q.x - p.x, q.y - p.y])
                qr = np.array([r.x - q.x, r.y - q.y])
                if i == 0:  # first tangent
                    self.tangents_cache.append(normalize(pq))
                    i += 1
                # tangent of q
                self.tangents_cache.append(normalize(pq + qr))
                i += 1
                if i == len(path_ls) - 1:   # last tangent
                    self.tangents_cache.append(normalize(qr))
                    i += 1
            print(self.tangents_cache)
            
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
    
    
    # @staticmethod
    def sim_to_frenet_frame(self, ref_path:ConstLineString3d, x_vector, y_vector):
        """
            @param ref_path:    ConstLineString3d. Change to something general? Enforce types in python?
        """
        x, x_vel, x_acc = x_vector
        y, y_vel, y_acc = y_vector
        velocity = np.array([x_vel, y_vel])

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
                percent_delta_s = (s - arclen) / dist
                unit_tangent = normalize( self.tangents_cache[i] + percent_delta_s * (self.tangents_cache[i+1] - self.tangents_cache[i]) )
                break
            arclen += dist

        if unit_tangent is None:
            print("point is outside the reference path.")
            raise OutsideRefPathException()
        
        unit_normal = np.array([-1 * unit_tangent[1], unit_tangent[0]])
        vel_frenet = np.array([
            np.dot(velocity, unit_tangent),
            np.dot(velocity, unit_normal)])
        return [s, vel_frenet[0], x_acc], [d, vel_frenet[1], y_acc]


    # @staticmethod
    def frenet_to_sim_frame(self, ref_path, s_vector, d_vector):
        """
            @param ref_path:    iterable of lanelet2.core.Point3d. Change to something general?
        """
        s, s_vel, s_acc = s_vector
        d, d_vel, d_acc = d_vector

        point_on_ls = None
        unit_tangent = None
        arclen = 0
        for i in range(len(ref_path)):
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
                
                percent_delta_s = delta_s / dist
                unit_tangent = normalize(self.tangents_cache[i] + percent_delta_s * (self.tangents_cache[i+1] - self.tangents_cache[i]))
                unit_normal = np.array([-1 * unit_pq[1], unit_pq[0]])
                break
            
            arclen += dist
            # print((p.x, p.y, q.x, q.y))
        
        if point_on_ls is None:
            print("s {} d {} is outside the reference path.".format(s, d))
            raise OutsideRefPathException()
        

        point_in_cart = point_on_ls + unit_normal*d
        vel = s_vel * unit_tangent + d_vel * unit_normal
        return [point_in_cart[0], vel[0], s_acc], [point_in_cart[1], vel[1], d_acc]

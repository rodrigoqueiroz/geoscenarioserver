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
from lanelet2.traffic_rules import Locations, Participants
from lanelet2.projection import UtmProjector
from lanelet2.routing import RelationType

from matplotlib import pyplot as plt
from itertools import tee
import numpy as np

from typing import List


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
    def __init__(self, projector):
        self.example_map = "/home/divit/lanelet2_standalone/lanelet2_maps/res/mapping_example.osm"
        # map is collection of primitive layers
        self.lanelet_map, errors = lanelet2.io.loadRobust(self.example_map, projector)
        assert not errors
        # update cache for every new ref path
        self.tangents_cache = []

        # generate routing table
        traffic_rules = lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle)
        self.routing_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, traffic_rules)
        
        # Route has additional semantic information for the shortest path
        # shortest_path = self.get_shortest_path(4984315, 2925017)
        # for ll in shortest_path:
        #     print(ll)
        # self.plot_lanelets(shortest_path)
        # plt.show()
        # exit()

    def get_right(self, lanelet):
        return self.routing_graph.right(lanelet)
    
    def get_left(self, lanelet):
        return self.routing_graph.left(lanelet)
    
    @staticmethod
    def get_left_in_route(route, lanelet):
        leftRelation = route.leftRelation(lanelet)
        if leftRelation.lanelet and leftRelation.relationType == RelationType.Left \
            and route.contains(leftRelation.lanelet):
            return leftRelation.lanelet
        return None

    @staticmethod
    def get_right_in_route(route, lanelet):
        rightRelation = route.rightRelation(lanelet)
        if rightRelation and rightRelation.lanelet and rightRelation.relationType == RelationType.Right \
            and route.contains(rightRelation.lanelet):
            return rightRelation.lanelet
        return None

    def get_route(self, from_lanelet_id:int, to_lanelet_id:int):
        from_ll = self.lanelet_map.laneletLayer[from_lanelet_id]
        to_ll = self.lanelet_map.laneletLayer[to_lanelet_id]
        # Route object automatically constructs a sub-laneletmap
        route = self.routing_graph.getRoute(from_ll, to_ll)
        assert route
        return route
    
    def get_route_via(self, lanelets:List[Lanelet]):
        assert len(lanelets) > 2
        route = self.routing_graph.getRouteVia(lanelets[0], lanelets[1:-1], lanelets[-1])
        assert route
        return route

    def get_shortest_path(self, from_lanelet_id:int, to_lanelet_id:int):
        route = self.get_route(from_lanelet_id, to_lanelet_id)
        shortest_path = route.shortestPath()
        assert(shortest_path)
        return shortest_path

    def get_occupying_lanelet(self, x, y):
        point = BasicPoint2d(x, y)
        
        # get all intersecting lanelets using a trivial bounding box
        searchbox = BoundingBox2d(point, point)
        intersecting_lls = self.lanelet_map.laneletLayer.search(searchbox)

        if len(intersecting_lls) == 0:
            raise Exception("Lanelet Error: vehicle not part of any lanelet.")
        elif len(intersecting_lls) > 1:
            # filter results for lanelets containing the point
            intersecting_lls = list(filter(lambda ll: inside(ll, point), intersecting_lls))
            if len(intersecting_lls) > 1:
                raise Exception("Point {} part of more than one lanelet ({}), cannot automatically resolve.".format(
                    (x,y), [ll.id for ll in intersecting_lls]))

        return intersecting_lls[0]

    def get_occupying_lanelet_in_route(self, s, lanelet_route):
        # NOTE: doesn't account for lane changes in route
        running_length = 0
        index = 0
        llmap = lanelet_route.laneletMap()
        for ll in llmap.laneletLayer:
            # ll = self.lanelet_map.laneletLayer[ll_id]
            ll_length = length2d(ll)
            if running_length <= s <= running_length + ll_length:
                return ll, index
            
            index += 1
            running_length += ll_length
        
        # s is outside the route
        return self.lanelet_map.laneletLayer[lanelet_route[-1]], -1

    def get_occupying_lanelet_by_position(self, lanelet_route, x, y):
        index = 0 # NOTE index means nothing if not search in an order
        ret = None
        # NOTE use laneletMap() requires a fix to python bindings
        for ll in lanelet_route.laneletMap().laneletLayer:
            if inside(ll, BasicPoint2d(x, y)):
                ret = ll
                break
            index += 1

        return ret, index

    def get_global_path_for_route(self, lanelet_route, x = None, y = None, meters_ahead=float("inf")):
        """ This looks 100m ahead of the beginning of the current lanelet. Change?
            x, y only used to determine the starting lanelet, allowed to be a little outdated.
            NOTE: lookahead isn't implemented yet, since change in path results in change in frenet coords
            IDEALLY we don't want to request for this very often - only when we generate a new trajectory
            TODO: factor in lane changes in route - maybe return path for current lane?
            @param lanelet_route:   Route object from lanelet2
            @return:    list of lanelet2.core.Point3d
        """
        # if a position is not given, take the first lanelet in the shortest path
        # otherwise find the lanelet we are in in the shortest path
        cur_ll, _ = (lanelet_route.shortestPath()[0], 0) if x is None or y is None \
            else self.get_occupying_lanelet_by_position(lanelet_route, x, y)
        assert cur_ll
        
        cur_lane = lanelet_route.fullLane(cur_ll)
        assert cur_lane
        
        # cur_ll = lanelet_route[0]
        path = []
        # path = [ Point3d(0, cur_ll.centerline[0].x, cur_ll.centerline[0].y, 0.0) ]
        path_length = 0
        for ll in cur_lane:
            for p, q in pairwise(ll.centerline):
                # for first iteration, also append p
                if path_length == 0:
                    path.append(Point3d(0, p.x, p.y, 0.0))

                dist = distance(p, q)
                path.append(Point3d(0, q.x, q.y, 0.0))
                path_length += dist

        path_ls = ConstLineString3d(0, path)

        # rebuild tangent cache
        self.tangents_cache = []
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
        # print(self.tangents_cache)
            
        return path_ls

    def plot_lanelets(self, lanelets:iter):
        for ll in lanelets:
            # LaneletMap.plot_ll(self.lanelet_map.laneletLayer[ll_id])
            LaneletMap.plot_ll(ll)

    @staticmethod
    def plot_ll(lanelet):
        """ Plots the bounds of the lanelet on the current pyplot
        """
        for bound in (lanelet.leftBound, lanelet.rightBound):
            xs = [pt.x for pt in bound]
            ys = [pt.y for pt in bound]
            plt.plot(xs, ys, 'r')
        
        plt.plot(
            [pt.x for pt in lanelet.centerline],
            [pt.y for pt in lanelet.centerline],
            'go')
    

    @staticmethod
    def get_lane_width(lanelet, x, y):
        """ Two ways to do this: project the point onto leftbound and rightbound and add their distances
            OR intersect the centerline normal at s with leftbound and rightbound.
            Either way the width would be discontinuous as you move along s.
            @param s:   length along the lanelet centerline
        """
        # x, y = LaneletMap.frenet_to_sim_frame(lanelet.centerline, s, 0)
        point_on_centerline = project(ConstLineString2d(lanelet.centerline), BasicPoint2d(x, y))
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

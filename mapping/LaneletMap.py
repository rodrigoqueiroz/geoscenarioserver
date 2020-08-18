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
import glog as log

from typing import List


def pairwise(iterable):
    i, j = tee(iterable, 2)
    next(j, None)
    return zip(i, j)


class LaneletMap(object):
    def __init__(self):
        self.lanelet_map = None

    def load_lanelet_map(self, filename, projector):
        self.lanelet_map, errors = lanelet2.io.loadRobust(filename, projector)
        assert not errors, log.error(errors)

        # generate routing table
        traffic_rules = lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle)
        self.routing_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, traffic_rules)

    def get_right(self, lanelet):
        return self.routing_graph.right(lanelet)

    def get_left(self, lanelet):
        return self.routing_graph.left(lanelet)

    def get_next(self, lanelet):
        # returns first following lanelet
        following = self.routing_graph.following(lanelet)
        return following[0] if following else None

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
        assert len(lanelets) >= 2

        if len(lanelets) > 2:
            route = self.routing_graph.getRouteVia(lanelets[0], lanelets[1:-1], lanelets[-1])
        else:
            route = self.routing_graph.getRoute(lanelets[0], lanelets[1])
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

    def get_occupying_lanelet_in_reference_path(self, ref_path, lanelet_route, x, y):
        """ Returns the lanelet closest to (x,y) that lies along ref_path.
            Does not check if (x,y) lies inside the lanelet.

            @param lanelet_route:   the lanelet2 Route object that ref_path goes through.
        """
        point_on_path = project(ConstLineString2d(ref_path), BasicPoint2d(x, y))
        return self.get_occupying_lanelet_by_route(lanelet_route, point_on_path.x, point_on_path.y)

    def get_occupying_lanelet_by_route(self, lanelet_route, x, y):
        """ Returns the lanelet in lanelet_route that completely encloses the point defined by x,y.
            If multiple lanelets enclose the point, the first is returned.
        """
        ret = None
        # NOTE use laneletMap() requires a fix to python bindings
        for ll in lanelet_route.laneletMap().laneletLayer:
            if inside(ll, BasicPoint2d(x, y)):
                ret = ll
                break

        return ret

    def get_global_path_for_route(self, lanelet_route, x=None, y=None, meters_after_end=50):
        """ This looks 100m ahead of the beginning of the current lanelet. Change?
            x, y only used to determine the starting lanelet, allowed to be a little outdated.
            Ideally we don't want to request for this very often - only when we generate a new trajectory.

            @param lanelet_route:       Route object from lanelet2
            @param meters_after_end:    distance beyond the end of lanelet_route to extend the global path by

            @return:    list of lanelet2.core.Point3d
        """
        # if a position is not given, take the first lanelet in the shortest path
        # otherwise find the lanelet we are in in the shortest path
        cur_ll = lanelet_route.shortestPath()[0] if x is None or y is None \
            else self.get_occupying_lanelet_by_route(lanelet_route, x, y)
        assert cur_ll, "Cannot get current lane from x={}, y={}".format(x, y)

        cur_lane = lanelet_route.fullLane(cur_ll)
        assert cur_lane

        path = []
        # append centerline of each lanelet in the lane
        for ll in cur_lane:
            for p in ll.centerline:
                path.append(Point3d(0, p.x, p.y, 0.0))

        # add a padding at the end of the path
        path_length = 0
        next_ll = self.get_next(cur_lane[-1])
        while next_ll is not None and path_length < meters_after_end:
            for p, q in pairwise(next_ll.centerline):
                # for first iteration, also append p
                if path_length == 0:
                    path.append(Point3d(0, p.x, p.y, 0.0))

                path.append(Point3d(0, q.x, q.y, 0.0))
                path_length += distance(p, q)

                if path_length >= meters_after_end: break
            next_ll = self.get_next(next_ll)

        path_ls = ConstLineString3d(0, path)
        return path_ls

    def get_points(self,  x_min=0, y_min=0, x_max=0, y_max=0):
        data = []
        if (x_min == x_max == y_min == y_max):
            points = self.lanelet_map.pointLayer
        else:
            searchBox = BoundingBox2d(BasicPoint2d(x_min, y_min), BasicPoint2d(x_max, y_max))
            points = self.lanelet_map.pointLayer.search(searchBox)

        data = np.array([[pt.x for pt in points], [pt.y for pt in points]])
        return data

    def get_lines(self, x_min=0, y_min=0, x_max=0, y_max=0):
        data = []

        if (x_min == x_max == y_min == y_max):
            lines = self.lanelet_map.lineStringLayer
        else:
            searchBox = BoundingBox2d(BasicPoint2d(x_min, y_min), BasicPoint2d(x_max, y_max))
            lines = self.lanelet_map.lineStringLayer.search(searchBox)

        for line in lines:
            xs = [pt.x for pt in line]
            ys = [pt.y for pt in line]
            data.append([xs,ys])

        return data

    def plot_all_lanelets(self, x_min=0, y_min=0, x_max=0, y_max=0):
        """ Plots all lanelets within given boundaries.
            Center line is ommited.
            @param drawline: draw as lines (heavier). If false, draw as points.
            @param split: split left and right lanelet into using color scheme.
        """
        if (x_min == x_max == y_min == y_max):
            lanelets = self.lanelet_map.laneletLayer
        else:
            searchBox = BoundingBox2d(BasicPoint2d(x_min, y_min), BasicPoint2d(x_max, y_max))
            lanelets = self.lanelet_map.laneletLayer.search(searchBox)

        for lanelet in lanelets:
            xs = [pt.x for pt in lanelet.rightBound]
            ys = [pt.y for pt in lanelet.rightBound]
            plt.plot(xs, ys, 'b-')
            xs = [pt.x for pt in lanelet.leftBound]
            ys = [pt.y for pt in lanelet.leftBound]
            plt.plot(xs, ys, 'g-')


    def plot_lanelet_ids(self, lanelet_ids):
        for ll_id in lanelet_ids:
            LaneletMap.plot_ll(self.lanelet_map.laneletLayer[ll_id])

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
        point_on_centerline = project(ConstLineString2d(lanelet.centerline), BasicPoint2d(x, y))
        # project on left and right bounds
        point_on_leftbound = project(ConstLineString2d(lanelet.leftBound), point_on_centerline)
        point_on_rightbound = project(ConstLineString2d(lanelet.rightBound), point_on_centerline)
        return distance(point_on_leftbound, point_on_rightbound)

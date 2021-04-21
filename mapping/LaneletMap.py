#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# LaneletMap
# Class to parse a Lanelet 2 map and interface with lanelet2 library
# --------------------------------------------

import lanelet2
from lanelet2.core import getId, BasicPoint2d, BasicPoint3d, Point3d, Point2d, ConstPoint2d, ConstPoint3d, BoundingBox2d, BoundingBox3d, LineString3d, LineString2d, ConstLineString2d, ConstLineString3d, Lanelet, RegulatoryElement, TrafficLight
from lanelet2.geometry import distance, to2D, boundingBox2d, boundingBox3d,inside, toArcCoordinates, project, length2d, findNearest, intersects2d, intersects3d
from lanelet2.traffic_rules import Locations, Participants
from lanelet2.projection import UtmProjector
from lanelet2.routing import RelationType, Route

from matplotlib import pyplot as plt
import numpy as np
import glog as log
from typing import List

from util.Utils import pairwise


class LaneletMap(object):
    def __init__(self):
        self.lanelet_map = None

    def load_lanelet_map(self, filename, projector):
        self.lanelet_map, errors = lanelet2.io.loadRobust(filename, projector)
        assert not errors, log.error(errors)

        self.projector = projector

        # generate routing table
        traffic_rules = lanelet2.traffic_rules.create(Locations.Germany, Participants.Vehicle)
        self.routing_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, traffic_rules)

        # Add participant attribute to all lanelets
        for elem in self.lanelet_map.laneletLayer:
            # ensure every lanelet has a subtype (default: road)
            if "subtype" not in elem.attributes:
                elem.attributes["subtype"] = "road"

            if elem.attributes["subtype"] == "road":
                elem.attributes["participant:vehicle"] = "yes"
            elif elem.attributes["subtype"] in ["crosswalk", "walkway", "traffic_island"]:
                elem.attributes["one_way"] = "no"
                elem.attributes["participant:pedestrian"] = "yes"


    def get_right(self, lanelet):
        return self.routing_graph.right(lanelet)

    def get_left(self, lanelet):
        return self.routing_graph.left(lanelet)

    def get_next(self, lanelet):
        return self.routing_graph.following(lanelet)

    def get_previous(self, lanelet:Lanelet):
        return self.routing_graph.previous(lanelet)

    def get_right_by_route(self, lanelet_route:Route, lanelet:Lanelet):
        # NOTE: lanelet must be on lanelet_route

        right = []

        right_relations = lanelet_route.rightRelations(lanelet)
        for relation in right_relations:
            if relation.relationType == RelationType.Right:
                right.append(relation.lanelet)

        return right

    def get_left_by_route(self, lanelet_route:Route, lanelet:Lanelet):
        # NOTE: lanelet must be on lanelet_route

        left = []

        left_relations = lanelet_route.leftRelations(lanelet)
        for relation in left_relations:
            if relation.relationType == RelationType.Left:
                left.append(relation.lanelet)

        return left

    def get_next_by_route(self, lanelet_route:Route, lanelet:Lanelet):
        # NOTE: lanelet must be on lanelet_route

        following = []

        following_relations = lanelet_route.followingRelations(lanelet)
        for relation in following_relations:
            following.append(relation.lanelet)

        return following

    def get_previous_by_route(self, lanelet_route:Route, lanelet:Lanelet):
        # NOTE: lanelet must be on lanelet_route

        previous = []

        previous_relations = lanelet_route.previousRelations(lanelet)
        for relation in previous_relations:
            previous.append(relation.lanelet)

        return previous

    def get_traffic_light_by_name(self, name):
        #filter regulatory elemments for TL only
        tl_res = list(filter(lambda res: isinstance(res, TrafficLight), self.lanelet_map.regulatoryElementLayer))
        for tl_re in tl_res:
            #physical lights in the re
            for tl in tl_re.trafficLights:
                tlname = tl.attributes['name']
                if name.casefold() == tlname.casefold():
                    #print(name+" and "+tlname)
                    return tl_re
        return None


    def get_traffic_light_pos(self, id):
        tl = self.lanelet_map.regulatoryElementLayer[id]
        x = 0
        y = 0
        linepoints = []
        if tl:
            for physical_ligh in tl.trafficLights: #LineString3d
                x = physical_ligh[0].x #LineString3d node 0
                y = physical_ligh[0].y
            line = self.lanelet_map.lineStringLayer[tl.stopLine.id]
            xs = [pt.x for pt in line]
            ys = [pt.y for pt in line]
            linepoints = [xs,ys]
        return x,y,linepoints

    def get_crosswalks_entry_pts(self):
        all_xwalks = {}
        xwalk_id = 1

        for elem in self.lanelet_map.laneletLayer:
            if elem.attributes["subtype"] == "crosswalk":
                entrance_pt_left = np.array([elem.leftBound[0].x, elem.leftBound[0].y])
                entrance_pt_right = np.array([elem.rightBound[0].x, elem.rightBound[0].y])
                exit_pt_left = np.array([elem.leftBound[1].x, elem.leftBound[1].y])
                exit_pt_right = np.array([elem.rightBound[1].x, elem.rightBound[1].y])

                entrance_pt = (entrance_pt_left + entrance_pt_right) / 2
                exit_pt = (exit_pt_left + exit_pt_right) / 2

                all_xwalks[xwalk_id] = [entrance_pt, exit_pt]

                xwalk_id += 1

        return all_xwalks


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

        #for i in range(len(lanelets)):
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
                log.warn("Point {} part of more than one lanelet ({}), cannot automatically resolve.".format(
                    (x,y), [ll.id for ll in intersecting_lls]))
                return intersecting_lls[1]
        return intersecting_lls[0]

    def get_occupying_lanelet_by_participant(self, x, y, participant):
        point = BasicPoint2d(x, y)

        # get all intersecting lanelets using a trivial bounding box
        searchbox = BoundingBox2d(point, point)
        intersecting_lls = self.lanelet_map.laneletLayer.search(searchbox)

        participant_tag = "participant:" + participant

        if len(intersecting_lls) == 0:
            if participant == "vehicle":
                raise Exception("Lanelet Error: vehicle not part of any lanelet.")
            else:
                # case: non-vehicle agent is not on any lanelet
                return None
        elif len(intersecting_lls) > 1:
            # filter results for lanelets containing the point
            intersecting_lls = list(filter(lambda ll: inside(ll, point), intersecting_lls))
            # filter results for lanelets with allowed participants matching participant_tag
            participant_lls = list(filter(lambda ll: participant_tag in ll.attributes and ll.attributes[participant_tag] == "yes", intersecting_lls))

            # case: agent is on one lanelet where they are the allowed participant
            if len(participant_lls) == 1:
                return participant_lls[0]

            # case: agent is on more than one lanelet where they are the allowed participant
            if len(participant_lls) > 1:
                log.warn("Point {} part of more than one lanelet ({}), cannot automatically resolve.".format(
                    (x,y), [ll.id for ll in participant_lls]))
                return participant_lls[1]

            # case: agent is on more than one lanelet where they are NOT the allowed participant
            if len(intersecting_lls) > 1:
                log.warn("Point {} part of more than one lanelet ({}), cannot automatically resolve.".format(
                    (x,y), [ll.id for ll in intersecting_lls]))
                return intersecting_lls[1]

            # case: if agent is not on any lanelet (but bounding box intersected multiple lanelets)
            if len(intersecting_lls) == 0:
                return None

        return intersecting_lls[0]

    def get_occupying_area(self, x, y):
        point = BasicPoint2d(x, y)

        # get all intersecting areas using a trivial bounding box
        searchbox = BoundingBox2d(point, point)
        intersecting_areas = self.lanelet_map.areaLayer.search(searchbox)

        if len(intersecting_areas) == 0:
            return None

        if len(intersecting_areas) > 1:
            # filter results for areas containing the point
            intersecting_areas = list(filter(lambda area: inside(area, point), intersecting_lls))
            if len(intersecting_areas) > 1:
                log.warn("Point {} part of more than one area ({}), cannot automatically resolve.".format(
                    (x,y), [area.id for area in intersecting_areas]))
                return intersecting_areas[1]

        if len(intersecting_areas) == 0:
            return None

        return intersecting_areas[0]

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

        try:
            route_submap = lanelet_route.laneletSubmap().laneletMap()
        except Exception:
            try:
                route_submap = lanelet_route.laneletMap()
            except Exception:
                log.error("Lanelet Map can't find route submap")
                return

        for ll in route_submap.laneletLayer:
                if inside(ll, BasicPoint2d(x, y)):
                    ret = ll
                    break
        return ret

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

    def get_route_lines(self, route):
        data = []

        lines = route.laneletMap().lineStringLayer

        for line in lines:
            xs = [pt.x for pt in line]
            ys = [pt.y for pt in line]
            data.append([xs,ys])

        return data

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

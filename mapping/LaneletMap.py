#!/usr/bin/env python3
# rqueiroz@uwaterloo.ca
# d43sharm@uwaterloo.ca
# mantkiew@uwaterloo.ca
# --------------------------------------------
# LaneletMap
# Class to parse a Lanelet 2 map and interface with lanelet2 library
# --------------------------------------------

import lanelet2
from lanelet2.core import getId, BasicPoint2d, BasicPoint3d, Point3d, Point2d, ConstPoint2d, ConstPoint3d, BoundingBox2d, BoundingBox3d, LineString3d, LineString2d, ConstLineString2d, ConstLineString3d, Lanelet, RegulatoryElement, TrafficLight, AllWayStop, RightOfWay
from lanelet2.geometry import distance, to2D, boundingBox2d, boundingBox3d, inside, toArcCoordinates, project, length2d, findNearest, intersects2d, intersects3d
from lanelet2.traffic_rules import Locations, Participants
from lanelet2.routing import RelationType, Route

from matplotlib import pyplot as plt
import numpy as np
from typing import List
import logging
log = logging.getLogger(__name__)
from util.Utils import pairwise


class LaneletMap(object):
    def __init__(self):
        self.lanelet_map = None
        #cache
        self.stop_lines = None
        self.stop_signs = None
        self.all_way_stops = None

    def load_lanelet_map(self, filename, projector):
        self.lanelet_map, errors = lanelet2.io.loadRobust(filename, projector)
        assert not errors, log.error(errors)

        self.projector = projector

        # Set up circular references from reg elems
        for regelem in self.lanelet_map.regulatoryElementLayer:
            if isinstance(regelem, RightOfWay):
                log.info(
                    f'Setting circular references for RightOfWay {regelem.id}')
                for lanelet in regelem.yieldLanelets():
                    if regelem not in lanelet.regulatoryElements:
                        lanelet.addRegulatoryElement(regelem)
                        log.info(f'Added to lanelet {lanelet.id}')
            if isinstance(regelem, AllWayStop):
                log.info(
                    f'Setting circular references for AllWayStop {regelem.id}')
                for lanelet in regelem.lanelets():
                    if regelem not in lanelet.regulatoryElements:
                        lanelet.addRegulatoryElement(regelem)
                        log.info(f'Added to lanelet {lanelet.id}')

        # generate routing table
        traffic_rules = lanelet2.traffic_rules.create(
            Locations.Germany, Participants.Vehicle)
        self.routing_graph = lanelet2.routing.RoutingGraph(
            self.lanelet_map, traffic_rules)

        # generate routing table specifically for pedestrians
        traffic_rules_pedestrians = lanelet2.traffic_rules.create(
            Locations.Germany, Participants.Pedestrian)
        self.routing_graph_pedestrians = lanelet2.routing.RoutingGraph(
            self.lanelet_map, traffic_rules_pedestrians)

        # Add participant attribute to all lanelets
        for elem in self.lanelet_map.laneletLayer:
            # ensure every lanelet has a subtype (default: road)
            if "subtype" not in elem.attributes:
                elem.attributes["subtype"] = "road"

            if elem.attributes["subtype"] == "road":
                elem.attributes["participant:vehicle"] = "yes"
            elif elem.attributes["subtype"] in ["crosswalk", "walkway", "traffic_island", "walking_area"]:
                elem.attributes["one_way"] = "no"
                elem.attributes["participant:pedestrian"] = "yes"

    def get_conflicting_by_route(self, lanelet_route:Route, lanelet:Lanelet, with_intersecting_points = False):
        ''''
            Returns a list of conflicting Lanelets with both current and succeeding
            lanelets based on the route. 
            The list is order by distance from lanelet start 
            and a longitudinal distance from start is assigned
        '''
        conflicting = []
        for c in lanelet_route.conflictingInMap(lanelet):
            conflicting.append(c)
            if with_intersecting_points:
                intersecting_points = [] #BasicPoints2d 
                self.lanelet_map.intersection(
                    lanelet.centerline2d.basicLineString, 
                    c.centerline2d.basicLineString, 
                    intersecting_points)
        return conflicting

    def get_right(self, lanelet):
        #routable lanelet, if exists
        right_ll =  self.routing_graph.right(lanelet)
        if right_ll:
            return right_ll,  "routable"
        #adjacent, non-routable lanelet, if exists
        right_ll = self.routing_graph.adjacentRight(lanelet) # allowLaneChanges=True # param not available
        if right_ll:
            return right_ll, "adjacent"
        #adjacent, but opposite direction
        lls = self.lanelet_map.laneletLayer.findUsages(lanelet.rightBound.invert())
        if len(lls) > 0:
            right_ll = lls[0]
        if len(lls) > 1:
            log.warning("multiple right adjacent lanelets found for {}. Using {}".format(lanelet.id, right_ll.id))
        if right_ll:
            return right_ll, "opposite"
        #Not found
        return None, ""

    def get_left(self, lanelet):
        #routable lanelet, if exists
        left_ll =  self.routing_graph.left(lanelet)
        if left_ll:
            return left_ll,  "routable"
        #adjacent, non-routable lanelet, if exists
        left_ll = self.routing_graph.adjacentLeft(lanelet) # allowLaneChanges=True # param not available
        if left_ll:
            return left_ll, "adjacent"
        #adjacent, but opposite direction
        lls = self.lanelet_map.laneletLayer.findUsages(lanelet.leftBound.invert())
        for ll in lls:
            left_ll = ll
        if len(lls) > 1:
            log.warning("multiple left adjacent lanelets found for {}. Using {}".format(lanelet.id, left_ll.id))
        if left_ll:
            return left_ll, "opposite"
        #Not found
        return None, ""

    def get_next(self, lanelet):
        return self.routing_graph.following(lanelet)

    def get_previous(self, lanelet: Lanelet):
        return self.routing_graph.previous(lanelet)

    def get_right_by_route(self, lanelet_route: Route, lanelet: Lanelet):
        # NOTE: lanelet must be on lanelet_route

        right = []

        right_relations = lanelet_route.rightRelations(lanelet)
        for relation in right_relations:
            if relation.relationType == RelationType.Right:
                right.append(relation.lanelet)

        return right

    def get_left_by_route(self, lanelet_route: Route, lanelet: Lanelet):
        # NOTE: lanelet must be on lanelet_route

        left = []
        # Provides all lanelets left of a given lanelet within the Route
        left_relations = lanelet_route.leftRelations(lanelet)
        for relation in left_relations:
            if relation.relationType == RelationType.Left:
                left.append(relation.lanelet)

        return left

    def get_next_by_route(self, lanelet_route: Route, lanelet: Lanelet):
        # NOTE: lanelet must be on lanelet_route

        next = []
        for relation in lanelet_route.followingRelations(lanelet):
            next.append(relation.lanelet)
        return next

    def get_previous_by_route(self, lanelet_route: Route, lanelet: Lanelet):
        # NOTE: lanelet must be on lanelet_route

        previous = []

        previous_relations = lanelet_route.previousRelations(lanelet)
        for relation in previous_relations:
            previous.append(relation.lanelet)

        return previous

    def get_next_sequence_by_route(self, lanelet_route: Route, lanelet: Lanelet, distance = None):
        ''' Returns a single chain of next lanelets on the route, up to the distance threshold
            Lanelet must be on lanelet_route
        '''
        sequence = []
        for relation in lanelet_route.followingRelations(lanelet):
            sequence.append(relation.lanelet)
            # length of centerline in 2d
            distance_covered = length2d(relation.lanelet)
            #print(distance_covered)
            #include next lanelets if distance not covered
            if (distance is not None) and (distance_covered < distance):
                next_relations = lanelet_route.followingRelations(
                    relation.lanelet)
                #only include following lanelets if it is a single path forward, otherwise can't make assumptions of continuity
                #this will avoid mixing up paths with intersections
                if len(next_relations) == 1:
                    next_lanelet = next_relations[0].lanelet
                    sequence.append(next_lanelet)
                    distance_covered += length2d(next_lanelet)
            #print("Following sequence {} distance {}".format(len(sequence),distance_covered))
        return sequence

    def route_full_lane(self, lanelet_route: Route, lanelet: Lanelet):
        # NOTE: this is a wrapper for lanelet_route.fullLane(lanelet)
        # Currently, if lanelet is on lanelet_route.shortestPath(), then there are
        # cases where lanelet_route.fullLane(lanelet) is not the full lane that
        # is in the shortest path (i.e., the shortest path contains more of the lane)
        # E.g., when the shortest path passes through one of the cul-de-sacs in
        # scenarios/maps/experimental_maps/lanelet2_bathurst.osm, then
        # lanelet_route.fullLane(lanelet) will end before the cul-de-sac, but the
        # lane containing lanelet in the shortest path will include the cul-de-sac

        full_lane = []

        shortest_path = lanelet_route.shortestPath()
        lanelet_on_path = False
        next_lls = []
        for ll in shortest_path:
            append = False
            for next_ll in next_lls:
                if (next_ll.id == ll.id) and (ll.id != full_lane[0].id):
                    append = True
                    break

            if append:
                full_lane.append(ll)
            elif lanelet_on_path:
                break
            else:
                full_lane = [ll]

            if ll.id == lanelet.id:
                lanelet_on_path = True

            next_lls = self.get_next_by_route(lanelet_route, ll)

        if not lanelet_on_path:
            full_lane = lanelet_route.fullLane(lanelet)

        return full_lane

    def get_traffic_light_by_name(self, name):
        #filter regulatory elemments for TL only
        tl_res = list(filter(lambda res: isinstance(
            res, TrafficLight), self.lanelet_map.regulatoryElementLayer))
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
            for physical_ligh in tl.trafficLights:  # LineString3d
                x = physical_ligh[0].x  # LineString3d node 0
                y = physical_ligh[0].y
            line = self.lanelet_map.lineStringLayer[tl.stopLine.id]
            xs = [pt.x for pt in line]
            ys = [pt.y for pt in line]
            linepoints = [xs, ys]
        return x, y, linepoints

    def get_crosswalks(self):
        return [ll for ll in self.lanelet_map.laneletLayer if ll.attributes["subtype"] == "crosswalk"]

    def get_stop_lines(self):
        if self.stop_lines is None:
            self.stop_lines = []
            for ls in self.lanelet_map.lineStringLayer:
                if "type" in ls.attributes:
                    type = ls.attributes["type"]
                    if type == "stop_line":
                        self.stop_lines.append(ls)
        return self.stop_lines

    def get_stop_signs(self):
        if self.stop_signs is None:
            self.stop_signs = []
            for ls in self.lanelet_map.lineStringLayer:
                if "type" in ls.attributes:
                    if ls.attributes["type"] == "traffic_sign":
                        if ls.attributes["subtype"] == "usR1-1":
                            self.stop_signs.append(ls[0])  # first node only
            for node in self.lanelet_map.pointLayer:
                if "type" in node.attributes:
                    if node.attributes["type"] == "traffic_sign":
                        if node.attributes["subtype"] == "usR1-1":
                            self.stop_signs.append(node)

        return self.stop_signs

    def get_my_ref_line(self, my_ll, yield_lanelets, stop_lines):
        assert(len(stop_lines) == len(yield_lanelets))
        my_index = None
        for i in range(len(yield_lanelets)):
            if my_ll.id == yield_lanelets[i].id:
                my_index = i
                break
        if my_index is not None:
            return stop_lines[my_index]

        return None

    def get_all_way_stops(self):
        signs = []
        stop_lines = []
        lanelets = []
        #aws_res = [re for re in self.lanelet_map.regulatoryElementLayer if re.attributes["subtype"] == "all_way_stop"] <-- alternatve using primitives
        aws_res = list(filter(lambda res: isinstance(
            res, AllWayStop), self.lanelet_map.regulatoryElementLayer))
        for aws_re in aws_res:
            #Stop Lines
            #for stop_line in aws_re.parameters["ref_line"]:
            for stop_line in aws_re.stopLines():
                stop_lines.append(stop_line)

            #Stop Signs
            #for stop_sign in aws_re.trafficSigns(): #<--- the trafficSigns() method does not work if the sign exists as Node instead of Way. Using the primitive instead.
            for stop_sign in aws_re.parameters["refers"]:
                #print(type(stop_sign))
                if isinstance(stop_sign, ConstPoint3d):
                    signs.append([stop_sign.x, stop_sign.y])

                elif isinstance(stop_sign, ConstLineString3d):
                    signs.append([stop_sign[0].x, stop_sign[0].y])

            #Yielding Lanelets
            #for lanelet in aws_re.parameters["yield"]: #<-- No parameter as yield and no python convertion since they are ConstWeakLanelet
            for ll in aws_re.lanelets():
                lanelets.append(ll)

        return signs, stop_lines, lanelets

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

    def get_route(self, from_lanelet_id: int, to_lanelet_id: int):
        from_ll = self.lanelet_map.laneletLayer[from_lanelet_id]
        to_ll = self.lanelet_map.laneletLayer[to_lanelet_id]
        # Route object automatically constructs a sub-laneletmap
        route = self.routing_graph.getRoute(from_ll, to_ll)
        assert route
        return route

    def get_pedestrian_route(self, from_lanelet_id: int, to_lanelet_id: int):
        from_ll = self.lanelet_map.laneletLayer[from_lanelet_id]
        to_ll = self.lanelet_map.laneletLayer[to_lanelet_id]
        # Route object automatically constructs a sub-laneletmap
        route = self.routing_graph_pedestrians.getRoute(from_ll, to_ll)
        assert route
        return route

    def get_route_via(self, lanelets: List[Lanelet]):
        assert len(lanelets) >= 2

        #for i in range(len(lanelets)):
        if len(lanelets) > 2:
            route = self.routing_graph.getRouteVia(
                lanelets[0], lanelets[1:-1], lanelets[-1])
        else:
            route = self.routing_graph.getRoute(lanelets[0], lanelets[1])
        assert route
        return route

    def get_shortest_path(self, from_lanelet_id: int, to_lanelet_id: int):
        route = self.get_route(from_lanelet_id, to_lanelet_id)
        shortest_path = route.shortestPath()
        assert(shortest_path)
        return shortest_path

    def get_pedestrian_shortest_path(self, from_lanelet_id: int, to_lanelet_id: int):
        route = self.get_pedestrian_route(from_lanelet_id, to_lanelet_id)
        shortest_path = route.shortestPath()
        assert(shortest_path)
        return shortest_path

    def get_occupying_lanelet(self, x, y):
        point = BasicPoint2d(x, y)

        # get all intersecting lanelets using a trivial bounding box
        searchbox = BoundingBox2d(point, point)
        intersecting_lls = self.lanelet_map.laneletLayer.search(searchbox)

        if len(intersecting_lls) == 0:
            #raise Exception("Lanelet Error: vehicle not part of any lanelet.")
            log.error("Lanelet Error: vehicle not part of any lanelet.")
            return None
        elif len(intersecting_lls) > 1:
            # filter results for lanelets containing the point
            intersecting_lls = list(
                filter(lambda ll: inside(ll, point), intersecting_lls))
            if len(intersecting_lls) > 1:
                #log.warning("Point {} part of more than one lanelet ({}), cannot automatically resolve.".format(
                #    (x, y), [ll.id for ll in intersecting_lls]))
                return intersecting_lls[1]
        return intersecting_lls[0]

    def get_all_occupying_lanelets(self, x, y):
        ''' Returns a list of overlapping lanelets.
            If not match, returns empty list."
         '''
        point = BasicPoint2d(x, y)
        searchbox = BoundingBox2d(point, point)
        intersecting_lls = self.lanelet_map.laneletLayer.search(searchbox)
        if len(intersecting_lls) > 1:
            # filter results for lanelets containing the point
            intersecting_lls = list(
                filter(lambda ll: inside(ll, point), intersecting_lls))
        return intersecting_lls

    def get_occupying_lanelet_by_participant(self, x, y, participant):
        point = BasicPoint2d(x, y)

        # get all intersecting lanelets using a trivial bounding box
        searchbox = BoundingBox2d(point, point)
        intersecting_lls = self.lanelet_map.laneletLayer.search(searchbox)

        participant_tag = "participant:" + participant

        if len(intersecting_lls) == 0:
            if participant == "vehicle":
                raise Exception(
                    "Lanelet Error: vehicle not part of any lanelet.")
            else:
                # case: non-vehicle agent is not on any lanelet
                return None
        elif len(intersecting_lls) > 1:
            # filter results for lanelets containing the point
            intersecting_lls = list(
                filter(lambda ll: inside(ll, point), intersecting_lls))
            # filter results for lanelets with allowed participants matching participant_tag
            participant_lls = list(filter(
                lambda ll: participant_tag in ll.attributes and ll.attributes[participant_tag] == "yes", intersecting_lls))

            # case: agent is on one lanelet where they are the allowed participant
            if len(participant_lls) == 1:
                return participant_lls[0]

            # case: agent is on more than one lanelet where they are the allowed participant
            if len(participant_lls) > 1:
                # log.warning("Point {} part of more than one lanelet ({}), cannot automatically resolve.".format(
                #     (x,y), [ll.id for ll in participant_lls]))
                return participant_lls[1]

            # case: agent is on more than one lanelet where they are NOT the allowed participant
            if len(intersecting_lls) > 1:
                # log.warning("Point {} part of more than one lanelet ({}), cannot automatically resolve.".format(
                #     (x,y), [ll.id for ll in intersecting_lls]))
                return intersecting_lls[1]

            # case: if agent is not on any lanelet (but bounding box intersected multiple lanelets)
            if len(intersecting_lls) == 0:
                return None

        return intersecting_lls[0]

    def get_spaces_list_occupied_by_pedestrian(self, position):
        spaces_list = {'lanelets': [], 'areas': []}

        point = BasicPoint2d(position[0], position[1])

        # get all intersecting lanelets using a trivial bounding box
        searchbox = BoundingBox2d(point, point)
        intersecting_lls = self.lanelet_map.laneletLayer.search(searchbox)
        intersecting_areas = self.lanelet_map.areaLayer.search(searchbox)

        # filter lanelets
        if len(intersecting_lls) > 0:
            # filter results for lanelets containing the point
            intersecting_lls = list(
                filter(lambda ll: inside(ll, point), intersecting_lls))
            # filter results for lanelets with allowed participants matching participant_tag
            pedestrian_lls = list(filter(
                lambda ll: 'participant:pedestrian' in ll.attributes and ll.attributes['participant:pedestrian'] == "yes", intersecting_lls))
            spaces_list['lanelets'] = pedestrian_lls

        # filter areas
        if len(intersecting_areas) > 0:
            # filter results for lanelets containing the point
            intersecting_areas = list(
                filter(lambda area: distance(point, area) <= 0.0, intersecting_areas))
            # note: pedestrians are allowed in all areas so there is no need to filter by participant
            spaces_list['areas'] = intersecting_areas

        return spaces_list

    def get_occupying_area(self, x, y):
        point = BasicPoint2d(x, y)

        # get all intersecting areas using a trivial bounding box
        searchbox = BoundingBox2d(point, point)
        intersecting_areas = self.lanelet_map.areaLayer.search(searchbox)

        if len(intersecting_areas) == 0:
            return None

        if len(intersecting_areas) > 1:
            # filter results for areas containing the point
            intersecting_areas = list(
                filter(lambda area: inside(area, point), intersecting_lls))
            if len(intersecting_areas) > 1:
                log.warning("Point {} part of more than one area ({}), cannot automatically resolve.".format(
                    (x, y), [area.id for area in intersecting_areas]))
                return intersecting_areas[1]

        if len(intersecting_areas) == 0:
            return None

        return intersecting_areas[0]

    def inside_lanelet_or_area(self, position, lanelet_or_area):
        point = BasicPoint2d(position[0], position[1])
        if lanelet_or_area.attributes['type'] == 'multipolygon':
            return distance(point, lanelet_or_area) <= 0.0
        return inside(lanelet_or_area, point)

    def get_occupying_lanelet_in_reference_path(self, ref_path, lanelet_route, x, y):
        """ Returns the lanelet closest to (x,y) that lies along ref_path.
            Does not check if (x,y) lies inside the lanelet.

            @param lanelet_route:   the lanelet2 Route object that ref_path goes through.
        """
        point_on_path = project(
            ConstLineString2d(ref_path), BasicPoint2d(x, y))
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

    def area_and_lanelet_share_node(self, area, lanelet):
        """ Return True if either left or right bounds of the lanelet share a node with the area
        """
        share_area_node_with_left = any(
            [node in area.outerBound[0] for node in lanelet.leftBound])
        share_area_node_with_right = any(
            [node in area.outerBound[0] for node in lanelet.rightBound])

        return share_area_node_with_left or share_area_node_with_right

    #deprecated?
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

                if path_length >= meters_after_end:
                    break
            next_ll = self.get_next(next_ll)

        path_ls = ConstLineString3d(0, path)
        return path_ls

    def get_points(self,  x_min=0, y_min=0, x_max=0, y_max=0):
        data = []
        if (x_min == x_max == y_min == y_max):
            points = self.lanelet_map.pointLayer
        else:
            searchBox = BoundingBox2d(BasicPoint2d(
                x_min, y_min), BasicPoint2d(x_max, y_max))
            points = self.lanelet_map.pointLayer.search(searchBox)

        data = np.array([[pt.x for pt in points], [pt.y for pt in points]])
        return data

    def get_lines(self, x_min=0, y_min=0, x_max=0, y_max=0):
        data = []

        if (x_min == x_max == y_min == y_max):
            lines = self.lanelet_map.lineStringLayer
        else:
            searchBox = BoundingBox2d(BasicPoint2d(
                x_min, y_min), BasicPoint2d(x_max, y_max))
            lines = self.lanelet_map.lineStringLayer.search(searchBox)

        for line in lines:
            xs = [pt.x for pt in line]
            ys = [pt.y for pt in line]
            type = 'virtual'  # all unmarked lines are assumed to be virtual
            try:
                type = line.attributes["type"]
            except KeyError:
                pass
            subtype = None
            try:
                subtype = line.attributes["subtype"]
            except KeyError:
                pass
            data.append((xs, ys, type, subtype))

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
            searchBox = BoundingBox2d(BasicPoint2d(
                x_min, y_min), BasicPoint2d(x_max, y_max))
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

    def plot_lanelets(self, lanelets: iter):
        for ll in lanelets:
            # LaneletMap.plot_ll(self.lanelet_map.laneletLayer[ll_id])
            LaneletMap.plot_ll(ll)

    def get_route_lines(self, route):
        data = []

        lines = route.laneletMap().lineStringLayer

        for line in lines:
            xs = [pt.x for pt in line]
            ys = [pt.y for pt in line]
            data.append([xs, ys])

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
        point_on_centerline = project(ConstLineString2d(
            lanelet.centerline), BasicPoint2d(x, y))
        # project on left and right bounds
        point_on_leftbound = project(ConstLineString2d(
            lanelet.leftBound), point_on_centerline)
        point_on_rightbound = project(ConstLineString2d(
            lanelet.rightBound), point_on_centerline)
        return distance(point_on_leftbound, point_on_rightbound)


def get_line_format(type: str, subtype: str):
    color = 'red'  # color for unhandled type
    linestyle = 'solid'  # most lines are solid
    linewidth = 1  # default width
    if type == 'road_border':
        color = 'black'
        linewidth = 2
    elif type == 'guard_rail':
        color = 'purple'
        linewidth = 2
    elif type == 'virtual':
        color = 'lightgray'
        linestyle = 'dotted'
    elif type == 'line_thin':
        color = 'gray'
        linestyle = subtype
    elif (type == 'line_thick' or type == 'stop_line' or type == 'stop'):
        color = 'gray'
        linestyle = subtype
        linewidth = 3
    elif type == 'curbstone':
        color = 'darkgray'
        if subtype == 'high':
            linewidth = 2
    elif type == 'pedestrian_marking':
        color = 'gray'
        linestyle = 'dashed'
    elif type == 'zebra_marking':
        color = 'gray'
        linestyle = 'dashed'
        linewidth = 2
    elif type == 'bump':
        color = 'lightgray'
        linestyle = 'solid'
        linewidth = 4
    elif type == "traffic_light":
        return None  # do not draw
    else:
        log.error(f'Unhandled format of line type: {type}, subtype: {subtype}')
    return (color, linestyle, linewidth)

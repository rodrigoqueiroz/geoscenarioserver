from math import floor, ceil

from lanelet2.core import ConstLineString3d, LaneletSequence, Point3d
from matplotlib import pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev

from mapping.LaneletMap import LaneletMap
import SimConfig
from util.Transformations import OutsideRefPathException, sim_to_frenet_position
from util.Utils import distance_2p

class SDVRoute(object):
    lanelet_map = None

    def __init__(self, lanelet_route, lanelet_map:LaneletMap, start_x=None, start_y=None):
        # NOTE: start_x and start_y should only be None in the case where x and 
        #       y aren't known; e.g., when starting with frenet state

        if SDVRoute.lanelet_map is None:
            SDVRoute.lanelet_map = lanelet_map

        self._lanelet_route = lanelet_route

        self._sdv_paths = None
        self._current_sdv_path = None
        self._set_sdv_paths()

        # This is True if the vehicle should lane change to stay on its route
        self._should_lane_change:bool = False

        self.update_global_path(start_x, start_y)

    def get_global_path(self):
        return self._current_sdv_path.get_global_path()

    def get_lane_change_direction(self, s):
        start, end, direction = self._current_sdv_path.get_lane_change_info()

        if self._should_lane_change:
            return direction
        return None

    def get_reference_path(self):
        return self._current_sdv_path.get_ref_path()

    def get_reference_path_s_start(self):
        return self._current_sdv_path.get_ref_path_s_start()

    def update_global_path(self, x, y):
        if (x is None) or (y is None):
            curr_ll = self._lanelet_route.shortestPath()[0]
        else:
            curr_ll = SDVRoute.lanelet_map.get_occupying_lanelet_by_route(
                self._lanelet_route, x, y
            )
        assert curr_ll

        for sdv_path in self._sdv_paths:
            lane = sdv_path.get_lane()
            if any(ll.id == curr_ll.id for ll in lane):
                self._current_sdv_path = sdv_path
                break

        if (x is None) or (y is None):
            x = curr_ll.centerline[0].x
            y = curr_ll.centerline[0].y
        s = sim_to_frenet_position(self.get_global_path(), x, y, 0)[0]
        self._update_should_lane_change(s, self.get_global_path(), 0)

    def update_reference_path(self, ref_path_origin:float, plan_lane_change=False):
        self._current_sdv_path.update_ref_path(ref_path_origin)

        if plan_lane_change:
            self._update_should_lane_change(
                0, self.get_reference_path(), self.get_reference_path_start_s()
            )

    def _set_sdv_paths(self):
        self._sdv_paths = []

        route_lls = self._lanelet_route.laneletSubmap().laneletLayer
        visited_route_lls = []
        for route_ll in route_lls:
            if any(ll.id == route_ll.id for ll in visited_route_lls):
                continue

            route_lane = self._lanelet_route.fullLane(route_ll)

            next_ll_in_route = SDVRoute.lanelet_map.get_next_by_route(self._lanelet_route, route_lane[-1])
            lane_is_loop = (
                (next_ll_in_route is not None) and (next_ll_in_route.id == route_lane[0].id)
            )

            prev_lane = []
            if not lane_is_loop:
                prev_ll = SDVRoute.lanelet_map.get_previous(route_lane[0])
                while (prev_ll is not None):
                    if prev_ll.id == route_lane[-1].id:
                        lane_is_loop = True
                        break

                    prev_lane.append(prev_ll)
                    prev_ll = SDVRoute.lanelet_map.get_previous(prev_ll)

                prev_lane.reverse()

            next_lane = []
            if not lane_is_loop:
                next_ll = SDVRoute.lanelet_map.get_next(route_lane[-1])
                while (next_ll is not None):
                    if next_ll.id == route_lane[0].id:
                        lane_is_loop = True
                        break
                    
                    next_lane.append(next_ll)
                    next_ll = SDVRoute.lanelet_map.get_next(next_ll)

            # full_lane = prev_lane + route_lane + next_lane
            full_lane = prev_lane
            for ll in route_lane:
                full_lane.append(ll)
                visited_route_lls.append(ll)
            full_lane += next_lane

            remaining_lls = self._lanelet_route.remainingShortestPath(route_lane[0])
            if len(remaining_lls) == 0:
                # None of route_lane is on the shortest path
                lane_change_start = route_lane[0].centerline[0]
                lane_change_end = route_lane[-1].centerline[-1]
                lane_change_direction = None

                right_lls = SDVRoute.lanelet_map.get_right_by_route(self._lanelet_route, route_lane[0])
                for ll in right_lls:
                    if len(self._lanelet_route.remainingShortestPath(ll)) > 0:
                        lane_change_direction = -1
                        break

                if lane_change_direction is None:
                    left_lls = SDVRoute.lanelet_map.get_left_by_route(self._lanelet_route, route_lane[0])
                    for ll in left_lls:
                        if len(self._lanelet_route.remainingShortestPath(ll)) > 0:
                            lane_change_direction = 1
                            break

                assert lane_change_direction
            elif all(ll in route_lane for ll in remaining_lls):
                # The route lane is the shortest path
                lane_change_start = None
                lane_change_end = None
                lane_change_direction = None
            else:
                # The route lane is on the shortest path
                lane_change_start = None
                lane_change_end = route_lane[-1].centerline[-1]
                lane_change_direction = None

                right_lls = SDVRoute.lanelet_map.get_right_by_route(self._lanelet_route, route_lane[-1])
                left_lls = SDVRoute.lanelet_map.get_left_by_route(self._lanelet_route, route_lane[-1])
                if right_lls and (right_lls[0] in remaining_lls):
                    lane_change_direction = -1
                    lane_change_start = right_lls[0].centerline[0]
                elif left_lls and (left_lls[0] in remaining_lls):
                    lane_change_start = left_lls[0].centerline[0]
                    lane_change_direction = 1
                
                assert lane_change_start, lane_change_direction

            self._sdv_paths.append(
                SDVPath(
                    full_lane, lane_is_loop,
                    lane_change_start, lane_change_end, lane_change_direction
                )
            )

        # for sdv_path in self._sdv_paths:
        #     map_lines = SDVRoute.lanelet_map.get_lines()
        #     for line in map_lines:
        #         plt.plot(line[0], line[1], '-', color='black')

        #     for ll in route_lls:
        #         l_lanelet_x = []
        #         l_lanelet_y = []
        #         for point in ll.leftBound:
        #             l_lanelet_x.append(point.x)
        #             l_lanelet_y.append(point.y)

        #         r_lanelet_x = []
        #         r_lanelet_y = []
        #         for point in ll.rightBound:
        #             r_lanelet_x.append(point.x)
        #             r_lanelet_y.append(point.y)
        #         plt.plot(l_lanelet_x, l_lanelet_y, '-', color='red')
        #         plt.plot(r_lanelet_x, r_lanelet_y, '-', color='red')

        #     global_path_x = []
        #     global_path_y = []
        #     for point in sdv_path.get_global_path():
        #         global_path_x.append(point.x)
        #         global_path_y.append(point.y)
        #     plt.plot(global_path_x, global_path_y, '.', color='blue')

        #     plt.axis([-50, 40, -50, 40])
        #     plt.show()

    def _update_should_lane_change(self, s, path, s_start):
        start, end, direction = self._current_sdv_path.get_lane_change_info()

        if (start is not None) and (end is not None):
            try:
                start_s = sim_to_frenet_position(
                    path, start.x, start.y, s_start
                )[0]
            except OutsideRefPathException:
                start_s = None

            try:
                end_s = sim_to_frenet_position(
                    path, end.x, end.y, s_start
                )[0]
            except OutsideRefPathException:
                end_s = None

            if (start_s is None) and (end_s is not None):
                # start_s is None because it lies before path
                self._should_lane_change = (s <= end_s)
            elif (start_s is not None) and (end_s is None):
                # end_s is None because it lies after path
                self._should_lane_change = (s >= start_s)
            elif (start_s is not None) and (end_s is not None):
                # both start_s and end_s lie on path
                self._should_lane_change = (s >= start_s) and (s <= end_s)
            # NOTE: start_s and end_s will both be None in the case where path is
            #       the reference path; end_s is gauranteed to lie on the global path
        else:
            # NOTE: start and end will either both be None, or both be a point
            self._should_lane_change = False

class SDVPath(object):
    def __init__(self, full_lane, lane_is_loop:bool, lane_change_start:Point3d, lane_change_end:Point3d, lane_change_direction:int):
        # These mark the start and end of a section on the path where a lane change
        # would be necessary to stay on the route
        self._lane_change_start:Point3d = lane_change_start
        self._lane_change_end:Point3d = lane_change_end
        self._lane_change_direction:int = lane_change_direction

        # Points per meter along the global and reference paths
        self._path_density:float = SimConfig.POINTS_PER_METER

        # The lanelets that make the global path
        self._lane = full_lane
        self._lane_is_loop:bool = lane_is_loop

        # The global path
        self._global_path = None
        self._global_path_len:int = 0
        self._set_global_path()

        # The reference path
        self._ref_path = None
        self._ref_path_s_start:float = 0.0
        self._ref_path_s_end:float = 0.0

    def get_global_path(self):
        # NOTE: global_path needs to be stored as a list because ConstLineString3d
        #       does not support slicing
        return None if (self._global_path is None) else ConstLineString3d(0, self._global_path)

    def get_lane(self):
        return self._lane

    def get_lane_change_info(self):
        return self._lane_change_start, self._lane_change_end, self._lane_change_direction

    def get_ref_path(self):
        return self._ref_path

    def get_ref_path_s_start(self):
        return self._ref_path_s_start

    def update_ref_path(self, ref_path_origin:float, ref_path_s_start:float=-100.0, ref_path_s_end:float=100.0):
        # NOTE: ref_path_origin is the position (s) of the origin of the reference
        #       path along the global path

        global_path_start_index, global_path_end_index = self._get_global_path_start_and_end_indices(
            ref_path_origin + ref_path_s_start,
            ref_path_origin + ref_path_s_end
        )

        # NOTE: _ref_path_s_start and _ref_path_s_end will be as close as possible
        #       to ref_path_s_start and ref_path_s_end respectively
        self._ref_path_s_start = self._path_index_to_s(global_path_start_index) - ref_path_origin
        self._ref_path_s_end = self._path_index_to_s(global_path_end_index) - ref_path_origin

        if self._lane_is_loop:
            global_path_start_index = global_path_start_index % self._global_path_len
            global_path_end_index = global_path_end_index % self._global_path_len
        else:
            # If the path is not a loop, then global_path_start_index cannot be
            # before index 0 and global_path_end_index cannot be after the last index
            global_path_start_index = max(0, global_path_start_index)
            global_path_end_index = min(self._global_path_len - 1, global_path_end_index)

            # In case the range changed
            self._ref_path_s_start = self._path_index_to_s(global_path_start_index) - ref_path_origin
            self._ref_path_s_end = self._path_index_to_s(global_path_end_index) - ref_path_origin

        # NOTE: assumes global_path_start_index never equals global_path_end_index
        if self._lane_is_loop and (global_path_start_index < global_path_end_index):
            # The slice lies between the ends of the global path
            self._ref_path = ConstLineString3d(
                0, self._global_path[ global_path_start_index : global_path_end_index ]
            )
        elif self._lane_is_loop and (global_path_start_index > global_path_end_index):
            # The slice wraps around the ends of the global path
            self._ref_path = ConstLineString3d(
                0, self._global_path[ global_path_start_index : ] +
                self._global_path[ : global_path_end_index ]
            )
        elif not self._lane_is_loop:
            # NOTE: it is assumed global_path_start_index < global_path_end_index
            #       in this case
            self._ref_path = ConstLineString3d(
                0, self._global_path[ global_path_start_index : global_path_end_index ]
            )

    def _get_global_path_start_and_end_indices(self, global_path_start_s:float, global_path_end_s:float):
        # NOTE: the range global_path_start_index to global_path_end_index will
        #       contain global_path_start_s and global_path_end_s
        global_path_start_index = floor(self._s_to_path_index(global_path_start_s))
        global_path_end_index = ceil(self._s_to_path_index(global_path_end_s))

        return global_path_start_index, global_path_end_index

    def _path_index_to_s(self, path_index:int):
        return float(path_index) / self._path_density

    def _s_to_path_index(self, s:float):
        # NOTE: the path index won't necessarily be an integer
        return s * self._path_density

    def _set_global_path(self):
        centerline_length = 0.0
        centerline_x = [ self._lane[0].centerline[0].x ]
        centerline_y = [ self._lane[0].centerline[0].y ]

        not_too_close_to_last_point = lambda x, y: (
            # (x, y) is a distance of 1cm or more from the last centerline point
            distance_2p(x, y, centerline_x[-1], centerline_y[-1]) >= 0.01
        )

        for ll in self._lane:
            for point in ll.centerline:
                if not_too_close_to_last_point(point.x, point.y):
                    centerline_length += distance_2p(
                        centerline_x[-1], centerline_y[-1], point.x, point.y
                    )

                    centerline_x.append(point.x)
                    centerline_y.append(point.y)

        # fit a spline to the centerline
        # a higher s means less closeness, and more smoothness, of fit
        tck, u = splprep([centerline_x, centerline_y], s=3, per=self._lane_is_loop)

        num_points = floor(self._path_density * centerline_length)
        # generate evenly spaced points along the spline
        spline = splev(np.linspace(0, 1, num=num_points), tck)

        if self._lane_is_loop:
            # if the last point is not removed then the distance between it and
            # the first point would be 0 instead of 1/_path_density
            spline[0] = spline[0][:-1]
            spline[1] = spline[1][:-1]
            num_points -= 1

        avg_meters_per_point = 0.0
        self._global_path = [ Point3d(0, spline[0][0], spline[1][0], 0.0) ]
        for x, y in zip(spline[0][1:], spline[1][1:]):
            avg_meters_per_point += distance_2p(x, y, self._global_path[-1].x, self._global_path[-1].y)
            self._global_path.append(Point3d(0, x, y, 0.0))
        avg_meters_per_point /= (num_points - 1)

        # update _path_density since num_points was floored, and splprep isn't exact
        self._path_density = 1.0 / avg_meters_per_point

        self._global_path_len = len(self._global_path)

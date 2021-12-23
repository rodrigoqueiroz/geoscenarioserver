from math import floor
from typing import List

from lanelet2.core import ConstLineString3d, Lanelet, Point3d
from lanelet2.routing import Route
from matplotlib import pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev

from gsc.GSParser import Node
from mapping.LaneletMap import LaneletMap, get_line_format
import SimConfig
from util.Transformations import OutsideRefPathException, sim_to_frenet_position
from util.Utils import distance_2p

class SDVRoute(object):
    lanelet_map = None

    def __init__(self, lanelet_route:Route, lanelet_map:LaneletMap, start_x:float=None, start_y:float=None, route_nodes:List[Node]=None):
        # NOTE: start_x and start_y should only be None in the case where x and 
        #       y aren't known; e.g., when starting with frenet state

        if SDVRoute.lanelet_map is None:
            SDVRoute.lanelet_map = lanelet_map

        self._lanelet_route:Route = lanelet_route

        self._sdv_paths:List[SDVPath] = None
        self._current_sdv_path:SDVPath = None
        self._set_sdv_paths()

        # This is True if the vehicle should lane swerve to stay on its route
        self._should_lane_swerve:bool = False

        if (start_x is None) or (start_y is None):
            curr_ll = self._lanelet_route.shortestPath()[0]
            start_x = curr_ll.centerline[0].x
            start_y = curr_ll.centerline[0].y
        self.update_global_path(start_x, start_y)
        start_s = sim_to_frenet_position(self.get_global_path(), start_x, start_y, 0)[0]
        self._update_should_lane_swerve(start_s, self.get_global_path(), 0)
        self.update_reference_path(start_s)

        self._route_nodes:List[Node] = route_nodes
        self._update_route_progress(True)

    def get_global_path(self):
        return self._current_sdv_path.get_global_path()

    def get_lane_swerve_direction(self, s:float):
        start, end, direction = self._current_sdv_path.get_lane_swerve_info()

        if self._should_lane_swerve:
            return direction
        return None

    def get_reference_path(self):
        return self._current_sdv_path.get_ref_path()

    def get_reference_path_origin(self):
        return self._current_sdv_path.get_ref_path_origin()

    def get_reference_path_s_start(self):
        return self._current_sdv_path.get_ref_path_s_start()

    def route_complete(self):
        if self._route_nodes is not None:
            return len(self._route_nodes) == 0
        return True

    def update_global_path(self, x:float, y:float):
        curr_ll = SDVRoute.lanelet_map.get_occupying_lanelet_by_route(
            self._lanelet_route, x, y
        )
        assert curr_ll

        for sdv_path in self._sdv_paths:
            lane = sdv_path.get_lane()
            if any(ll.id == curr_ll.id for ll in lane):
                self._current_sdv_path = sdv_path
                break

    def update_reference_path(self, ref_path_origin:float, plan_lane_swerve:bool=False, update_route_progress:bool=False):
        self._current_sdv_path.update_ref_path(ref_path_origin)

        if update_route_progress:
            self._update_route_progress()

        if plan_lane_swerve:
            self._update_should_lane_swerve(
                0, self.get_reference_path(), self.get_reference_path_s_start()
            )

    def _update_route_progress(self, search_until_found=False):
        if not self.route_complete():
            copy_start = 0
            for i, node in enumerate(self._route_nodes.copy()):
                try:
                    sim_to_frenet_position(
                        self.get_reference_path(), node.x,
                        node.y, self.get_reference_path_s_start()
                    )
                    search_until_found = False
                except OutsideRefPathException:
                    if not search_until_found:
                        break

                # If node is on the reference path, then it is visited
                copy_start = i + 1

            self._route_nodes = self._route_nodes[copy_start:]

    def _set_sdv_paths(self):
        self._sdv_paths = []

        route_lls = self._lanelet_route.laneletSubmap().laneletLayer
        visited_route_lls = []
        for route_ll in route_lls:
            if any(ll.id == route_ll.id for ll in visited_route_lls):
                continue

            route_lane = SDVRoute.lanelet_map.route_full_lane(self._lanelet_route, route_ll)

            next_lls = SDVRoute.lanelet_map.get_next_by_route(self._lanelet_route, route_lane[-1])
            lane_is_loop = any(next_ll.id == route_lane[0].id for next_ll in next_lls)

            prev_lane = []
            if not lane_is_loop:
                prev_lls = SDVRoute.lanelet_map.get_previous(route_lane[0])
                while len(prev_lls) > 0:
                    if any(prev_ll.id == route_lane[-1].id for prev_ll in prev_lls):
                        lane_is_loop = True
                        break
                    #Circle maps get stuck if there is a loop before the route
                    if any(ll in prev_lane for ll in prev_lls):
                        break

                    prev_ll = prev_lls[0]
                    prev_lane.append(prev_ll)
                    prev_lls = SDVRoute.lanelet_map.get_previous(prev_ll)
                prev_lane.reverse()

            next_lane = []
            if not lane_is_loop:
                next_lls = SDVRoute.lanelet_map.get_next(route_lane[-1])
                while len(next_lls) > 0:
                    if any(next_ll.id == route_lane[0].id for next_ll in next_lls):
                        lane_is_loop = True
                        break
                    if any(ll in next_lane for ll in next_lls):
                        break

                    next_ll = next_lls[0]
                    next_lane.append(next_ll)
                    next_lls = SDVRoute.lanelet_map.get_next(next_ll)

            # NOTE: full_lane = prev_lane + route_lane + next_lane
            full_lane = prev_lane
            for ll in route_lane:
                full_lane.append(ll)
                visited_route_lls.append(ll)
            full_lane += next_lane

            remaining_lls = self._lanelet_route.remainingShortestPath(route_lane[0])
            if len(remaining_lls) == 0:
                # None of route_lane is on the shortest path
                lane_swerve_start = route_lane[0].centerline[0]
                lane_swerve_end = route_lane[-1].centerline[-1]
                lane_swerve_direction = None

                right_lls = SDVRoute.lanelet_map.get_right_by_route(self._lanelet_route, route_lane[0])
                for ll in right_lls:
                    if len(self._lanelet_route.remainingShortestPath(ll)) > 0:
                        lane_swerve_direction = -1
                        break

                if lane_swerve_direction is None:
                    left_lls = SDVRoute.lanelet_map.get_left_by_route(self._lanelet_route, route_lane[0])
                    for ll in left_lls:
                        if len(self._lanelet_route.remainingShortestPath(ll)) > 0:
                            lane_swerve_direction = 1
                            break
            elif all(ll in route_lane for ll in remaining_lls):
                # The route lane is the shortest path
                lane_swerve_start = None
                lane_swerve_end = None
                lane_swerve_direction = None
            else:
                # The route lane is on the shortest path
                lane_swerve_start = None
                lane_swerve_end = route_lane[-1].centerline[-1]
                lane_swerve_direction = None

                right_lls = SDVRoute.lanelet_map.get_right_by_route(self._lanelet_route, route_lane[-1])
                left_lls = SDVRoute.lanelet_map.get_left_by_route(self._lanelet_route, route_lane[-1])
                if right_lls and (right_lls[0] in remaining_lls):
                    lane_swerve_direction = -1
                    lane_swerve_start = right_lls[0].centerline[0]
                elif left_lls and (left_lls[0] in remaining_lls):
                    lane_swerve_start = left_lls[0].centerline[0]
                    lane_swerve_direction = 1

            self._sdv_paths.append(
                SDVPath(
                    full_lane, lane_is_loop,
                    lane_swerve_start, lane_swerve_end, lane_swerve_direction
                )
            )

        if SimConfig.PLOT_VEHICLE_ROUTES:
            for sdv_path in self._sdv_paths:
                plt.figure()
                map_lines = SDVRoute.lanelet_map.get_lines()
                for xs, ys, type, subtype in data:
                    line_format = get_line_format(type, subtype)
                    if line_format is None:
                        pass
                    else:
                        color, linestyle, linewidth = line_format
                        plt.plot(xs, ys, color=color, linestyle=linestyle,
                                 linewidth=linewidth, zorder=0)

                for ll in route_lls:
                    l_lanelet_x = []
                    l_lanelet_y = []
                    for point in ll.leftBound:
                        l_lanelet_x.append(point.x)
                        l_lanelet_y.append(point.y)

                    r_lanelet_x = []
                    r_lanelet_y = []
                    for point in ll.rightBound:
                        r_lanelet_x.append(point.x)
                        r_lanelet_y.append(point.y)
                    plt.plot(l_lanelet_x, l_lanelet_y, '-', color='red')
                    plt.plot(r_lanelet_x, r_lanelet_y, '-', color='red')

                global_path_x = []
                global_path_y = []
                for point in sdv_path.get_global_path():
                    global_path_x.append(point.x)
                    global_path_y.append(point.y)
                plt.plot(global_path_x, global_path_y, '.', color='blue')

                plt.axis([-50, 40, -50, 40])

            plt.show()

    def _update_should_lane_swerve(self, s:float, path:ConstLineString3d, s_start:float):
        start, end, direction = self._current_sdv_path.get_lane_swerve_info()

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
                self._should_lane_swerve = (s <= end_s)
            elif (start_s is not None) and (end_s is None):
                # end_s is None because it lies after path
                self._should_lane_swerve = (s >= start_s)
            elif (start_s is not None) and (end_s is not None):
                # both start_s and end_s lie on path
                self._should_lane_swerve = (s >= start_s) and (s <= end_s)
            # NOTE: start_s and end_s will both be None in the case where path is
            #       the reference path; end_s is gauranteed to lie on the global path
        else:
            # NOTE: start and end will either both be None, or both be a point
            self._should_lane_swerve = False

class SDVPath(object):
    def __init__(self, lane:List[Lanelet], lane_is_loop:bool, lane_swerve_start:Point3d, lane_swerve_end:Point3d, lane_swerve_direction:int):
        # These can be used to mark a section of the global path as a "lane swerve zone"
        self._lane_swerve_start:Point3d = lane_swerve_start
        self._lane_swerve_end:Point3d = lane_swerve_end
        # NOTE: the values are the same as the ones used in LaneConfig.id; -1 for
        #       right, 1 for left, and None for no lane swerve
        self._lane_swerve_direction:int = lane_swerve_direction

        # The lanelets that make the global path
        self._lane:List[Lanelet] = lane
        self._lane_is_loop:bool = lane_is_loop

        # The global path
        # NOTE: _global_path needs to be stored as a List of Point3d because ConstLineString3d
        #       does not support slicing
        self._global_path:List[Point3d] = None
        self._global_path_i_len:int = 0
        self._global_path_s_len:float = 0.0
        self._set_global_path()

        # The reference path
        self._ref_path:ConstLineString3d = None
        # NOTE: it is assumed that this value is <= 0
        self._ref_path_s_start:float = -100.0
        # NOTE: it is assumed that this value is > 0
        self._ref_path_s_end:float = 100.0

        # Positions of of reference path points relative to the global path
        self._global_path_s_origin:float = 0.0
        self._global_path_i_start:int = 0
        self._global_path_s_start:float = 0.0
        self._global_path_i_end:int = 0
        self._global_path_s_end:float = 0.0

    def get_global_path(self):
        return ConstLineString3d(0, self._global_path)

    def get_lane(self):
        return self._lane

    def get_lane_swerve_info(self):
        return self._lane_swerve_start, self._lane_swerve_end, self._lane_swerve_direction

    def get_ref_path(self):
        return self._ref_path

    def get_ref_path_origin(self):
        return self._global_path_s_origin

    def get_ref_path_s_start(self):
        ref_path_s_start = self._global_path_s_start - self._global_path_s_origin

        if self._lane_is_loop and ref_path_s_start > 0:
            ref_path_s_start -= self._global_path_s_len

        return ref_path_s_start

    def update_ref_path(self, ref_path_origin:float):
        assert 0 <= ref_path_origin <= self._global_path_s_len

        self._global_path_s_origin = ref_path_origin

        global_path_s_start = self._global_path_s_origin + self._ref_path_s_start
        global_path_s_end = self._global_path_s_origin + self._ref_path_s_end
        if self._lane_is_loop:
            global_path_s_start %= self._global_path_s_len
            global_path_s_end %= self._global_path_s_len
        else:
            global_path_s_start = max(0.0, global_path_s_start)
            global_path_s_end = min(global_path_s_end, self._global_path_s_len)

        delta_s_start = global_path_s_start - self._global_path_s_start
        delta_s_end = global_path_s_end - self._global_path_s_end

        if self._lane_is_loop:
            if abs(delta_s_start + self._global_path_s_len) < abs(delta_s_start):
                # s_start changed from the end of the global path to the start
                delta_s_start += self._global_path_s_len
                self._global_path_i_start = 0
                self._global_path_s_start = 0
            elif abs(delta_s_start - self._global_path_s_len) < abs(delta_s_start):
                # s_start changed from the start of the global path to the end
                delta_s_start -= self._global_path_s_len
                self._global_path_i_start = self._global_path_i_len - 1
                self._global_path_s_start = self._global_path_s_len

            if abs(delta_s_end + self._global_path_s_len) < abs(delta_s_end):
                # s_end changed from the end of the global path to the start
                delta_s_end += self._global_path_s_len
                self._global_path_i_end = 0
                self._global_path_s_end = 0
            elif abs(delta_s_end - self._global_path_s_len) < abs(delta_s_end):
                # s_end changed from the start of the global path to the end
                delta_s_end -= self._global_path_s_len
                self._global_path_i_end = self._global_path_i_len - 1
                self._global_path_s_end = self._global_path_s_len

        s = self._global_path_s_start
        if delta_s_start > 0.0:
            for i in range(self._global_path_i_start, self._global_path_i_len - 1):
                ds = distance_2p(
                    self._global_path[i].x, self._global_path[i].y,
                    self._global_path[i + 1].x, self._global_path[i + 1].y
                )

                if s <= global_path_s_start < s + ds:
                    self._global_path_i_start = i
                    self._global_path_s_start = s
                    break
                elif global_path_s_start == s + ds:
                    self._global_path_i_start = i + 1
                    self._global_path_s_start = s + ds
                    break

                s += ds
        elif delta_s_start < 0.0:
            for i in reversed(range(1, self._global_path_i_start + 1)):
                ds = distance_2p(
                    self._global_path[i].x, self._global_path[i].y,
                    self._global_path[i - 1].x, self._global_path[i - 1].y
                )

                if s - ds <= global_path_s_start < s:
                    self._global_path_i_start = i - 1
                    self._global_path_s_start = s - ds
                    break
                elif global_path_s_start == s:
                    self._global_path_i_start = i
                    self._global_path_s_start = s
                    break

                s -= ds

        s = self._global_path_s_end
        if delta_s_end > 0.0:
            for i in range(self._global_path_i_end, self._global_path_i_len - 1):
                ds = distance_2p(
                    self._global_path[i].x, self._global_path[i].y,
                    self._global_path[i + 1].x, self._global_path[i + 1].y
                )

                if s < global_path_s_end <= s + ds:
                    self._global_path_i_end = i + 1
                    self._global_path_s_end = s + ds
                    break
                elif global_path_s_end == s:
                    self._global_path_i_end = i
                    self._global_path_s_end = s
                    break

                s += ds
        elif delta_s_end < 0.0:
            for i in reversed(range(1, self._global_path_i_end + 1)):
                ds = distance_2p(
                    self._global_path[i].x, self._global_path[i].y,
                    self._global_path[i - 1].x, self._global_path[i - 1].y
                )

                if s - ds < global_path_s_end <= s:
                    self._global_path_i_end = i
                    self._global_path_s_end = s
                    break
                elif global_path_s_end == s - ds:
                    self._global_path_i_end = i - 1
                    self._global_path_s_end = s - ds
                    break

                s -= ds

        # NOTE: assumes _global_path_i_start never equals _global_path_i_end
        if self._lane_is_loop and (self._global_path_i_start < self._global_path_i_end):
            # The slice lies between the ends of the global path
            self._ref_path = ConstLineString3d(
                0, self._global_path[ self._global_path_i_start : self._global_path_i_end ]
            )
        elif self._lane_is_loop and (self._global_path_i_start > self._global_path_i_end):
            # The slice wraps around the ends of the global path
            self._ref_path = ConstLineString3d(
                0, self._global_path[ self._global_path_i_start : ] +
                self._global_path[ : self._global_path_i_end ]
            )
        elif not self._lane_is_loop:
            # NOTE: it is assumed _global_path_i_start < _global_path_i_end
            #       in this case
            self._ref_path = ConstLineString3d(
                0, self._global_path[ self._global_path_i_start : self._global_path_i_end ]
            )

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

        num_points = floor(SimConfig.POINTS_PER_METER * centerline_length)
        # generate evenly spaced points along the spline
        spline = splev(np.linspace(0, 1, num=num_points), tck)

        self._global_path_s_len = 0.0
        self._global_path = [ Point3d(0, spline[0][0], spline[1][0], 0.0) ]
        for x, y in zip(spline[0][1:], spline[1][1:]):
            self._global_path_s_len += distance_2p(x, y, self._global_path[-1].x, self._global_path[-1].y)
            self._global_path.append(Point3d(0, x, y, 0.0))

        self._global_path_i_len = len(self._global_path)

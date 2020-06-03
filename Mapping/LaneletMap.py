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


class LaneletMap(object):
    def __init__(self):
        self.example_map = "/home/divit/lanelet2_standalone/lanelet2_maps/res/mapping_example.osm"
        projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
        # map is collection of primitive layers
        self.lanelet_map, errors = lanelet2.io.loadRobust(self.example_map, projector)
        assert not errors

        # nearest point test
        # testpt = BasicPoint2d(2342.69, 863)
        # lls = lanelet_map.laneletLayer.nearest(testpt, 1)

        llid = 45166
        # every layer is like a list with overlaoded [] like a map
        ll = self.lanelet_map.laneletLayer[llid]

        # testpt = BasicPoint2d(1125, 557)
        testpt = BasicPoint2d(ll.centerline[0].x, ll.centerline[0].y)
        # print(LaneletMap.get_lane_width(ll, 0))
        # print(inside(ll, testpt))
        # corresponding_ll = self.get_occupying_lanelet(testpt.x, testpt.y)
        # print(corresponding_ll)

        # TODO: put this stuff in unit tests
        # print(distance(testpt, BasicPoint2d(ll.centerline[0].x, ll.centerline[0].y)))
        # print(distance(ConstLineString2d(ll.centerline), Point2d(getId(), testpt.x, testpt.y)))
        # print(testpt)
        # s, d = LaneletMap.sim_to_frenet_frame(ll, testpt.x, testpt.y)
        # print((s, d))
        # x, y = LaneletMap.frenet_to_sim_frame(ll, s, d)
        # print((x, y))

        # plot the lanelet points for sanity testing
        # plt.plot(testpt.x, testpt.y, 'bo')
        # LaneletMap.plot_ll(ll)
        # plt.show()

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

    def get_global_path_for_route(self, x, y, lanelet_route, meters_ahead=100):
        """ This looks 100m ahead of the beginning of the current lanelet. Change?
            x, y only used to determine the starting lanelet, allowed to be a little outdated
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
    def get_lane_width(lanelet, s):
        """ Two ways to do this: project the point onto leftbound and rightbound and add their distances
            OR intersect the centerline normal at s with leftbound and rightbound.
            Either way the width would be discontinuous as you move along s.
            TODO: do we actually need average/min lane width over some lookahead time?
            @param s:   length along the lanelet centerline
        """
        x, y = LaneletMap.sim_to_frenet_frame(lanelet.centerline, s, 0)
        point_on_centerline = BasicPoint2d(x, y)
        # project on left and right bounds
        point_on_leftbound = project(ConstLineString2d(lanelet.leftBound), point_on_centerline)
        point_on_rightbound = project(ConstLineString2d(lanelet.rightBound), point_on_centerline)
        return distance(point_on_leftbound, point_on_rightbound)
    
    
    @staticmethod
    def sim_to_frenet_frame(ref_path, x, y):
        """ TODO: need to transform heading?
            @param ref_path:    ConstLineString3d. Change to something general? Enforce types in python?
        """
        assert isinstance(ref_path, ConstLineString3d)

        path_ls = ConstLineString2d(ref_path)
        # toArcCoordinates does not interpolate beyond or before ref_path
        # so when it goes over, it's time to switch to a new ref path?
        arc_coords = toArcCoordinates(path_ls, BasicPoint2d(x, y))
        return arc_coords.length, arc_coords.distance


    @staticmethod
    def frenet_to_sim_frame(ref_path, s, d):
        """
            @param ref_path:    iterable of lanelet2.core.Point3d. Change to something general?
        """
        point_on_ls = None
        tangent = None
        arclen = 0
        for p, q in pairwise(ref_path):
            pq = np.array([q.x - p.x, q.y - p.y])
            dist = np.linalg.norm(pq)

            if arclen <= s <= arclen + dist:
                delta_s = s - arclen
                tangent = normalize(pq)
                point_on_ls = np.array([p.x, p.y]) + tangent * delta_s
                break
            
            arclen += dist
            # print((p.x, p.y, q.x, q.y))
        
        if point_on_ls is None:
            print("s is outside the lanelet")
            return None, None
        
        normal = np.array([-1 * tangent[1], tangent[0]])
        point_in_cart = point_on_ls + normal*d
        return point_in_cart[0], point_in_cart[1]

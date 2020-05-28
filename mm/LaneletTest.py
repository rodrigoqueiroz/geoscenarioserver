import lanelet2
from lanelet2.core import AttributeMap, getId, BasicPoint2d, Point3d, Point2d, BoundingBox2d, ConstPoint2d, LineString3d, ConstLineString2d, ConstLineString3d, Lanelet, RegulatoryElement, TrafficLight, LaneletMap, createMapFromLanelets
from lanelet2.geometry import distance, intersects2d, boundingBox2d, to2D, inside, toArcCoordinates, project
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


class LaneletTest(object):
    def __init__(self):
        self.example_map = "/home/divit/lanelet2_standalone/lanelet2_maps/res/mapping_example.osm"
        projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
        # map is collection of primitive layers
        self.lanelet_map, errors = lanelet2.io.loadRobust(self.example_map, projector)
        assert not errors

        # nearest point test
        # testpt = BasicPoint2d(2342.69, 863)
        # lls = lanelet_map.laneletLayer.nearest(testpt, 1)
        # nearestll = lls[0]

        # print(nearestll.centerline)
        # for pt in nearestll.rightBound:
        #     print(pt)
        
        llid = 45166
        # every layer is like a list with overlaoded [] like a map
        ll = self.lanelet_map.laneletLayer[llid]

        # testpt = BasicPoint2d(1125, 557)
        # testpt = BasicPoint2d(10, 557)
        testpt = BasicPoint2d(ll.centerline[0].x, ll.centerline[0].y)
        # print(LaneletTest.get_lane_width(ll, 0))
        # print(inside(ll, testpt))
        # corresponding_ll = self.get_occupying_lanelet(testpt.x, testpt.y)
        # print(corresponding_ll)

        # print(distance(testpt, BasicPoint2d(ll.centerline[0].x, ll.centerline[0].y)))
        # print(distance(ConstLineString2d(ll.centerline), Point2d(getId(), testpt.x, testpt.y)))
        # print(testpt)
        # s, d = LaneletTest.sim_to_frenet_frame(ll, testpt.x, testpt.y)
        # print((s, d))
        # x, y = LaneletTest.frenet_to_sim_frame(ll, s, d)
        # print((x, y))


        # plot the lanelet points for sanity testing
        # plt.plot(testpt.x, testpt.y, 'bo')
        # LaneletTest.plot_ll(ll)
        # plt.show()


    def get_occupying_lanelet(self, x, y):
        point = BasicPoint2d(x, y)
        
        # get all intersecting lanelets using a trivial bounding box
        searchbox = BoundingBox2d(point, point)
        intersecting_lls = self.lanelet_map.laneletLayer.search(searchbox)
        # commenting cause this fn doesn't work yet
        # if len(intersecting_lls) == 0:
        #     return None
        
        # intersecting_lls = list(filter(lambda ll: inside(ll, point), intersecting_lls))
        # print([ll.id for ll in intersecting_lls])

        # use routing or some other information to determine which lanelet we are in
        

        return self.lanelet_map.laneletLayer[45166]


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
        x, y = LaneletTest.sim_to_frenet_frame(lanelet, s, 0)
        point_on_centerline = BasicPoint2d(x, y)
        # project on left and right bounds
        point_on_leftbound = project(ConstLineString2d(lanelet.leftBound), point_on_centerline)
        point_on_rightbound = project(ConstLineString2d(lanelet.rightBound), point_on_centerline)
        return distance(point_on_leftbound, point_on_rightbound)
    
    
    @staticmethod
    def sim_to_frenet_frame(lanelet, x, y):
        """ TODO: need to transform heading?
        """
        # ref path can be something else once this fn is moved out
        ref_path = lanelet.centerline

        # toArcCoordinates does not interpolate beyond or before ref_path
        # so when it goes over, it's time to switch to a new ref path?
        arc_coords = toArcCoordinates(ConstLineString2d(ref_path), BasicPoint2d(x, y))
        return arc_coords.length, arc_coords.distance


    @staticmethod
    def frenet_to_sim_frame(lanelet, s, d):
        ref_path = lanelet.centerline
        
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



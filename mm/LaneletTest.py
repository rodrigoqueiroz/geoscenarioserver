import lanelet2
from lanelet2.core import AttributeMap, getId, BasicPoint2d, Point3d, Point2d, BoundingBox2d, ConstPoint2d, LineString3d, ConstLineString2d, ConstLineString3d, Lanelet, RegulatoryElement, TrafficLight, LaneletMap, createMapFromLanelets
from lanelet2.geometry import distance, intersects2d, boundingBox2d, to2D, inside, toArcCoordinates
from lanelet2.projection import UtmProjector

from matplotlib import pyplot as plt
from itertools import tee


def get_test_ll():
    left = LineString3d(getId(), [Point3d(getId(), 1, y, 0) for y in range(0, 5)])
    right = LineString3d(getId(), [Point3d(getId(), 2, y, 0) for y in range(0, 5)])
    return Lanelet(getId(), left, right)

def pairwise(iterable):
    i, j = tee(iterable, 2)
    next(j, None)
    return zip(i, j)



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
        
        llid = 44992
        # every layer is like a list with overlaoded [] like a map
        ll = self.lanelet_map.laneletLayer[llid]

        # testpt = BasicPoint2d(1125, 557)
        # testpt = BasicPoint2d(10, 557)
        testpt = BasicPoint2d(ll.leftBound[0].x, ll.leftBound[0].y)
        # print(inside(ll, testpt))
        # corresponding_ll = self.get_occupying_lanelet(testpt.x, testpt.y)
        # print(corresponding_ll)

        # print(distance(testpt, BasicPoint2d(ll.centerline[0].x, ll.centerline[0].y)))
        # print(distance(ConstLineString2d(ll.centerline), Point2d(getId(), testpt.x, testpt.y)))
        print(LaneletTest.sim_to_frenet_frame(ll, testpt))

        LaneletTest.frenet_to_sim_frame(ll, 0.2465, 1.53)

        # plot the lanelet points for sanity testing
        LaneletTest.plot_ll(ll)
        plt.plot(testpt.x, testpt.y, 'bo')
        plt.show()
    

    def get_occupying_lanelet(self, x, y):
        point = BasicPoint2d(x, y)
        
        # get all intersecting lanelets using a trivial bounding box
        searchbox = BoundingBox2d(point, point)
        intersecting_lls = self.lanelet_map.laneletLayer.search(searchbox)
        if len(intersecting_lls) == 0:
            return None
        
        intersecting_lls = list(filter(lambda ll: inside(ll, point), intersecting_lls))
        print([ll.id for ll in intersecting_lls])

        # use routing or some other information to determine which lanelet we are in
        return intersecting_lls[-2]


    @staticmethod
    def plot_ll(lanelet):
        xs = [pt.x for pt in lanelet.rightBound] + [pt.x for pt in lanelet.leftBound]
        ys = [pt.y for pt in lanelet.rightBound] + [pt.y for pt in lanelet.leftBound]
        
        plt.plot(xs, ys, 'ro')
        plt.plot(
            [pt.x for pt in lanelet.centerline],
            [pt.y for pt in lanelet.centerline],
            'go')
    
    
    @staticmethod
    def sim_to_frenet_frame(lanelet, pt):
        """ pt is a BasicPoint2d, change that later
        """
        # ref path can be something else once this fn is moved out
        ref_path = lanelet.centerline

        # toArcCoordinates does not interpolate beyond or before ref_path
        # so when it goes over, it's time to switch to a new ref path?
        arc_coords = toArcCoordinates(ConstLineString2d(ref_path), pt)
        return arc_coords.length, arc_coords.distance
    
    @staticmethod
    def frenet_to_sim_frame(lanelet, s, d):
        ref_path = lanelet.centerline
        
        for p, q in pairwise(ref_path):
            print((p.x,q.x))


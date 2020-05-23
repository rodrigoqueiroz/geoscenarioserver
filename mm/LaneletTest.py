import lanelet2
from lanelet2.core import AttributeMap, getId, BasicPoint2d, Point3d, LineString3d, Lanelet, RegulatoryElement, TrafficLight, LaneletMap, createMapFromLanelets
from lanelet2.geometry import distance, intersects2d, boundingBox2d, to2D
from lanelet2.projection import UtmProjector

class LaneletTest(object):
    def __init__(self):
        self.example_map = "/home/divit/lanelet2_standalone/lanelet2_maps/res/mapping_example.osm"
        projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
        # map is collection of primitive layers
        lanelet_map, errors = lanelet2.io.loadRobust(self.example_map, projector)
        assert not errors

        testpt = BasicPoint2d(2342.69, 863)
        lls = lanelet_map.laneletLayer.nearest(testpt, 1)
        nearestll = lls[0]

        print(nearestll.centerline)
        for pt in nearestll.rightBound:
            print(pt)
        # every layer is like a list with overlaoded []
        # for pt in lanelet_map.pointLayer:
            # print(pt)

#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------

import xml.etree.ElementTree
import re
import gsc.Utils as Utils
from gsc.Report import Report

# do we want the projection dependency here?
from lanelet2.core import GPSPoint


class Node(object):
    def __init__(self):
        self.id = None  # id from josm
        self.lat = None
        self.lon = None
        self.x = None
        self.y = None
        self.tags = {}


class Way(object):
    def __init__(self):
        self.id = None
        self.nodes = []
        self.tags = {}


class GSParser(object):

    def __init__(self):
        self.nodes = {}
        self.ways = []
        self.filename = ''

        self.pedestrians = {}
        self.vehicles = {}
        self.staticobjects = {}
        self.tlights = {}
        self.origin = None
        self.globalconfig = None
        self.egostart = None
        self.egogoals = {}
        self.subscenariogoals = {}
        self.metrics = {}
        self.locations = {}
        self.triggers = {}
        self.paths = {}
        self.routes = {}

        self.report = Report()

    def load_geoscenario_file(self, filepath):
        xml_root = xml.etree.ElementTree.parse(filepath).getroot()
        for osm_node in xml_root.findall('node'):
            node = Node()
            node.id = int(osm_node.get('id'))
            node.lat = float(osm_node.get('lat'))
            node.lon = float(osm_node.get('lon'))
            self.parse_tags(osm_node, node)
            self.nodes[node.id] = node
        for osm_node in xml_root.findall('way'):
            way = Way()
            way.id = int(osm_node.get('id'))
            self.parse_tags(osm_node, way)
            for osm_nd in osm_node.findall('nd'):
                way.nodes.append(self.nodes[ int(osm_nd.get('ref')) ])
            self.ways.append(way)

    def load_and_validate_geoscenario(self, filepath):
        #Load XML
        self.load_geoscenario_file(filepath)
        self.isValid = True
        self.report.file = filepath
        self.filename = filepath
        #Process Nodes
        for nid, node in self.nodes.items():
            if "gs" in node.tags:
                #physical elements
                if node.tags["gs"] == "staticobject": self.check_static_object(node)  #also way / closedway
                elif node.tags["gs"] == "pedestrian": self.check_pedestrian(node)
                elif node.tags["gs"] == "vehicle": self.check_vehicle(node)
                elif node.tags["gs"] == "trafficlight": self.check_traffic_light(node)
                #logic elements
                elif node.tags["gs"] == "origin": self.check_origin(node)
                elif node.tags["gs"] == "egostart": self.check_ego_start(node)
                elif node.tags["gs"] == "egogoal": self.check_ego_goal(node)
                elif node.tags["gs"] == "location": self.check_location(node) #also way / closedway
                elif node.tags["gs"] == "metric": self.check_metric(node)
                elif node.tags["gs"] == "globalconfig": self.check_global_config(node)
                elif node.tags["gs"] == "trigger": self.check_trigger(node)
        #Process Ways
        for way in self.ways:
            if "gs" in way.tags:
                if way.tags["gs"] == "staticobject": self.check_static_object(way)
                elif way.tags["gs"] == "location": self.check_location(way)
                elif way.tags["gs"] == "path": self.check_path(way)
                elif way.tags["gs"] == "route": self.check_route(way)

        #Return result
        return self.isValid

    def parse_tags(self, osm_node, node):
        for osm_tag in osm_node.findall('tag'):
            v = osm_tag.get('v')
            node.tags[osm_tag.get('k')] = float(v) if Utils.is_number(v) else v
            # if v == 'origin':
            #     print(osm_tag.get('k'))
            #     print(node.tags['gs'])

    def project_nodes(self, projector):
        assert len(self.nodes) > 0

        for node_id, node in self.nodes.items():
            # NOTE: no altitude information
            cart_pt = projector.forward(GPSPoint(node.lat, node.lon, 0.0))
            node.x = cart_pt.x
            node.y = cart_pt.y

    def check_static_object(self, n):  # node /  way / area
        mandatory = {"gs","name","area"}
        optional = {"model","height","group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.staticobjects[n.tags["name"]] = n

    def check_pedestrian(self, n):
        mandatory = {"gs","name"}
        optional = {"orientation","speed","path","cycles","usespeedprofile","start","group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.pedestrians[n.tags["name"]] = n

    def check_vehicle(self, n):
        mandatory = {"gs","vid","name"}
        optional = {
            "orientation","speed","path","cycles",
            "usespeedprofile","start","group", "simid",
            "btree", "route"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        if ("speed" in n.tags):
            speed = n.tags['speed']
            if not Utils.is_positive_number(speed):
                speed_range = Utils.get_number_range(speed)
                if speed_range is not None:
                    if not Utils.is_valid_number_range(speed_range[0],speed_range[1],0,100):
                        self.report.log_error("Invalid speed range")
                else:
                    speed_list = Utils.get_numberlist(speed)
                    if speed_list is not None:
                        if not Utils.is_valid_numberlist(speed_list,0,100):
                            self.report.log_error("Element "+n.id +". Invalid speed list " + str(speed_list))
                    else:
                        self.report.log_error("Invalid speed value")

        self.vehicles[n.tags["vid"]] = n

    def check_traffic_light(self, n):
        mandatory = {"gs","name","states"}
        optional = {"duration","group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.tlights[n.tags["name"]] = n
         #assert duration match states

    def check_path(self, n:Way): #:Way
        mandatory = {"gs","name"}
        optional = {"abstract"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)
        #todo: check nd and their ids
        assert len(n.nodes) > 0
        self.paths[n.tags['name']] = n
        return None

    def check_route(self, n:Way): #:Way
        mandatory = {"gs","name"}
        optional = {}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)
        #todo: check nd and their ids
        assert len(n.nodes) > 0
        self.routes[n.tags['name']] = n
        return None

    #== Logical

    def check_origin(self,n):
        mandatory = {"gs"}
        optional = {}
        self.check_tags(n, mandatory, optional)
        if self.origin is not None:
            self.report.log_error( "Element " + n.id + ": Duplicate origin node. Must be unique in a scenario")
        self.origin = n

    def check_metric(self,n):
        mandatory = {"gs","name"}
        optional = {"reference","agents", "group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.metrics[n.tags["name"]] = n

    def check_ego_start(self, n):
        mandatory = {"gs"}
        optional = {"orientation"}
        self.check_tags(n,mandatory, optional)
        if self.origin is not None:
            self.report.log_error( "Element " + n.id + ": Duplicate Egostart node. Must be unique")
        self.egostart = n;

    def check_ego_goal(self, n):
        mandatory = {"gs","name"}
        optional = {"order"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.egogoals[n.tags["name"]] = n
        #todo #assertorder
        #for goal in egogoals:

    def check_location(self, n): #also way / closedway
        mandatory = {"gs","name","area"}
        optional = {"continuous","group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.locations[n.tags["name"]] = n

    def check_global_config(self, n):
        mandatory = {"gs","version","name","lanelet","timeout",}
        optional = {"collision","notes","metric","mutate","optimize","optmetric","plotvid",}
        self.check_tags(n, mandatory, optional)
        if self.globalconfig is not None:
            self.report.log_error( "Element " + n.id + ": Duplicate Global Config node. Must be unique")
        self.globalconfig = n

    def check_trigger(self, n):
        mandatory = {"gs","name","activate"}
        optional = {"radius","time","metric","value","owner","target","aspeed" ,
        "aspeedprofile","alocation","apath","astate" ,"astart","afail","asuccess","delay","group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.triggers[n.tags["name"]] = n

        #assert owner exist
        #assert target exist
        #assert target can take action


   #== Aux


    def check_tags(self,n, mandatory_set, optional_set):
        expected_set = set.union(mandatory_set,optional_set)
        tags_list = n.tags.keys()
        actual_set = set(tags_list)

        #duplicates
        if ( len(tags_list) != len(actual_set)):
            self.report.log_error("Element {} contains duplicate tags: {}".format(
                n.id, tags_list
            ))

        #mandatory tags
        if not mandatory_set.issubset(actual_set):
            self.report.log_error("Element {} missing mandatory tags: {}".format(
                n.id, mandatory_set - actual_set
            ))

        #all tags
        if not actual_set.issubset(expected_set):
            self.report.log_warning("Element {} has unknown tags: {}".format(
                n.id, actual_set - expected_set
            ))

    def check_nd_ref(self,n):
       #todo: check nd attributes, and  if nodes exist
       return None


    def check_uniquename(self,n):
        name = n.tags['name']

        self.is_unique_name(n,self.staticobjects)
        self.is_unique_name(n,self.vehicles)
        self.is_unique_name(n,self.pedestrians)
        self.is_unique_name(n,self.tlights)
        self.is_unique_name(n,self.locations)
        self.is_unique_name(n,self.triggers)

    def is_unique_name(self,n,elements):
        name = n.tags['name']
        if name in elements:
            self.report.log_error("Element "+n.id+" name conflicts with "+ elements[name].id + " : " + name)
            return False
        else:
            return True

    def print_stats(self):
        print(Report.bcolors.HEADER+"#### GeoScenario Summary #### "+Report.bcolors.ENDC)
        print("Pedestrians: " + str(len(self.pedestrians)))
        print("Vehicles: " + str(len(self.vehicles)))
        print("Static Objects: " + str(len(self.staticobjects)))
        print("Triggers: " + str(len(self.triggers)))
        print("Paths: " + str(len(self.paths)))

    def print_scenario(self):
        print(Report.bcolors.HEADER+"#### GeoScenario Detailed #### "+Report.bcolors.ENDC)
        print("GeoScenario: " + str(self.filename))
        for x in self.pedestrians:
            print (x,':', self.pedestrians[x])
            for y in self.pedestrians[x].tags:
                print (y, ':', self.pedestrians[x].tags[y])



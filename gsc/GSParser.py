#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------

import xml.etree.ElementTree
import re
import gsc.Utils as Utils
from gsc.Report import Report
from SimConfig import UNIQUE_GS_TAGS_PER_SCENARIO

# do we want the projection dependency here?
from lanelet2.core import GPSPoint

import logging
log = logging.getLogger(__name__)

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
        self.trajectories = {}

        self.report = Report()

    def load_geoscenario_file(self, filepaths):
        #for file_num,filepath in enumerate(filepaths):
        agent_ids = {'pedestrian': [], 'vehicle': []}

        for file_num,filepath in enumerate(filepaths):
            xml_root = xml.etree.ElementTree.parse(filepath).getroot()
            for osm_node in xml_root.findall('node'):
                node = Node()
                node.id = int(osm_node.get('id'))
                node.lat = float(osm_node.get('lat'))
                node.lon = float(osm_node.get('lon'))
                if not self.parse_tags(osm_node, node, agent_ids):
                    return False
                if file_num == 0 or 'gs' not in node.tags:
                    self.nodes[node.id] = node
                else:
                    # only add gs tags if the value is not in the list of
                    # unique tags per scenario (e.g. origin, globalconfig)
                    if (node.tags['gs'] not in UNIQUE_GS_TAGS_PER_SCENARIO):
                        self.nodes[node.id] = node

            # there are currently no way tags that must be unique per scenario
            for osm_node in xml_root.findall('way'):
                way = Way()
                way.id = int(osm_node.get('id'))
                self.parse_tags(osm_node, way, agent_ids)
                for osm_nd in osm_node.findall('nd'):
                    way.nodes.append(self.nodes[ int(osm_nd.get('ref')) ])
                self.ways.append(way)

        # assign auto-generated ids
        gs_nodes = [node for node in self.nodes.values() if 'gs' in node.tags]
        nvehicles = len([node for node in gs_nodes if node.tags['gs'] == 'vehicle'])
        npedestrians = len([node for node in gs_nodes if node.tags['gs'] == 'pedestrian'])

        vehicle_auto_ids = iter([i for i in range(1, nvehicles + 1) if i not in agent_ids['vehicle']])
        ped_auto_ids = iter([i for i in range(1, npedestrians + 1) if i not in agent_ids['pedestrian']])

        for node in gs_nodes:
            # check if node does not have a pid or vid
            if not any([k in ['vid', 'pid'] for k in node.tags]):
                if node.tags['gs'] == 'vehicle':
                    node.tags['vid'] = next(vehicle_auto_ids)
                elif node.tags['gs'] == 'pedestrian':
                    node.tags['pid'] = next(ped_auto_ids)

        return True


    def load_and_validate_geoscenario(self, filepaths):
        #Load XML
        if not self.load_geoscenario_file(filepaths):
            return False
        self.isValid = True
        self.report.file = filepaths[0]
        self.filename = filepaths[0]
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
                elif way.tags["gs"] == "trajectory": self.check_trajectory(way)

        #Return result
        return self.isValid

    def parse_tags(self, osm_node, node, agent_ids):
        for osm_tag in osm_node.findall('tag'):
            k = osm_tag.get('k')
            v = osm_tag.get('v')

            node.tags[k] = float(v) if Utils.is_number(v) else v

        #ignore if not a gs element
        if not ('gs' in node.tags):
            return True

        # if node has a pid or vid, add it to appropriate array in agent_ids
        if any([k in ['vid', 'pid'] for k in node.tags]):
            id_type = 'vid' if 'vid' in node.tags else 'pid'
            assigned_id = int(node.tags[id_type])

            if assigned_id in agent_ids[node.tags['gs']]:
                log.error("Conflicting {}={} in the loaded scenario files.".format(id_type, assigned_id))
                log.error("Please assign a different {} for one of the agents.".format(id_type))
                return False

            agent_ids[node.tags['gs']].append(assigned_id)

        return True

    def project_nodes(self, projector, altitude):
        assert len(self.nodes) > 0

        for node_id, node in self.nodes.items():
            cart_pt = projector.forward(GPSPoint(node.lat, node.lon, altitude))
            node.x = cart_pt.x
            node.y = cart_pt.y

    def check_static_object(self, n):  # node /  way / area
        mandatory = {"gs","name","area"}
        optional = {"model","height","group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.staticobjects[n.tags["name"]] = n

    def check_pedestrian(self, n):
        mandatory = {"gs","pid","name"}
        optional = {"yaw","model","btype","trajectory", "esource", "eid",
                    "speed","path","cycles","usespeedprofile","start","group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.pedestrians[n.tags["pid"]] = n

    def check_vehicle(self, n):
        mandatory = {"gs","vid","name"}
        optional = { "yaw","model","btype","trajectory","route","btree", "esource", "eid",
                    "speed","path","cycles","usespeedprofile","start","group",}
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
        vid = int(n.tags["vid"])
        self.vehicles[vid] = n

    def check_traffic_light(self, n):
        mandatory = {"gs","name","states",}
        optional = {"group","duration","interval"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.tlights[n.tags['name']] = n

        # ensure no. states match no. durations
        nstates = len(n.tags["states"].split(','))
        ndurations = len(n.tags["duration"].split(',')) if isinstance(n.tags["duration"], str) else 1
        assert nstates == ndurations

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

    def check_trajectory(self, n:Way): #:Way
        mandatory = {"gs","name"}
        optional = {}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)
        #todo: check nd and their ids
        assert len(n.nodes) > 0
        self.trajectories[n.tags['name']] = n
        return None

    #== Logical

    def check_origin(self,n):
        mandatory = {"gs"}
        optional = {}
        self.check_tags(n, mandatory, optional)
        if self.origin is not None:
            self.report.log_error("Element {}: Duplicate origin node. Must be unique in a scenario".format(n.id))
        self.origin = n

    def check_metric(self,n):
        mandatory = {"gs","name"}
        optional = {"reference","agents", "group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

        self.metrics[n.tags["name"]] = n

    def check_ego_start(self, n):
        mandatory = {"gs"}
        optional = {"yaw"}
        self.check_tags(n, mandatory, optional)
        if self.origin is not None:
            self.report.log_error("Element {}: Duplicate Egostart node. Must be unique".format(n.id))
        self.egostart = n

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
            self.report.log_error("Element " + n.id + ": Duplicate Global Config node. Must be unique")
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
        log.info(f"Pedestrians: {len(self.pedestrians)}")
        log.info(f"Vehicles: {len(self.vehicles)}")
        log.info(f"Static Objects: {len(self.staticobjects)}")
        log.info(f"Triggers: {len(self.triggers)}")
        log.info(f"Paths: {len(self.paths)}")

    def print_scenario(self):
        print(Report.bcolors.HEADER+"#### GeoScenario Detailed #### "+Report.bcolors.ENDC)
        print("GeoScenario: " + str(self.filename))
        for x in self.pedestrians:
            print (x,':', self.pedestrians[x])
            for y in self.pedestrians[x].tags:
                print (y, ':', self.pedestrians[x].tags[y])
        for x in self.vehicles:
            print (x,':', self.vehicles[x])
            for y in self.vehicles[x].tags:
                print (y, ':', self.vehicles[x].tags[y])

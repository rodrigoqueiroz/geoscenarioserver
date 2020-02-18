__author__ = "Rodrigo Queiroz"
__email__ = "rqueiroz@gsd.uwaterloo.ca"

import xml.etree.ElementTree
import re
import Utils
import Report

class GSChecker(object):

    def __init__(self):
        self.nodes = []
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

        self.report = Report.Report() 

    def load_geoscenario_file(self, filepath):
        xml_root = xml.etree.ElementTree.parse(filepath).getroot()
        for osm_node in xml_root.findall('node'):
            node = Node()
            node.id = osm_node.get('id')
            node.lat = osm_node.get('lat')
            node.lon = osm_node.get('lon')
            for osm_tag in osm_node.findall('tag'):
                node.tags[osm_tag.get('k')] =  osm_tag.get('v')
            self.nodes.append(node)
        for osm_node in xml_root.findall('way'):
            way = Way()
            way.id = osm_node.get('id')
            for osm_tag in osm_node.findall('tag'):
                way.tags[osm_tag.get('k')] =  osm_tag.get('v')
            self.ways.append(way)

    def validate_geoscenario(self, filepath):
        #Load XML
        self.load_geoscenario_file(filepath);
        self.isValid = True
        self.report.file = filepath
        self.filename = filepath
        #Process Nodes
        for node in self.nodes:
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
        mandatory = {"gs","name"}
        optional = {"orientation","speed","path","cycles","usespeedprofile","start","group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)

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

        
        self.vehicles[n.tags["name"]] = n

    def check_traffic_light(self, n):
        mandatory = {"gs","name","states"}
        optional = {"duration","group"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)
        
        self.tlights[n.tags["name"]] = n
         #assert duration match states

    def check_path(self, n): #:Way
        mandatory = {"gs","name"}
        optional = {"abstract"}
        self.check_tags(n, mandatory, optional)
        self.check_uniquename(n)
        #todo: check nd and their ids
        self.paths[n.tags['name']] = n;
        return None

    #== Logical

    def check_origin(self,n):
        mandatory = {"gs"}
        optional = {}
        self.check_tags(n, mandatory, optional)
        if self.origin is not None:
            self.report.log_error( "Element " + n.id + ": Duplicate origin node. Must be unique in a scenario")
        self.origin = n;

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
        mandatory = {"gs","version","name","lanelet","collision","timeout"}
        optional = {"notes","metric","mutate","optimize","optmetric"}
        self.check_tags(n, mandatory, optional)
        if self.globalconfig is not None:
            self.report.log_error( "Element " + n.id + ": Duplicate Global Config node. Must be unique")

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
            self.report.log_error( "Element " + n.id 
                + " contains duplicate tags: " + str(tags_list))

        #mandatory tags
        if not mandatory_set.issubset(actual_set):
            self.report.log_error( "Element " + n.id 
                + " missing mandatory tags: " + str(mandatory_set - actual_set))

        #all tags
        if not actual_set.issubset(expected_set):
            self.report.log_warning("Element " + n.id 
            + " has unknown tags: " + str(actual_set - expected_set))

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

class Node(object):
    def __init__(self):
        self.id = None
        self.lat = None
        self.lon = None
        self.tags = {}

class Way(object):
    def __init__(self):
        self.id = None
        self.nodes = []
        self.tags = {}

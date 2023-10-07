#!/usr/bin/env python3
# jinwei.zhang@uwaterloo.ca
# ---------------------------------------------

'''
Build a scenario file constructor given the map and some inputs
Task breakdown:
Constrcut basic primitives
- [✓] node
- [✓] way
Construct basic Configuration elements
- [✓] Global Configuration
- [✓] Origin
- [✓] Lanelet Map Directory
- [ ] Traffic Light
- [✓] Vehicle configuration
 - [✓] Initial state
 - [✓] Route injection
- [✓] Planning model
- [ ] misbehavior injection strategy
'''

import os
from logging import error
from xml.dom import minidom
from lanelet2.core import getId, GPSPoint, BasicPoint3d
from lanelet2.projection import UtmProjector
from lanelet2.io import Origin
from xml.dom import minidom
from typing import Any, List

class GSWriter():
    
    def __init__(self, auto_init: bool=True, auto_origin: bool=True) -> None:
        self.root = None
        self.osm = None
        self.nodes = []             # list of node ids of node primitives
        self.ways = []              # list of node ids of ways (vehicle routes)
        self.vehicles = []          # list of node ids of vehicles
        self.reference_dict = {}    # Data type: dict{[way_id: int]: list(waypoint_node_id: int)}
        self.origin = None          # Data type of origin: list[lat: float, lon: float, alt: float]
                                    ## Note that the origin is saved as WGS84(lat/lon/ele) format, 
                                    ## which is consistent with for format definition of .osm file
        self.projector = None       # Data type of projector: lanelet2.projection.UtmProjector
        if auto_init:
            self.initHeader()
        if auto_origin:
            self.addOrigin()
        
    def getId(self) -> int:
        return -1*getId()
    
    def __python2xml__(self, input: Any) -> str:
        if type(input) == bool:
            return 'yes' if input else 'no'
        else:
            TypeError("Unsupported type for python2mxl syntac transformer.")
    
    def addProjector(self):
        try:
            if type(self.origin) == list:
                self.projector = UtmProjector(Origin(self.origin[0], self.origin[1], self.origin[2]))
            else:
                raise Exception
        except Exception:
            error('EmptyObjectError: Cannot construct projector before specifying origin attribute.')
            return None
        
    
    # convert WGS84 (latitide/longitude) to UTM (meters)
    def ll2m(self, lat: float, lon: float, ele: None or float=None) -> List[float] or None:
        try:
            if type(self.projector) == UtmProjector:
                if not ele: # set the default latitude as origin.
                    ele = self.origin[2]
                utm_point = self.projector.forward(GPSPoint(lat, lon, ele))
                return [utm_point.x, utm_point.y, utm_point.z]
            else:
                raise Exception
        except Exception:
            error('EmptyObjectError: Cannot do projection before defining projector attribute as type of lanelet2 utmProjector.')
            return None
    
    # convert UTM (meters) to WGS84 (latitide/longitude)
    def m2ll(self, 
             x: float or List[float], 
             y: float or List[float],
             z: float or List[float] or None=None) -> List[float] or List[List[float]] or None:
        try:
            if type(self.projector) == UtmProjector:
                # set the default latitude as origin.
                if not z:
                    # Note that the latitude in WGS84 and height in UTM are the same unit 
                    # in meters, therefore they are interchangable
                    z = self.origin[2]
                if type(x) == list:
                    assert len(x) == len(y)
                    # TODO: take care of the case when z is a list
                    lon_arr = []
                    lat_arr = []
                    for x_point, y_point in zip(x, y):
                        gps_point = self.projector.reverse(BasicPoint3d(x_point, y_point, z))
                        lon_arr.append(gps_point.lon)
                        lat_arr.append(gps_point.lat)
                    return [lat_arr, lon_arr, z]
                elif type(x) == float:
                    gps_point = self.projector.reverse(BasicPoint3d(x, y, z))
                    return [gps_point.lat, gps_point.lon, gps_point.alt]
            else:
                raise Exception
        except Exception:
            error('EmptyObjectError: Cannot do projection before defining projector attribute as type of lanelet2 utmProjector.')
            return None
    
    def initHeader(self) -> None:
        self.root = minidom.Document()
        self.osm = self.root.createElement('osm') 
        self.osm.setAttribute('version', '0.6')
        self.osm.setAttribute('generator', 'GSWriter')
        self.root.appendChild(self.osm)

    def writeOSM(self, file_name: str, file_dir: str=os.getcwd()) -> None:
        assert type(file_name) is str
        assert type(file_dir) is str
        osm_str = self.root.toprettyxml(indent ="\t")
        save_path_file = os.path.join(file_dir, file_name)
        with open(save_path_file, "w") as f:
            f.write(osm_str)
            f.close()
        
    def addGlobalConfig(self, lat: float, lon: float, scenario_name: str, lanelet_map_dir: str, collision: bool, timeout: float) -> None:
        nid = self.getId()
        n = self.getNode(lat, lon, nid)
        t_globalconfig = self.getTag('gs', 'globalconfig')
        self.attachTagToNodeTree(t_globalconfig, n)
        t_name = self.getTag('name', str(scenario_name))
        self.attachTagToNodeTree(t_name, n)
        t_lanelet = self.getTag('lanelet', str(lanelet_map_dir))
        self.attachTagToNodeTree(t_lanelet, n)
        t_collision = self.getTag('collision', self.__python2xml__(collision))
        self.attachTagToNodeTree(t_collision, n)
        t_mutate = self.getTag('mutate', 'no')
        self.attachTagToNodeTree(t_mutate, n)
        t_timeout = self.getTag('timeout', str(timeout))
        self.attachTagToNodeTree(t_timeout, n)
        t_version = self.getTag('version', '2.0')
        self.attachTagToNodeTree(t_version, n)
        
        # add node id to object
        self.nodes.append(nid)
        
        # add global config to osm object
        self.addNodeToTree(n)
    
    def addOrigin(self, lat: float=0.0, lon: float=0.0, altitude: float=300, area: float=100) -> None:
        # sync the origin definition to the object attributes
        self.origin = [lat, lon, altitude]
        self.addProjector()
        # construct the origin node
        nid = self.getId()
        n = self.getNode(lat, lon, nid)
        t_altitude = self.getTag('altitude', str(altitude))
        self.attachTagToNodeTree(t_altitude, n)
        t_area = self.getTag('area', str(area))
        self.attachTagToNodeTree(t_area, n)
        t_gs = self.getTag('gs', 'origin')
        self.attachTagToNodeTree(t_gs, n)
        # add node id to object
        self.nodes.append(nid)
        # add global config to osm object
        self.addNodeToTree(n)
        
    
    def getNode(self, lat: float, lon: float, node_id: int) -> minidom.Element:
        nid = node_id
        lat_str = str(lat)
        lon_str = str(lon)
        n = minidom.Document().createElement('node')
        n.setAttribute('id', str(nid))
        n.setAttribute('action', 'modify')
        n.setAttribute('visible', 'true')
        n.setAttribute('lat', lat_str)
        n.setAttribute('lon', lon_str)
        self.nodes.append(nid)
        return n

    def addNodeToTree(self, node: minidom.Element) -> None:
        self.osm.appendChild(node)
        
    def getTag(self, key: Any, value: Any) -> minidom.Element:
        key_str = str(key)
        val_str = str(value)
        t = minidom.Document().createElement('tag')
        t.setAttribute('k', str(key_str))
        t.setAttribute('v', str(val_str))
        return t
        
    def attachTagToNodeTree(self, tag: minidom.Element, node: minidom.Element) -> None:
        node.appendChild(tag)

    def getWay(self, lat_arr: List[float], lon_arr: List[float], name: str, way_id: int, 
               interpolate: bool=True,
               speed_arr: List[float] or None=None, 
               timestamp_arr: List[float] or None=None, 
               yaw_arr: List[float] or None=None) -> minidom.Element:
        w = minidom.Document().createElement('way')
        wid = way_id
        w.setAttribute('id', str(way_id))
        w.setAttribute('action', 'modify')
        w.setAttribute('visible', 'true')
        node_id_list = []
        assert len(lat_arr) == len(lon_arr)

        # initialize the trajectory flag as false
        trajectory_flag = False

        # check if the speed, timestamp, yaw array is valid
        if speed_arr is not None:
            assert len(speed_arr) == len(lat_arr)
            assert len(timestamp_arr) == len(lat_arr)
            assert len(yaw_arr) == len(lat_arr)
            trajectory_flag = True

        # add the nodes to the way
        for i in range(len(lat_arr)):
            nid = self.getId()
            n = self.getNode(lat_arr[i], lon_arr[i], nid)
            # add the tags to the node for trajectory of TV
            if trajectory_flag:
                t_speed = self.getTag('speed', str(speed_arr[i]))
                self.attachTagToNodeTree(t_speed, n)
                # print(timestamp_arr)
                t_timestamp = self.getTag('time', str(timestamp_arr[i]))
                self.attachTagToNodeTree(t_timestamp, n)
                t_yaw = self.getTag('yaw', str(yaw_arr[i]))
                self.attachTagToNodeTree(t_yaw, n)
            node_id_list.append(nid)
            self.addNodeToTree(n)
                
        for nid in node_id_list:
            r = minidom.Document().createElement('nd')
            r.setAttribute('ref', str(nid))
            w.appendChild(r)
        
        if trajectory_flag:
            t_gs = self.getTag('gs', 'trajectory')
            self.attachTagToNodeTree(t_gs, w)
            # add interpolation tag
            t_interpolate = self.getTag('interpolate', self.__python2xml__(interpolate))
            self.attachTagToNodeTree(t_interpolate, w)
        else:
            t_gs = self.getTag('gs', 'route')
            self.attachTagToNodeTree(t_gs, w)
        t_name = self.getTag('name', str(name))
        self.attachTagToNodeTree(t_name, w)
        self.reference_dict[wid] = node_id_list
        self.ways.append(wid)
            
        return w

    def addWayToTree(self, way: minidom.Element) -> None:
        self.osm.appendChild(way)
    
    def getVehicle(self,
                   vehicle_name: str,
                   route_name: str,
                   behavior_type: str,
                   starting_yaw_deg: float,
                   starting_vx: float,
                   starting_vy: float,
                   starting_ax: float,
                   starting_ay: float,
                   trajectory_lat: List[float],
                   trajectory_lon: List[float],
                   icon_lat: float,
                   icon_lon: float,
                   vehicle_id: int,
                   node_id: int,
                   behavior_tree_dir: str or None=None, 
                   vehicle_model: str='light_vehicle',
                   start: bool = True,
                   uses_speed_profile: bool = False
                  ):
        # prepare id for objects
        node_id = self.getId()
        way_id = self.getId()
        # add vehicle's route object to osm tree
        w = self.getWay(trajectory_lat, trajectory_lon, route_name, way_id)
        self.addWayToTree(w)
        
        # Vehicle representation is based on the definition of node.
        n = self.getNode(icon_lat, icon_lon, node_id)
        # add behavior tree tag
        if behavior_tree_dir:
            t_btree = self.getTag('btree', behavior_tree_dir)
            self.attachTagToNodeTree(t_btree, n)
        # add behavior type tag
        t_btype = self.getTag('btype', behavior_type)
        self.attachTagToNodeTree(t_btype, n)
        # add car model tag
        t_model = self.getTag('model', vehicle_model)
        self.attachTagToNodeTree(t_model, n)
        # add vehicle name tag
        t_name = self.getTag('name', vehicle_name)
        self.attachTagToNodeTree(t_name, n)
        # add route name tag, so the vehicle node can link with way.
        t_route = self.getTag('route', route_name)
        self.attachTagToNodeTree(t_route, n)
        # add start tag
        t_start = self.getTag('start', self.__python2xml__(start))
        self.attachTagToNodeTree(t_start, n)
        # add uses_speed_profile tag
        t_usespeedprofile = self.getTag('usespeedprofile', self.__python2xml__(uses_speed_profile))
        self.attachTagToNodeTree(t_usespeedprofile, n)
        # add vehicle id tag
        t_vid = self.getTag('vid', str(vehicle_id))
        self.attachTagToNodeTree(t_vid, n)
        # add yaw tag
        t_yaw = self.getTag('yaw', str(starting_yaw_deg))
        self.attachTagToNodeTree(t_yaw, n)
        # add vehicle initial state tag 
        # with the format of [vx, vy, ax, ay] in m/s and m/s^2
        init_state_str = str(starting_vx)+','+str(starting_vy)+','+str(starting_ax)+','+str(starting_ay)
        t_start_cartesian = self.getTag('start_cartesian', init_state_str)
        self.attachTagToNodeTree(t_start_cartesian, n)
        # add some extra tags
        t_gs = self.getTag('gs', 'vehicle')
        self.attachTagToNodeTree(t_gs, n)
        t_cycles = self.getTag('cycles', '1')
        self.attachTagToNodeTree(t_cycles, n)
        
        # add vehicle object to osm tree
        self.addNodeToTree(n)
        
        # record vehicle node id into the object
        self.ways.append(node_id)
        
        # return vehicle node object
        return n

    def getSDV(self,
               vehicle_name: str,
               route_name: str,
               starting_yaw_deg: float,
               starting_vx: float,
               starting_vy: float,
               starting_ax: float,
               starting_ay: float,
               trajectory_lat: List[float],
               trajectory_lon: List[float],
               icon_lat: float,
               icon_lon: float,
               vehicle_id: int,
               node_id: int,
               behavior_tree_dir: str or None=None, 
               vehicle_model: str='light_vehicle',
               start: bool = True,
               uses_speed_profile: bool = False
               ):
        # prepare id for objects
        node_id = self.getId()
        way_id = self.getId()
        # add vehicle's route object to osm tree
        w = self.getWay(trajectory_lat, trajectory_lon, route_name, way_id)
        self.addWayToTree(w)
        
        # Vehicle representation is based on the definition of node.
        n = self.getNode(icon_lat, icon_lon, node_id)
        # add behavior tree tag
        if behavior_tree_dir:
            t_btree = self.getTag('btree', behavior_tree_dir)
            self.attachTagToNodeTree(t_btree, n)
        # add behavior type tag as SDV
        t_btype = self.getTag('btype', 'SDV')
        self.attachTagToNodeTree(t_btype, n)
        # add car model tag
        t_model = self.getTag('model', vehicle_model)
        self.attachTagToNodeTree(t_model, n)
        # add vehicle name tag
        t_name = self.getTag('name', vehicle_name)
        self.attachTagToNodeTree(t_name, n)
        # add route name tag, so the vehicle node can link with way.
        t_route = self.getTag('route', route_name)
        self.attachTagToNodeTree(t_route, n)
        # add start tag
        t_start = self.getTag('start', self.__python2xml__(start))
        self.attachTagToNodeTree(t_start, n)
        # add uses_speed_profile tag
        t_usespeedprofile = self.getTag('usespeedprofile', self.__python2xml__(uses_speed_profile))
        self.attachTagToNodeTree(t_usespeedprofile, n)
        # add vehicle id tag
        t_vid = self.getTag('vid', str(vehicle_id))
        self.attachTagToNodeTree(t_vid, n)
        # add yaw tag
        t_yaw = self.getTag('yaw', str(starting_yaw_deg))
        self.attachTagToNodeTree(t_yaw, n)
        # add vehicle initial state tag 
        # with the format of [vx, ax, vy, ay] in m/s and m/s^2, respectively
        init_state_str = str(starting_vx)+','+str(starting_ax)+','+str(starting_vy)+','+str(starting_ay)
        t_start_cartesian = self.getTag('start_cartesian', init_state_str)
        self.attachTagToNodeTree(t_start_cartesian, n)
        # add some extra tags
        t_gs = self.getTag('gs', 'vehicle')
        self.attachTagToNodeTree(t_gs, n)
        t_cycles = self.getTag('cycles', '1')
        self.attachTagToNodeTree(t_cycles, n)
        
        # add vehicle object to osm tree
        self.addNodeToTree(n)
        
        # record vehicle node id into the object
        self.ways.append(node_id)
        
        # return vehicle node object
        return n

    def getTV(self,
              vehicle_name: str,
              trajectory_name: str,
              starting_yaw_deg: float,
              trajectory_lat: List[float],
              trajectory_lon: List[float],
              trajectory_yaw: List[float],
              trajectory_timestamp: List[float],
              trajectory_speed: List[float],
              icon_lat: float,
              icon_lon: float,
              vehicle_id: int,
              node_id: int,
              vehicle_model: str='light_vehicle',
              start: bool = True,
              uses_speed_profile: bool = False
              ):
        # prepare id for objects
        node_id = self.getId()
        way_id = self.getId()
        # add vehicle's route object to osm tree
        w = self.getWay(trajectory_lat, trajectory_lon, trajectory_name, way_id, 
                        interpolate=True, speed_arr=trajectory_speed, 
                        timestamp_arr=trajectory_timestamp, yaw_arr=trajectory_yaw)
        self.addWayToTree(w)
        
        # Vehicle representation is based on the definition of node.
        n = self.getNode(icon_lat, icon_lon, node_id)
        # add behavior type tag
        t_btype = self.getTag('btype', 'TV')
        self.attachTagToNodeTree(t_btype, n)
        # add car model tag
        t_model = self.getTag('model', vehicle_model)
        self.attachTagToNodeTree(t_model, n)
        # add vehicle name tag
        t_name = self.getTag('name', vehicle_name)
        self.attachTagToNodeTree(t_name, n)
        # add route name tag, so the vehicle node can link with way.
        t_trajectory = self.getTag('trajectory', trajectory_name)
        self.attachTagToNodeTree(t_trajectory, n)
        # add start tag
        t_start = self.getTag('start', self.__python2xml__(start))
        self.attachTagToNodeTree(t_start, n)
        # add uses_speed_profile tag
        t_usespeedprofile = self.getTag('usespeedprofile', self.__python2xml__(uses_speed_profile))
        self.attachTagToNodeTree(t_usespeedprofile, n)
        # add vehicle id tag
        t_vid = self.getTag('vid', str(vehicle_id))
        self.attachTagToNodeTree(t_vid, n)
        # add yaw tag
        t_yaw = self.getTag('yaw', str(starting_yaw_deg))
        self.attachTagToNodeTree(t_yaw, n)
        # add some extra tags
        t_gs = self.getTag('gs', 'vehicle')
        self.attachTagToNodeTree(t_gs, n)
        t_cycles = self.getTag('cycles', '1')
        self.attachTagToNodeTree(t_cycles, n)
        
        # add vehicle object to osm tree
        self.addNodeToTree(n)
        
        # record vehicle node id into the object
        self.ways.append(node_id)
        
        # return vehicle node object
        return n


    def addSDV(self,
               vehicle_name: str,
               route_name: str,
               starting_yaw_deg: float,
               starting_vx: float,
               starting_vy: float,
               starting_ax: float,
               starting_ay: float,
               trajectory_lat: List[float],
               trajectory_lon: List[float],
               icon_lat: float,
               icon_lon: float,
               vehicle_id: int,
               behavior_tree_dir: str or None=None,
               vehicle_model: str='light_vehicle',
               start: bool = True,
               uses_speed_profile: bool = False
               ):
        node_id = self.getId()
        v = self.getSDV(vehicle_name, 
                        route_name, 
                        starting_yaw_deg, 
                        starting_vx, starting_vy, 
                        starting_ax, starting_ay,
                        trajectory_lat, trajectory_lon, 
                        icon_lat, icon_lon, 
                        vehicle_id, 
                        node_id, 
                        behavior_tree_dir, 
                        vehicle_model, 
                        start, 
                        uses_speed_profile)
        self.addNodeToTree(v)


    def addTV(self,
              vehicle_name: str,
              trajectory_name: str,
              starting_yaw_deg: float,
              trajectory_lat: List[float],
              trajectory_lon: List[float],
              trajectory_yaw: List[float],
              trajectory_timestamp: List[float],
              trajectory_speed: List[float],
              icon_lat: float,
              icon_lon: float,
              vehicle_id: int,
              vehicle_model: str='light_vehicle',
              start: bool = True,
              uses_speed_profile: bool = False
              ):
        node_id = self.getId()
        v = self.getTV(vehicle_name, 
                       trajectory_name, 
                       starting_yaw_deg, 
                       trajectory_lat, trajectory_lon, 
                       trajectory_yaw, trajectory_timestamp, trajectory_speed,
                       icon_lat, icon_lon, 
                       vehicle_id, 
                       node_id, 
                       vehicle_model, 
                       start, 
                       uses_speed_profile)
        self.addNodeToTree(v)
from sv.ManeuverUtils import *
from sv.SDVTrafficState import *
from TrafficLight import TrafficLightColor
from sv.SDVTrafficState import *
from sv.btree.BTreeConstants import *


class BTreeConditions:
        
    def __init__(self):
        self.yielding_vehicles = None
        pass

    #===========================================================
    #Basic conditions
    #===========================================================

    def sim_time(self,traffic_state:TrafficState, kwargs):
        t = get_node_param(kwargs,'t', None)
        if t:
            #accepts exact time or at most a planning cycle delay
            return t <= traffic_state.sim_time <= ( t + 1/PLANNER_RATE ) 
        tmin = get_node_param(kwargs,'tmin', 0)
        tmax = get_node_param(kwargs,'tmax',float('inf'))
        return tmin <= traffic_state.sim_time <= tmax

    def vehicle_state(self,traffic_state:TrafficState, kwargs):
        """ Checks if vehicle state has reached long and lateral values at minimum.
            s_pos is not available since the frenet frame is moving.
            pos can only be used as relative to other agents (use delta_state condition)
            If no vid or zid is assigned, defaults to self ID
        """
        #retrieve vehicle by looking at vid or zid. 
        if 'vid' in kwargs or "zid" in kwargs:
            vehicle = get_vehicle_by_ids(traffic_state, kwargs) 
            state = vehicle.state
        else:
            #if not parameter assigned, defaults to self
            state = traffic_state.vehicle_state

        if state is not None:
            s_pos = get_node_param(kwargs,'s_pos')
            if s_pos and not (state.d_pos >= s_pos):
                return False
            s_vel = get_node_param(kwargs,'s_vel')
            if s_vel and not (state.s_vel >= s_vel):
                return False
            s_acc = get_node_param(kwargs,'s_acc')
            if s_acc and not (state.s_acc >= s_acc):
                return False
            d_pos = get_node_param(kwargs,'d_pos')
            if d_pos and not (state.d_pos >= d_pos):
                return False
            d_vel = get_node_param(kwargs,'d_vel')
            if d_vel and not (state.d_vel >= d_vel):
                return False
            d_acc = get_node_param(kwargs,'d_acc')
            if d_acc and not (state.d_acc >= d_acc):
                return False
        
        return True 
    
    def delta_vehicle_state(self,traffic_state:TrafficState, kwargs):
        """ Checks if vehicle state has reached delta values at minimum.
            Always compare between self the the vehicle with vid (or per occupancy zone)
        """
        
        #retrieve vehicle by looking at vid or zid. 
        vehicle = get_vehicle_by_ids(traffic_state, kwargs) 
        if not vehicle:
            return False
        my_state:VehicleState = traffic_state.vehicle_state
        v_state = vehicle.state

        s_pos = get_node_param(kwargs,['s_pos', 's'])
        if s_pos and not (abs(v_state.s - my_state.s) >= s_pos):
            return False
        s_vel = get_node_param(kwargs,'s_vel')
        if s_vel and not (abs(v_state.s_vel - my_state.s_vel) >= s_vel):
            return False
        s_acc = get_node_param(kwargs,'s_acc')
        if s_acc and not (abs(v_state.s_acc - my_state.s_acc) >= s_acc):
            return False
        d_pos = get_node_param(kwargs,['d_pos','d'])
        if d_pos and not (abs(v_state.d - my_state.d) >= d_pos):
            return False
        d_vel = get_node_param(kwargs,'d_vel')
        if d_vel and not (abs(v_state.d_vel - my_state.d_vel) >= d_vel):
            return False
        d_acc = get_node_param(kwargs,'d_acc')
        if d_acc and not (abs(v_state.d_acc - my_state.d_acc) >= d_acc):
            return False
        return True 

    def wait(self,traffic_state:TrafficState, kwargs):
        '''wait condition is a timer handled handled by the BTReeLeaves'''
        pass  

    #===========================================================
    #Routing and Driving Mission
    #===========================================================

    def reached_goal(self,traffic_state:TrafficState, kwargs):
        """ Checks if the vehicle has reached or passed the goal point in the frenet frame.
        """
        if traffic_state.goal_point_frenet is None:
            return False
        threshold = get_node_param(kwargs,"distance",40)
        goal_s =  traffic_state.goal_point_frenet[0]
        return traffic_state.route_complete and (goal_s - traffic_state.vehicle_state.s < threshold)

    def at_lane_change_segment(self,traffic_state:TrafficState, kwargs):
        """ Checks if the vehicle is inside a zone where it needs to change lanes to conintue on route.
        """
        return traffic_state.lane_swerve_target is not None

    def target_lane(self,traffic_state:TrafficState, kwargs):
        lid = get_node_param(kwargs,['lid','target_lid','target_lane_id','target_lane'], required=True)
        if traffic_state.lane_swerve_target is not None:
                if (lid == traffic_state.lane_swerve_target):
                    return True
        return False

    def out_of_route(self,traffic_state:TrafficState, kwargs):
        '''
        Checks if vehicle is out of planned route
        TODO: currently is automatically done by Routing module. 
        Will move to this condition to allow custom Behavior
        '''
        return False

    def reroute(self,traffic_state:TrafficState, kwargs):
        '''
        Action Node. Calls the reoute module. 
        Always return False because is not an actual decision
        TODO: currently is automatically done by Routing module. 
        Will move to this condition to allow custom Behavior
        '''
        return False

    def set_target_lane(self,traffic_state:TrafficState, kwargs):
        lid = get_node_param(kwargs,['lid','target_lid','target_lane_id','target_lane'], required=True)
        if lid > 0:
            traffic_state.target_lane = RIGHT
        else:
            traffic_state.target_lane = LEFT
        
        return True
    #===========================================================
    #Traffic Interactions
    #===========================================================

    def is_ego(self,traffic_state:TrafficState, kwargs):
        '''Checks if given vehicle is Ego. 
        '''
        if 'vid' in kwargs or 'lid' in kwargs or 'zid' in kwargs:
            vehicle = get_vehicle_by_ids(traffic_state, kwargs) 
            if not vehicle:
                log.warning("No vehicle found in condition is_ego")
                return False
            return True if vehicle.id == EGO else False
        else:
            log.warning("Condition is_ego requires at least one id")
            return False

    def vehicle_stopped(self,traffic_state:TrafficState, kwargs):
        '''Checks if given vehicle has stopped. 
            If no id is given, assumes self
        '''
        vel_threshold = get_node_param(kwargs,"vel",0.05) #threshold to identify as stopped
        
        if 'vid' in kwargs or 'lid' in kwargs or 'zid' in kwargs:
            vehicle = get_vehicle_by_ids(traffic_state, kwargs) 
            if not vehicle:
                log.warning("No vehicle found in condition vehicle_stopped")
                return False
            vehicle_state = vehicle.state
        else:
            vehicle_state = traffic_state.vehicle_state
        
        return abs(vehicle_state.s_vel) < vel_threshold

    def vehicle_moving(self,traffic_state:TrafficState, kwargs):
        '''Checks if given vehicle is moving. 
            If no id is given, assumes self
        '''
        #log.info("VID {} condition vehicle moving".format(traffic_state.vid))
        vel_threshold = get_node_param(kwargs,"vel",0.05) #threshold to identify as moving

        if 'vid' in kwargs or 'lid' in kwargs or 'zid' in kwargs:
            vehicle = get_vehicle_by_ids(traffic_state, kwargs) 
            if not vehicle:
                log.warning("No vehicle found in condition vehicle_moving")
                return False
            vehicle_state = vehicle.state
        else: #self
            vehicle_state = traffic_state.vehicle_state
        
        return abs(vehicle_state.s_vel) > vel_threshold
    
    def vehicle_yielding(self,traffic_state:TrafficState, kwargs):
        '''Checks if given vehicle is stopped and at yielding position 
            (stop line or right before conflicint lanelet). 
            If no id is given, assumes self
        '''
        #REVISE: only works for self
        dist_threshold = get_node_param(kwargs,'distance', 1)
        vel_threshold = get_node_param(kwargs,'velocity', 0.01)
        
        if 'vid' in kwargs or 'lid' in kwargs or 'zid' in kwargs:
            vehicle = get_vehicle_by_ids(traffic_state, kwargs) 
            if not vehicle:
                log.warning("No vehicle found in condition vehicle_yielding")
                return False
            vehicle_state = vehicle.state
        else: #self
            vehicle_state = traffic_state.vehicle_state
            if traffic_state.lane_config.stopline_pos is None:
                return False
            else:
                pos_s = traffic_state.lane_config.stopline_pos[0]
                dist = pos_s - traffic_state.vehicle_state.s
                vel = abs(traffic_state.vehicle_state.s_vel)
                #print("BEHAVIOR: dist {},vel{}".format(dist,vel))
                return (dist < dist_threshold and vel < vel_threshold)
        
    def vehicle_parked(self,traffic_state:TrafficState, kwargs):
        '''
        Checks if given vehicle has stopped and at parking position
        '''
        #REVISE: limited to current lane, and right parking only

        vel_threshold = get_node_param(kwargs,"vel",0.05) #velocity threshold to identify as stopped
        distance_threshold = get_node_param(kwargs,"distance",0.5) #distance threshold from lane  centre to identify as stopped 
        
        if 'vid' in kwargs or 'lid' in kwargs or 'zid' in kwargs:
            vehicle = get_vehicle_by_ids(traffic_state, kwargs) 
            if not vehicle:
                log.warning("No vehicle found in condition vehicle_yielding")
                return False
            vehicle_state = vehicle.state
        else: #self
            vehicle_state = traffic_state.vehicle_state

        if abs(vehicle_state.s_vel) < vel_threshold: #stopped
            if traffic_state.lane_config._right_lane is None:    #is the rightmost lane
                if vehicle_state.d < (-distance_threshold):  #is positioned at the right of lane centre
                    log.debug(vehicle_state.d)
                    #if (vehicle.state.d - VEHICLE_WIDTH/2) < traffic_state.lane_config.get_central_d())
                    return True
            return False     

    # lead
    def lv_stopped(self,traffic_state:TrafficState, kwargs):
        if traffic_state.road_occupancy.front:
            lead = traffic_state.road_occupancy.front
            kwargs["vid"] = lead.id
            return self.vehicle_stopped(traffic_state,kwargs)
        else:
            log.warning("No lead vehicle")
            return False

    def lv_moving(self,traffic_state:TrafficState, kwargs):
        if traffic_state.road_occupancy.front:
            lead = traffic_state.road_occupancy.front
            kwargs["vid"] = lead.id
            return self.vehicle_moving(traffic_state,kwargs)
        else:
            log.warning("No lead vehicle")
            return False

    def lv_parked(self,traffic_state:TrafficState, kwargs):
        if traffic_state.road_occupancy.front:
            lead = traffic_state.road_occupancy.front
            kwargs["vid"] = lead.id
            return self.vehicle_parket(traffic_state,kwargs)
        else:
            log.warning("No lead vehicle")
            return False

    #surrounding

    def can_lane_change(self,traffic_state:TrafficState, kwargs):
        '''checks if there is space for a lane change
        '''
        #REVISE: Time Gap
        lid = get_node_param(kwargs,['lid','target_lid','target_lane_id','target_lane'], required=True)
        if lid is None:
            #check if target is by route
            if traffic_state.lane_swerve_target is not None:
                lid = traffic_state.lane_swerve_target
        distance_gap = get_node_param(kwargs,['gap', 'distance_gap'], default=20)
        time_gap = get_node_param(kwargs,['time_gap'], default=2)
        include_opposite = get_node_param(kwargs,['include_opposite'], default=False,int_to_boolean=True)
        if not lid:
            return False
        
        target_lane_config = traffic_state.lane_config.get_neighbour(lid,include_opposite)
        if not target_lane_config:
            log.warning("No reachable {} lane for lane changing vehicle {}".format(
                "LEFT" if lid == 1 else "RIGHT",traffic_state.vid))
            return False
        smin = traffic_state.vehicle_state.s - VEHICLE_LENGTH/2 - distance_gap
        smax = traffic_state.vehicle_state.s + VEHICLE_LENGTH/2 + distance_gap
        vehicles_in_lane = list(filter(
            lambda v: smin < v.state.s < smax,
            get_vehicles_in_lane(target_lane_config, traffic_state.traffic_vehicles)
            ))

        return len(vehicles_in_lane) == 0

    # metrics

    def gap(self,traffic_state:TrafficState, kwargs):
        time_threshold = get_node_param(kwargs,"time", None)
        distance_threshold = get_node_param(kwargs,"distance", None)
        #pbound = get_node_param(kwargs,"pbound", None)
        #min_distance = get_node_param(kwargs,"min_distance", None)
        #max_distance = get_node_param(kwargs,"max_distance", None)
        vehicle = get_vehicle_by_ids(traffic_state, kwargs) 
        result = False
        if vehicle:
            gap = range_gap(traffic_state.vehicle_state,vehicle)
            if distance_threshold: 
                result = (gap > distance_threshold)
                #print (result)
            if time_threshold:
                if gap > 0:
                    time_to_vehicle = gap / vehicle.state.s_vel if vehicle.state.s_vel != 0 else float('inf')
                    result = (0 <= time_to_vehicle < time_threshold)
        return result
   
    #===========================================================
    #Road and Regulatory Elements
    #===========================================================
    
    def approaching_intersection(self,traffic_state:TrafficState, kwargs):    
        '''checks if vehicle is approaching a regulated intersection
        '''
        threshold = get_node_param(kwargs,'distance',30)
        #approaching an intersection
        for intersection in traffic_state.intersections:
            if (isinstance(intersection, RightOfWayIntersection) 
            or isinstance(intersection, AllWayStopIntersection) 
            or isinstance(intersection, TrafficLightIntersection)):
                if intersection.stop_position is None:
                    continue
                return intersection.stop_position[0] -traffic_state.vehicle_state.s < threshold 
        return False
    
    def approaching_stop_sign(self,traffic_state:TrafficState, kwargs):
        '''check for generic stop definition when map is incomplete and
            doest not contain a proper regulatory element'''
        if traffic_state.lane_config.stopline_pos is None:
            return False
        threshold = get_node_param(kwargs,'threshold',30)
        pos_s = traffic_state.lane_config.stopline_pos[0]
        dist = pos_s - traffic_state.vehicle_state.s
        return (dist < threshold)

    def intersection_type(self,traffic_state:TrafficState, kwargs):
        mytype = get_node_param(kwargs,'type',30)
        for intersection in traffic_state.intersections:
            if isinstance(intersection, RightOfWayIntersection) and mytype == RIGHT_OF_WAY:
                return True 
            if isinstance(intersection, AllWayStopIntersection) and mytype ==  ALL_WAY_STOP:
                return True 
            if isinstance(intersection, TrafficLightIntersection) and mytype == TRAFFIC_LIGHT:
                return True 
            if isinstance(intersection, PedestrianIntersection) and mytype == PEDESTRIAN_CROSS:
                return True 
        return False

    def unsignalized_intersection(self,traffic_state:TrafficState, kwargs):
        #TODO
        return False

    def yield_role(self,traffic_state:TrafficState, kwargs):
        #Only vehicles with yield role get the reg eleme. change this
        for intersection in traffic_state.intersections:
            if isinstance(intersection, RightOfWayIntersection):
                return True
        return False

    def intersection_occupied(self,traffic_state:TrafficState, kwargs):
        #print("=== Intersection VID {} occupied? {}".format(
        #    traffic_state.vid, 
        #    traffic_state.road_occupancy.intersecting_zone))
        for intersection in traffic_state.intersections:
            if isinstance(intersection, RightOfWayIntersection):
                if len(traffic_state.road_occupancy.row_zone) > 0:
                    log.debug("Occupied row zone {}".format(traffic_state.road_occupancy.row_zone))
                    return True
            if isinstance(intersection, AllWayStopIntersection):
                if len(traffic_state.road_occupancy.intersecting_zone) > 0:
                    #print("Occupied".format(traffic_state.vid))
                    return True
        #print("Free".format(traffic_state.vid))
        #print(traffic_state.road_occupancy)
        return False  #No intersection detected or free

    def aws_yielding(self,traffic_state:TrafficState, kwargs):
        #if not (traffic_state.vid == 2):
        #    return False
        vel_threshold = 0.03  #for vehicles that start moving
        wait_time_threshold = 5
        draw_p = 0.05 #chances of breaking order (0 to 1)

        for intersection in traffic_state.intersections:
            if isinstance(intersection, AllWayStopIntersection):
                yielding_vehicles = traffic_state.road_occupancy.yielding_zone
                #intersection_vehicles = traffic_state.road_occupancy.intersecting_zone
                #appr_yielding_zone = traffic_state.road_occupancy.appr_yielding_zone
                #Debug
                #print("====Yield Inters VID {}, memory {}, yieldzone {}, appyz {},intersection {}".format(
                #    traffic_state.vid, 
                #    self.yielding_vehicles,
                #    yielding_vehicles,
                #    appr_yielding_zone,
                #    intersection_vehicles))
                if len(traffic_state.road_occupancy.yielding_zone)==0: 
                    #no vehicles, go
                    #print("no vehicles")
                    self.yielding_vehicles = None
                    return False #go
                else:
                    #has yielding vehicles
                    #first time
                    if self.yielding_vehicles is None: 
                        self.yielding_vehicles = [traffic_state.sim_time, yielding_vehicles] #save current state
                        #print("first time, then wait")
                        return True #wait
                    else:
                        if len(self.yielding_vehicles[1]) == 0:
                            #I have priority
                            #print("have priority")
                            #before, check if any yielding vehicle started moving
                            for vid in yielding_vehicles:
                                vehicle = get_vehicle_by_ids(traffic_state, {'vid':vid} )
                                if vehicle:
                                    speed = vehicle.state.get_cartesian_speed()
                                    if speed  > vel_threshold:
                                        #print("but other vehicle {} started moving".format(vid))
                                        return True #wait
                            self.yielding_vehicles = None
                            return False
                        else:
                            #no priority
                            wait_time = traffic_state.sim_time - self.yielding_vehicles[0]
                            #print("wait time is {}".format(wait_time))
                            #remove gone vehicles from list.
                            for vid in self.yielding_vehicles[1]:
                                if vid is not traffic_state.vid:
                                    if vid not in traffic_state.road_occupancy.yielding_zone: 
                                        self.yielding_vehicles[1].remove(vid)
                            #take a chance if waiting too long
                            if wait_time > wait_time_threshold:
                                #print("no priority >> take a chance")
                                if random.random() <= draw_p:
                                #if random.choices(population= [0,1], weights=[1-draw_weight,draw_weight], k=1)[0] > 0:
                                    #print("DRAW GO")
                                    #before, check if any yielding vehicle started moving
                                    for vid in yielding_vehicles:
                                        vehicle = get_vehicle_by_ids(traffic_state, {'vid':vid} )
                                        if vehicle:
                                            speed = vehicle.state.get_cartesian_speed()
                                            #print("vid {} vel {}".format(vid,speed))
                                            if speed > vel_threshold:
                                                #print("but other vehicle {} started moving".format(vid))
                                                return True #wait
                                    self.yielding_vehicles = None
                                    return False
                                else:
                                    #print(":( wait")
                                    return True
            return True # wait
                
        return False

    def lane_occupied(self,traffic_state:TrafficState, kwargs):
        lane_occupied, lv_id = is_in_following_range(
            traffic_state.vid,
            traffic_state.vehicle_state,
            traffic_state.traffic_vehicles,
            traffic_state.lane_config,
            kwargs['time_gap'] if 'time_gap' in kwargs else 5 ,
            kwargs['distance_gap'] if 'distance_gap' in kwargs else 30 )
        return lane_occupied

    # traffic lights

    def traffic_light_state(self,traffic_state:TrafficState, kwargs):
        color = get_node_param(kwargs,'color',30)
        for intersection in traffic_state.intersections:
            if isinstance(intersection, TrafficLightIntersection):
                if intersection.color == TrafficLightColor.Green and color == GREEN:
                    return True 
                if intersection.color == TrafficLightColor.Red and color == RED:
                    return True 
                if intersection.color == TrafficLightColor.Yellow and color == YELLOW:
                    return True 



#====================
#Utils
#====================


def get_node_param(kwargs, param_names, default = None, required = False, int_to_boolean=False):
    if type(param_names) == list:
        for name in param_names:
            if name in kwargs:
                value = kwargs[name]
                if int_to_boolean:
                    value = bool(int(value))
                return value
    elif param_names in kwargs:
            return kwargs[param_names]
    if required:
            log.error("Missing required node parameter {}".format(param_names))
            return None
    return default

def get_vehicle_by_ids(traffic_state:TrafficState, kwargs):
        ''' Returns vehicle based on identifiers:
            zids: zone clockwise from front
                1: front, 2: front left, 3: left, 4: back left
                5: back, 6: back right, 7: right, 8: front right
            lid: lane (or lane_id)
                +1 (positive) : LEFT
                -1 (negative) : RIGHT
            vid: vehicle in traffic
        '''

        #Find VID
        vid = get_node_param(kwargs,'vid', None)
        if not vid:
            #find vehicle on road
            lid = get_node_param(kwargs,'lid', None)
            zid = get_node_param(kwargs,'zid', None)
            if lid:
                if lid < 0: 
                    vid = traffic_state.road_occupancy.left
                else:
                    vid = traffic_state.road_occupancy.right
            if zid:
                if zid == 1: 
                    vid = traffic_state.road_occupancy.front
                elif zid == 2: 
                    vid = traffic_state.road_occupancy.left_front
                elif zid == 3: 
                    vid = traffic_state.road_occupancy.left
                elif zid == 4: 
                    vid = traffic_state.road_occupancy.left_back
                elif zid == 5: 
                    vid = traffic_state.road_occupancy.back
                elif zid == 6: 
                    vid = traffic_state.road_occupancy.right_back
                elif zid == 7: 
                    vid = traffic_state.road_occupancy.right
                elif zid == 8: 
                    vid = traffic_state.road_occupancy.right_front
        
        #get actual vehicle object
        if vid:
            if vid in traffic_state.traffic_vehicles:
                return traffic_state.traffic_vehicles[vid]
            elif vid in traffic_state.traffic_vehicles_orp:
                return traffic_state.traffic_vehicles_orp[vid]
        #print(traffic_state.traffic_vehicles)
        log.warning("No Vehicle found with IDs {} ".format(kwargs))
        return None

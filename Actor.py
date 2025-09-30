#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# VehicleState and Motion Plan
# --------------------------------------------
from dataclasses import dataclass
from enum import IntEnum
from util.Utils import to_equation, differentiate, normalize_angle
from SimConfig import *
import logging
import numpy as np
log = logging.getLogger(__name__)

class SpeedQualifier(IntEnum):
    CONSTANT = 0          # treat the speed as constant throughout the path (default)
    MAXIMUM = 1          # treat the given speed as the upper bound
    MINIMUM = 2          # treat the given speed as the lower bound
    INITIAL = 3          # start the agent with the given speed but adjust for collision as needed

class ActorSimState(IntEnum):
    INACTIVE = 0          #not in simulation, not present in traffic, and not visible for other agents
    ACTIVE = 1            #in simulation and visible to other agents
    INVISIBLE = 2         #in simulation but NOT visible to other agents (for reference)

class Actor(object):
    def __init__(self, id, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0], frenet_state=[0.0,0.0,0.0, 0.0,0.0,0.0], yaw=0.0, state=None, length=0.0, width=0.0):
        self.id = id
        self.name = name
        self.length = length
        self.width = width
        self.radius = min(length, width) / 2.0
        self.sim_state = ActorSimState.ACTIVE
        self.type = None
        self.model = None
        self.ghost_mode = False
        self.sim_traffic = None
        self.released = False
        self.set_constant = False
        self.c = 0
        self.k = 0
        self.alpha1 = 0

        #state
        #start state in sim frame
        self.state:ActorState = state or ActorState()
        self.state.x     = start_state[0]
        self.state.x_vel = start_state[1]
        self.state.x_acc = start_state[2]
        self.state.y     = start_state[3]
        self.state.y_vel = start_state[4]
        self.state.y_acc = start_state[5]
        #start state in frenet
        self.state.s     = frenet_state[0]
        self.state.s_vel = frenet_state[1]
        self.state.s_acc = frenet_state[2]
        self.state.d     = frenet_state[3]
        self.state.d_vel = frenet_state[4]
        self.state.d_acc = frenet_state[5]
        #common state
        self.state.yaw = yaw
        #path following
        self.path = [] #list of PathNode
        self.debug_path = [] # list of (x, y)
        self.set_speed = None #desired constant speed in m/s, used if speed profile is not provided in path
        self.reference_speed = None # for Pedestrian?
        self.collision_vid = None
        self.collision_point = None
        self.scenario_vehicles = [] #dict of vehicles in the scenario, used for collision checking
        self.keep_active = False #if true, vehicle will remain active after trajectory ends
        self.current_path_node = 0


    def future_euclidian_state(self, dt):
        """ Predicts a new state based on time and vel.
            Used for collision prediction and charts
            Note: Acc can rapidly change. Using the current acc to predict future
            can lead to overshooting forward or backwards when vehicle is breaking
            TODO: predict using history + kalman filter
        """
        state = [
            self.state.x + (self.state.x_vel * dt),
            self.state.x_vel,
            self.state.x_acc,
            self.state.y + (self.state.y_vel * dt),
            self.state.y_vel,
            self.state.y_acc
        ]
        return state

    def future_state(self, dt):
        """ Predicts a new state based on time and vel.
            Used for collision prediction and charts
            Note: Acc can rapidly change. Using the current acc to predict future
            can lead to overshooting forward or backwards when vehicle is breaking
            TODO: predict using history + kalman filter
        """
        state = [
            self.state.s + (self.state.s_vel * dt), #+ (self.state.s_acc * dt * dt),
            self.state.s_vel, #+ (self.state.s_acc * dt),
            self.state.s_acc,
            self.state.d + (self.state.d_vel * dt), #+ (self.state.d_acc * dt * dt),
            self.state.d_vel, #+ (self.state.d_acc * dt),
            self.state.d_acc
        ]
        return state

    def stop(self):
        pass

    def force_stop(self):
        #keep current position while forcing vel and acc to 0.0
        self.state.x_vel = self.state.x_acc = self.state.y_vel = self.state.y_acc = 0.0
        self.state.s_vel = self.state.s_acc = self.state.d_vel = self.state.d_acc = 0.0
        pass

    def remove(self):
        self.sim_state = ActorSimState.INACTIVE
        log.warning("Actor id {} is now INACTIVE".format(self.id))

    def tick(self, tick_count, delta_time, sim_time):
        pass

    def follow_trajectory(self, sim_time, trajectory):
        if trajectory:
            start_time = float(trajectory[0].time) #first node
            end_time = float(trajectory[-1].time) #last node
            #During trajectory
            if start_time <= sim_time <= end_time:
                #Trajectory starts
                if self.sim_state is ActorSimState.INACTIVE:
                    log.warning(f"Actor ID {self.id} is now ACTIVE")
                    self.sim_state = ActorSimState.ACTIVE
                    if self.ghost_mode:
                        self.sim_state = ActorSimState.INVISIBLE
                        log.warning(f"vid {self.id} is now INVISIBLE")
                    if EVALUATION_MODE:
                        if -self.id in self.sim_traffic.vehicles:
                            self.sim_traffic.vehicles[-self.id].sim_state = ActorSimState.ACTIVE
                            log.warning(f"vid {-self.id} is now ACTIVE")

                #find closest pair of nodes
                for i in range(len(trajectory)-1):
                    n1 = trajectory[i]
                    n2 = trajectory[i+1]
                    if (n1.time <= sim_time < n2.time): # n2 must be strictly after n1
                        dx = n2.x - n1.x
                        dx_vel = n2.x_vel - n1.x_vel
                        dy = n2.y - n1.y
                        dy_vel = n2.y_vel - n1.y_vel
                        dt = n2.time - n1.time
                        x_acc = dx_vel/dt
                        y_acc = dy_vel/dt
                        pdiff = (sim_time - n1.time)/dt #always positive
                        # Interpolate
                        self.state.set_X([n1.x + (dx * pdiff), n2.x_vel, x_acc])
                        self.state.set_Y([n1.y + (dy * pdiff), n2.y_vel, y_acc])
                        self.state.yaw = n1.yaw
                        break

            #After trajectory, stay in last position or get removed
            if sim_time > end_time:
                if not self.keep_active:
                    self.state.set_X([-9999, 0, 0])
                    self.state.set_Y([-9999, 0, 0])
                    if self.sim_state is ActorSimState.ACTIVE:
                        self.remove()
                else:
                    self.force_stop()
                     
    def get_velocity_yaw(self, velocity_x, velocity_y):
        return math.atan2(velocity_y, velocity_x)
    
    def get_curr_and_prev_path_nodes(self):
        if self.path:
            for i in range(len(self.path)-1):
                n1 = self.path[i]
                n2 = self.path[i+1]

                if (n1.s <= self.state.s <= n2.s):
                    return n1, n2
        return None, None

    def get_collision_pt(self, vehicle_pos, vehicle_vel):
        vehicle_yaw = self.get_velocity_yaw(vehicle_vel[0], vehicle_vel[1])

        def is_between(yaw, yaw1, yaw2):
            yaw = normalize_angle(yaw)
            yaw1 = normalize_angle(yaw1)
            yaw2 = normalize_angle(yaw2)
            if yaw1 < yaw2:
                return yaw1 <= yaw <= yaw2
            else:
                return yaw1 <= yaw or yaw >= yaw2
            
        for i in range(len(self.path)-1):
            n1 = self.path[i]
            n2 = self.path[i+1]
            
            n1_vector = np.array([n1.x, n1.y])
            n2_vector = np.array([n2.x, n2.y])
            
            vec_a = n1_vector - vehicle_pos
            vec_b = n2_vector - vehicle_pos

            yaw_a = math.atan2(vec_a[1], vec_a[0])
            yaw_b = math.atan2(vec_b[1], vec_b[0])
            
            if is_between(vehicle_yaw, yaw_a, yaw_b):
                
                # cramer's rule
                x1, y1 = n1.x, n1.y
                x2, y2 = n2.x, n2.y
                x3, y3 = vehicle_pos
                x4, y4 = vehicle_pos + vehicle_vel
                
                denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
                if denom == 0:
                    continue  # Lines are parallel

                px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denom
                py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denom

                return np.array([px, py]), n1, n2
            
        return None
        
    def configure_path_following(self, path, set_speed=None, speed_qualifier=SpeedQualifier.INITIAL, collision_vid=None, collision_point=None, keep_active=False):
        """
        For path following actors (PP and PV), must be called before follow_path can be used
        Arguments:
        - path: list of PathNode consisting of at least 2 nodes
        - set_speed: desired constant speed in m/s, used if speed profile is not provided in path
        - speed_qualifier: used only when set_speed is provided
        - collision_vid: vehicle id to be collided with, used for ensured collision
        - collision_point: designated collision point, used for ensured collision with fixed point

        For constant speed, set set_speed to the desired speed and speed_qualifier as required 
        For speed profile, set set_speed=None and speed_qualifier to None
        For ensured collision, set collision_vid to the vehicle id to be collided with
        For ensured collision with the designated point, set collision_point
        """
        self.path = path
        self.debug_path = [(n.x, n.y) for n in self.path]
        self.set_speed = set_speed
        self.speed_qualifier = speed_qualifier
        self.collision_vid = collision_vid
        self.collision_point = collision_point
        self.keep_active = keep_active


    def follow_path(self, delta_time):
        if self.path:
            # Which path node we have most recently passed
            node_checkpoint = 0
            collision_segment_prev_node = None
            collision_segment_next_node = None
            time_to_collision = None
            collision_vehicle = None
            collision_pt = None

            if self.collision_vid is not None:
                try:
                    collision_vehicle = self.scenario_vehicles[self.collision_vid]
                    vehicle_pos = np.array([collision_vehicle.state.x, collision_vehicle.state.y])
                    vehicle_vel = np.array([collision_vehicle.state.x_vel, collision_vehicle.state.y_vel])
                except KeyError:
                    raise KeyError(f"collision vehicle with vid {self.collision_vid} not found in scenario")

                # Calculate collision details if vehicle exists
                if collision_vehicle:
                    if self.collision_point is not None:
                        #use provided collision point
                        collision_pt = [self.collision_point.x, self.collision_point.y]
                        collision_segment_prev_node, collision_segment_next_node = self.get_curr_and_prev_path_nodes()
                        # Euclidean distance between vehicle and collision point
                        collision_vehicle_dist_to_collision = np.sqrt(np.sum((collision_pt - vehicle_pos) ** 2))
                        if collision_vehicle.state.s_vel == 0.0:
                            time_to_collision = float('inf')
                        else:
                            time_to_collision = collision_vehicle_dist_to_collision / collision_vehicle.state.s_vel
                    else:
                        #find the point once and save the result
                        collision_pt_result = self.get_collision_pt(vehicle_pos, vehicle_vel)
                        if collision_pt_result is not None:
                            collision_pt, collision_segment_prev_node, collision_segment_next_node = collision_pt_result
                            # Euclidean distance between vehicle and collision point
                            collision_vehicle_dist_to_collision = np.sqrt(np.sum((collision_pt - vehicle_pos) ** 2))
                            if collision_vehicle.state.s_vel == 0.0:
                                time_to_collision = float('inf')
                            else:
                                time_to_collision = collision_vehicle_dist_to_collision / collision_vehicle.state.s_vel


            for i in range(len(self.path)-1):
                n1 = self.path[i]
                n2 = self.path[i+1]

                if (n1.s <= self.state.s <= n2.s):

                    node_checkpoint = i
                    
                    # if collision point provided, use ensured collision logic
                    if self.collision_point is not None and time_to_collision is not None and collision_segment_prev_node is not None and collision_segment_next_node is not None:
                        # Project collision point to arc lengths
                        diff = np.array(collision_pt) - np.array([collision_segment_prev_node.x,
                                                                  collision_segment_prev_node.y])
                        euclidian_dist = float(np.sqrt(np.sum(diff**2)))
                        collision_pt_s = collision_segment_prev_node.s + euclidian_dist

                        # Distance this oncoming vehicle must travel to the collision point (along s)
                        distance_remaining = collision_pt_s - self.state.s

                        # If using collision point logic with constant speed
                        if not self.released and self.set_speed and self.id != 1:
                            v_set = max(1e-6, self.set_speed / 3.6)  # m/s, avoid divide-by-zero
                            t_oncoming = distance_remaining / v_set
                            
                            if not self.set_constant and time_to_collision is not None:
                                if time_to_collision > 6.0:
                                    self.k, self.c = 4.2, 7.8
                                    self.alpha1 = 0.8
                                elif 3.0 < time_to_collision <= 6.0:
                                    self.k, self.c = 2.2, 7.8
                                    self.alpha1 = 0.6
                                else:
                                    self.k, self.c = 1.0, 7.8
                                    self.alpha1 = 0.9
                                self.set_constant = True
                            
                            # buffer_log = k_log / max(1e-6, math.log(t_oncoming + c_log))
                            buffer_exp = self.k / (t_oncoming**self.alpha1 + self.c)

                            # Also shrink with TTC (fraction of TTC)
                            alpha2 = 0.7
                            buffer_ttc = alpha2 * t_oncoming

                            # Keep it within reasonable bounds (seconds)
                            buffer_min, buffer_max = 0.01, 2.0
                            buffer = min(max(min(buffer_exp, buffer_ttc), buffer_min), buffer_max)

                            ttc_cap = 0.5  # seconds; oncoming cannot leave earlier than this TTC

                            if time_to_collision > ttc_cap:
                                self.state.s_vel = 0.0
                            
                            if time_to_collision <= t_oncoming + buffer:
                                self.state.s_vel = v_set
                                self.released = True
                            else:
                                self.state.s_vel = 0.0

                    #else use actor heading collision logic
                    elif time_to_collision is not None:
                        # Calculate the collision-required speed
                        distance_remaining = np.sqrt(np.sum((collision_pt - np.array([self.state.x, self.state.y])) ** 2))
                        if time_to_collision > 0:
                            collision_required_speed = distance_remaining / time_to_collision
                        else:
                            collision_required_speed = 0.0  # stop either collided or missed collision window
                    
                        # Get the reference speed from the path nodes if speed profile to be used
                        if n1.speed is not None and n2.speed is not None:
                            # Interpolate the reference speed
                            ratio = (self.state.s - n1.s)/(n2.s - n1.s)
                            self.reference_speed = n1.speed + (n2.speed - n1.speed) * ratio
                        else:
                            self.reference_speed = self.set_speed if self.set_speed else 0.0

                        # Apply speed qualifier logic
                        if self.speed_qualifier:
                            if self.speed_qualifier == SpeedQualifier.CONSTANT:
                                # Use reference speed regardless of collision requirements
                                self.state.s_vel = self.reference_speed
                            elif self.speed_qualifier == SpeedQualifier.MAXIMUM:
                                # Reference speed is upper bound, use minimum of reference and collision-required
                                # if collision_required_speed > 0:
                                    self.state.s_vel = min(self.reference_speed, collision_required_speed)
                                # elif collision_required_speed <= 0:
                                #     self.state.s_vel = collision_required_speed
                            elif self.speed_qualifier == SpeedQualifier.MINIMUM:
                                # Reference speed is lower bound, use maximum of reference and collision-required
                                self.state.s_vel = max(self.reference_speed, collision_required_speed)
                            elif self.speed_qualifier == SpeedQualifier.INITIAL:
                                # Use collision-required speed, allowing realistic adjustments
                                self.state.s_vel = collision_required_speed

                    # Else just follow speed profile or given speed
                    # For now we assume that the velocity is specified at each path point or none of them
                    # Later we could instead interpolate between points with speed specified
                    elif n1.speed is not None and n2.speed is not None:
                        # Interpolate the velocity
                        ratio = (self.state.s - n1.s)/(n2.s - n1.s)
                        self.state.s_vel = n1.speed + (n2.speed - n1.speed) * ratio
                    break

            # Calculate frenet position
            self.state.s += (self.state.s_vel * delta_time)

            # Now calculate the cartesian state from the frenet state
            for i in range(node_checkpoint, len(self.path)-1):
                n1 = self.path[i]
                n2 = self.path[i+1]
                if (n1.s <= self.state.s <= n2.s):
                    dx = n2.x - n1.x
                    dy = n2.y - n1.y
                    d = n2.s - n1.s
                    # Calculate position
                    ratio = (self.state.s - n1.s)/(d)
                    self.state.x = n1.x + (dx) * ratio
                    self.state.y = n1.y + (dy) * ratio

                    # Calculate velocity vector
                    self.state.x_vel = self.state.s_vel * dx/d
                    self.state.y_vel = self.state.s_vel * dy/d

                    # Calculate yaw
                    self.state.yaw = math.degrees(math.atan2(dy, dx))
                    break

            # Reached the end of the path
            if self.state.s > self.path[-1].s:
                self.force_stop()
                if not self.keep_active:
                    self.state.set_X([-9999, 0, 0])
                    self.state.set_Y([-9999,0,0])
                    if self.sim_state is ActorSimState.ACTIVE:
                        self.remove()

@dataclass
class TrajNode:
    x:float = 0.0
    y:float = 0.0
    x_vel = None  # calculated as dx/dt, the first node same as the second
    y_vel = None  # calculated as dy/dt, the first node same as the second
    time:float = 0.0
    yaw:float = 0.0

@dataclass
class PathNode:
    x:float = 0.0      # [m]
    y:float = 0.0      # [m]
    s:float = 0.0      # [m]
    speed:float = 0.0  # [m/s]

@dataclass
class ActorState:
    #sim frame
    x:float = 0.0
    x_vel:float = 0.0
    x_acc:float = 0.0

    y:float = 0.0
    y_vel:float = 0.0
    y_acc:float = 0.0

    z:float = 0.0
    z_vel:float = 0.0
    z_acc:float = 0.0

    #frenet state
    s:float = 0.0
    s_vel:float = 0.0
    s_acc:float = 0.0
    d:float = 0.0
    d_vel:float = 0.0
    d_acc:float = 0.0

    #the direction the actor is facing
    yaw:float = 0.0

    #For easy shared memory parsing
    def get_cart_state_vector(self):
        return [self.x, self.x_vel, self.x_acc,
                self.y,  self.y_vel, self.y_acc]

    def get_frenet_state_vector(self):
        return [self.s, self.s_vel, self.s_acc,
                self.d, self.d_vel, self.d_acc]

    def set_state_vector(self, arr):
        self.set_X(arr[0:3])
        self.set_Y(arr[3:6])
        self.set_S(arr[6:9])
        self.set_D(arr[9:12])
        self.yaw = arr[12]

    #For easy shared memory parsing
    def get_state_vector(self):
        combined = self.get_cart_state_vector() + self.get_frenet_state_vector() + [self.yaw]
        return combined

    def get_X(self):
        return [self.x,self.x_vel,self.x_acc]

    def get_Y(self):
        return [self.y,self.y_vel,self.y_acc]

    def set_X(self, X):
        self.x, self.x_vel, self.x_acc = X

    def set_Y(self, Y):
        self.y, self.y_vel, self.y_acc = Y

    def get_S(self):
        return [self.s, self.s_vel, self.s_acc]

    def set_S(self, S):
        self.s, self.s_vel, self.s_acc = S

    def get_D(self):
        return [self.d, self.d_vel, self.d_acc]

    def set_D(self, D):
        self.d, self.d_vel, self.d_acc = D

    def get_cartesian_speed(self):
        return math.sqrt(abs(self.x_vel)**2 + abs(self.y_vel)**2)

@dataclass
class VehicleState(ActorState):
   steer:float = 0.0

@dataclass
class PedestrianState(ActorState):
   pass

@dataclass
class StaticObject():
    id:int = 0
    x:float = 0.0
    y:float = 0.0
    s:float = 0.0
    d:float = 0.0
    model:str = ''
#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#slarter@uwaterloo.ca
# --------------------------------------------
# SIMULATED PEDESTRIANS
# --------------------------------------------
import numpy as np
import random
import glog as log
from SimConfig import *
from util.Utils import *
from Actor import *
from sp.SPPlannerState import *
from shm.SimSharedMemory import *
from util.Utils import kalman
from util.Transformations import normalize
from SimTraffic import *
from sp.btree.BehaviorModels import BehaviorModels
from sp.ManeuverModels import *
import lanelet2.core


# Base class for Pedestrians
class Pedestrian(Actor):
    #pedestrian types
    N_TYPE = 0
    TP_TYPE = 1
    PP_TYPE = 2
    EP_TYPE = 3
    SP_TYPE = 4

    PEDESTRIAN_RADIUS = 0.2

    def __init__(self, id, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0]):
        super().__init__(id, name, start_state)
        self.type = Pedestrian.N_TYPE
        self.radius = Pedestrian.PEDESTRIAN_RADIUS


    def update_sim_state(self, new_state, delta_time):
        # only be done for remote pedestrians (which don't have a frenet state)
        if self.type is not Pedestrian.EP_TYPE:
            log.warn("Cannot update sim state for pedestrians directly")


    def get_sim_state(self):
        x = round((self.state.x * CLIENT_METER_UNIT))
        y = round((self.state.y * CLIENT_METER_UNIT))
        z = 0.0
        position = [x, y, z]
        velocity = [self.state.x_vel, self.state.y_vel]
        return self.id, self.type, position, velocity, self.state.yaw


class TP(Pedestrian):
    """
    A trajectory following pedestrian.
    @param keep_active: If True, pedestrian stays in simulation even when is not following a trajectory
    """
    def __init__(self, id, name, start_state, trajectory, keep_active = True):
        super().__init__(id, name, start_state)
        self.type = Pedestrian.TP_TYPE
        self.trajectory = trajectory
        self.keep_active = keep_active
        if not keep_active:
            #starts as inactive until trajectory begins
            self.sim_state = ActorSimState.INACTIVE
            self.state.set_X([9999, 0, 0]) #forcing
            self.state.set_Y([9999,0,0])

    def tick(self, tick_count, delta_time, sim_time):
        Pedestrian.tick(self, tick_count, delta_time, sim_time)
        self.follow_trajectory(sim_time, self.trajectory)


class SP(Pedestrian):
    """
    A pedestrian following a dynamic behavior model where parameters
    of the Social Force Model (SFM) are informed by behaviour trees
    """

    def __init__(self, id, name, start_state, destinations, root_btree_name, btree_locations=[], btype=""):
        super().__init__(id, name, start_state)
        self.btype = btype
        self.btree_locations = btree_locations
        self.root_btree_name = root_btree_name
        self.btree_reconfig = ""

        self.mconfig = None
        self.type = Pedestrian.SP_TYPE
        self.curr_route_node = 0
        self.destination = np.array(destinations[self.curr_route_node])
        #self.desired_speed = random.uniform(0.8,1.5)
        self.desired_speed = 1.5
        self.mass = random.uniform(50,80)
        self.radius = 1
        self.char_time = random.uniform(8,16) # characteristic time for SFM
        self.bodyFactor = 120000
        self.slideFricFactor = 240000

        self.behavior_model = BehaviorModels(self.id, self.root_btree_name, self.btree_reconfig, self.btree_locations, self.btype)


    def tick(self, tick_count, delta_time, sim_time):
        Pedestrian.tick(self, tick_count, delta_time, sim_time)
        self.update_behavior()
        self.update_position_SFM(np.array([self.state.x, self.state.y]), np.array([self.state.x_vel, self.state.y_vel]))

    def update_behavior(self):
        reg_elems = self.get_reg_elem_states(self.state)

        # Get planner state
        planner_state = PedestrianPlannerState(
                            pedestrian_state=self.state,
                            route=self.sim_config.pedestrian_goal_points[self.id],
                            curr_route_node=self.curr_route_node,
                            traffic_vehicles=self.sim_traffic.vehicles,
                            regulatory_elements=reg_elems,
                            pedestrians=self.sim_traffic.pedestrians,
                            lanelet_map=self.sim_traffic.lanelet_map
                        )

        # BTree Tick
        mconfig, snapshot_tree = self.behavior_model.tick(planner_state)

        # new maneuver
        if self.mconfig and self.mconfig.mkey != mconfig.mkey:
            log.info("PID {} started maneuver {}".format(self.id, mconfig.mkey.name))
            # print sp state and deltas
            state_str = (
                "PID {}:\n"
                "   position    sim=({:.3f},{:.3f})\n"
                "   speed       {:.3f}\n"
            ).format(
                self.id,
                self.state.x, self.state.y,
                np.linalg.norm([self.state.x_vel, self.state.y_vel])
            )
            log.info(state_str)
        self.mconfig = mconfig

        # Maneuver tick
        if mconfig:
            #replan maneuver

            self.curr_route_node, new_route_node, self.desired_speed = plan_maneuver(mconfig.mkey,
                                                                        mconfig,
                                                                        planner_state.pedestrian_state,
                                                                        planner_state.route,
                                                                        planner_state.curr_route_node,
                                                                        planner_state.traffic_vehicles,
                                                                        planner_state.pedestrians)


            if new_route_node != None:
                self.sim_config.pedestrian_goal_points[self.id].insert(self.curr_route_node, new_route_node)


            self.destination = np.array(self.sim_config.pedestrian_goal_points[self.id][self.curr_route_node])


    def get_reg_elem_states(self, pedestrian_state):
        traffic_light_states = {}
        for lid, tl in self.sim_traffic.traffic_lights.items():
            traffic_light_states[lid] = tl.current_color.value

        cur_ll = self.sim_traffic.lanelet_map.get_occupying_lanelet_by_participant(pedestrian_state.x, pedestrian_state.y, "pedestrian")

        if cur_ll == None:
            cur_ll = self.sim_traffic.lanelet_map.get_occupying_lanelet(pedestrian_state.x, pedestrian_state.y)

        # Get regulatory elements acting on this lanelet
        reg_elems = cur_ll.regulatoryElements
        reg_elem_states = []

        for re in reg_elems:
            if isinstance(re, lanelet2.core.TrafficLight):
                # lanelet2 traffic lights must have a corresponding state from the main process
                if re.id not in traffic_light_states:
                    continue

                stop_linestring = re.parameters['ref_line']

                # stop at middle of stop line
                stop_pos = (np.asarray([stop_linestring[0][0].x, stop_linestring[0][0].y]) + np.asarray([stop_linestring[0][-1].x, stop_linestring[0][-1].y])) / 2
                reg_elem_states.append(TrafficLightState(color=traffic_light_states[re.id], stop_position=stop_pos))

        return reg_elem_states


    def update_position_SFM(self, curr_pos, curr_vel):
        '''
        Paper with details on SFM formulas: https://royalsocietypublishing.org/doi/full/10.1098/rspb.2009.0405
        This paper explains the calculations and parameters in other_pedestrian_interaction() and wall_interaction()
        '''
        direction = np.array(normalize(self.destination - curr_pos))
        desired_vel = direction * self.desired_speed

        '''if (self.desired_speed == 0.0):
            curr_vel = 0.0'''

        delta_vel = desired_vel - curr_vel

        # if delta_vel is close enough to zero, assign value zero
        if np.allclose(delta_vel, np.zeros(2)):
            delta_vel = np.zeros(2)

        dt = 1 / TRAFFIC_RATE

        f_other_ped = np.zeros(2)
        f_walls = np.zeros(2)

        # placeholder until I can read border data from traffic
        walls = []

        # attracting force towards destination
        f_adapt = (delta_vel * self.mass) / self.char_time

        # repulsive forces from other pedestrians
        for other_ped in {ped for (pid,ped) in self.sim_traffic.pedestrians.items() if pid != self.id}:
            f_other_ped += self.other_pedestrian_interaction(curr_pos, curr_vel, other_ped)

        # repulsive forces from walls (borders)
        for wall in walls:
            f_walls += self.wall_interaction()


        f_sum = f_adapt + f_other_ped + f_walls

        curr_acc = f_sum / self.mass
        curr_vel += curr_acc*dt
        curr_pos += curr_vel * dt

        self.state.set_X([curr_pos[0], curr_vel[0], curr_acc[0]])
        self.state.set_Y([curr_pos[1], curr_vel[1], curr_acc[1]])

    def other_pedestrian_interaction(self, curr_pos, curr_vel, other_ped, A=4.5, gamma=0.35, n=2.0, n_prime=3.0, lambda_w=2.0, epsilon=0.005):
        '''
        Calculates repulsive forces between pedestrians
        '''
        other_pos = np.array([other_ped.state.x, other_ped.state.y])
        other_vel = np.array([other_ped.state.x_vel, other_ped.state.y_vel])

        eij = normalize(other_pos - curr_pos)
        Dij = lambda_w * (curr_vel - other_vel) + eij
        tij = normalize(Dij)
        nij = np.array([-tij[1], tij[0]])

        dij = np.linalg.norm(curr_pos - other_pos)
        dot_product = max(min(np.dot(tij, eij), 1.0), -1.0) # stay within [-1,1] domain
        theta = np.arccos(dot_product)

        B = gamma * np.linalg.norm(Dij)

        theta += B*epsilon

        fij = -A*np.exp(-dij/B) * (np.exp(-(n_prime*B*theta)**2)*tij + np.exp(-(n*B*theta)**2)*nij)

        return fij

    def wall_interaction(self):
        wij = np.zeros(2)

        return wij

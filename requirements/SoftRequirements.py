import numpy as np
import os

from functools import partial

from requirements.RequirementEvents import SoftRequirement
from sv.SDVTrafficState import TrafficState
from util.Utils import distance_2p

class SoftRequirements:
	def __init__(self, ego_vehicle):
		self.ego_vehicle     = ego_vehicle
		self.evaluation_mode = os.getenv("GSS_EVALUATION_NAME", "") != ""
		self.prev_acc        = np.array([0, 0]) # (s_acc, d_acc) Assuming the vehicle starts standstill

	def all_comparable_conditions(self):
		# Soft Requirements are unimportant unless we evaluate the final policy
		if not self.evaluation_mode:
			return []

		return [
			partial(self.as_comparable_soft_requirement, self.tracking_lateral_position_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_lateral_speed_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_longitudinal_position_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_longitudinal_speed_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_positional_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_speed_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_x_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_xvel_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_xacc_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_y_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_yvel_errors),
			partial(self.as_comparable_soft_requirement, self.tracking_yacc_errors)
		]

	def all_direct_conditions(self):
		# Soft Requirements are unimportant unless we evaluate the final policy
		if not self.evaluation_mode:
			return []

		return [
			partial(self.as_direct_soft_requirement, self.distance_to_closest_obstacle),
			partial(self.as_direct_soft_requirement, self.jerk)
		]

	def as_comparable_soft_requirement(self, getter, ground_truth:TrafficState, perceived:TrafficState):
		agent_id     = self.ego_vehicle.id
		metric_name  = getter.__name__
		metric_value = getter(ground_truth, perceived)

		self.ego_vehicle.events_queue.put(SoftRequirement(agent_id, metric_name, metric_value))


	def as_direct_soft_requirement(self, getter, traffic_state:TrafficState):
		agent_id     = self.ego_vehicle.id
		metric_name  = getter.__name__
		metric_value = getter(traffic_state)

		self.ego_vehicle.events_queue.put(SoftRequirement(agent_id, metric_name, metric_value))

	def distance_to_closest_obstacle(self, traffic_state:TrafficState):
		actor     = traffic_state.road_occupancy.front_center.closest()
		distance  = None
		ego_state = self.ego_vehicle.state

		if actor == None:
			return distance

		# Frenet Frame localizable
		if actor.state.s != 0.0 or actor.state.d != 0.0:
			distance = actor.state.s - ego_state.s

		# Euclidian localizable
		else:
			distance = distance_2p(actor.state.x, actor.state.y, ego_state.x, ego_state.y)

		return distance

	def jerk(self, traffic_state:TrafficState):
		curr_acc      = np.array([ self.ego_vehicle.state.s_acc, self.ego_vehicle.state.d_acc ])
		jerk_value    = np.linalg.norm(curr_acc - self.prev_acc)
		self.prev_acc = curr_acc
		
		return jerk_value

	def split_vehicles(self, ground_truth:TrafficState, perceived:TrafficState):
		sensed_vehicles = {}
		sensed_vehicles.update(perceived.traffic_vehicles)
		sensed_vehicles.update(perceived.traffic_vehicles_orp)

		truth_vehicles = {}
		truth_vehicles.update(ground_truth.traffic_vehicles)
		truth_vehicles.update(ground_truth.traffic_vehicles_orp)

		return truth_vehicles, sensed_vehicles

	def tracking_lateral_position_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		getter = lambda vehicle: vehicle.state.d
		return self.tracking_frenet_errors(getter, ground_truth, perceived)

	def tracking_lateral_speed_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		getter = lambda vehicle: vehicle.state.d_vel
		return self.tracking_frenet_errors(getter, ground_truth, perceived)

	def tracking_longitudinal_position_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		getter = lambda vehicle: vehicle.state.s
		return self.tracking_frenet_errors(getter, ground_truth, perceived)

	def tracking_longitudinal_speed_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		getter = lambda vehicle: vehicle.state.s_vel
		return self.tracking_frenet_errors(getter, ground_truth, perceived)

	def tracking_errors(self, getter, ground_truth:TrafficState, perceived:TrafficState):
		truth_vehicles, sensed_vehicles = self.split_vehicles(ground_truth, perceived)

		tracking_errors = {}

		for vid, vehicle in truth_vehicles.items():
			if vid != self.ego_vehicle.id and vid in sensed_vehicles:
				sensed_vehicle = sensed_vehicles[vid]
				tracking_errors[vid] = getter(vehicle) - getter(sensed_vehicle)

		return tracking_errors

	def tracking_frenet_errors(self, getter, ground_truth:TrafficState, perceived:TrafficState):
		truth_vehicles, sensed_vehicles = self.split_vehicles(ground_truth, perceived)

		frenet_errors = {}

		for vid, vehicle in truth_vehicles.items():
			if vid != self.ego_vehicle.id and vid in sensed_vehicles:
				sensed_vehicle = sensed_vehicles[vid]

				sensed_frenet_vector = np.array(sensed_vehicle.state.get_S() + sensed_vehicle.state.get_D())
				truth_frenet_vector  = np.array(vehicle.state.get_S() + vehicle.state.get_D())

				if np.any(sensed_frenet_vector) and np.any(truth_frenet_vector):
					frenet_errors[vid] = getter(vehicle) - getter(sensed_vehicle)

		return frenet_errors

	def tracking_positional_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		truth_vehicles, sensed_vehicles = self.split_vehicles(ground_truth, perceived)

		positional_errors = {}

		for vid, vehicle in truth_vehicles.items():
			if vid != self.ego_vehicle.id and vid in sensed_vehicles:
				sensed_vehicle = sensed_vehicles[vid]

				sensed_frenet_vector = np.array(sensed_vehicle.state.get_S() + sensed_vehicle.state.get_D())
				truth_frenet_vector  = np.array(vehicle.state.get_S() + vehicle.state.get_D())

				if np.any(sensed_frenet_vector) and np.any(truth_frenet_vector):
					positional_errors[vid] = distance_2p(vehicle.state.s, vehicle.state.d,
						                                 sensed_vehicle.state.s, sensed_vehicle.state.d)
				else:
					positional_errors[vid] = distance_2p(vehicle.state.x, vehicle.state.y, 
						                                 sensed_vehicle.state.x, sensed_vehicle.state.y)

		return positional_errors


	def tracking_speed_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		truth_vehicles, sensed_vehicles = self.split_vehicles(ground_truth, perceived)

		speed_errors = {}

		for vid, vehicle in truth_vehicles.items():
			if vid != self.ego_vehicle.id and vid in sensed_vehicles:
				sensed_vehicle = sensed_vehicles[vid]

				sensed_frenet_vector = np.array(sensed_vehicle.state.get_S() + sensed_vehicle.state.get_D())
				truth_frenet_vector  = np.array(vehicle.state.get_S() + vehicle.state.get_D())

				if np.any(sensed_frenet_vector) and np.any(truth_frenet_vector):
					speed_errors[vid] = distance_2p(vehicle.state.s_vel, vehicle.state.d_vel,
						                            sensed_vehicle.state.s_vel, sensed_vehicle.state.d_vel)
				else:
					speed_errors[vid] = distance_2p(vehicle.state.x_vel, vehicle.state.y_vel, 
						                            sensed_vehicle.state.x_vel, sensed_vehicle.state.y_vel)

		return speed_errors

	def tracking_x_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		getter = lambda vehicle: vehicle.state.x
		return self.tracking_errors(getter, ground_truth, perceived)

	def tracking_xvel_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		getter = lambda vehicle: vehicle.state.x_vel
		return self.tracking_errors(getter, ground_truth, perceived)

	def tracking_xacc_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		getter = lambda vehicle: vehicle.state.x_acc
		return self.tracking_errors(getter, ground_truth, perceived)

	def tracking_y_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		getter = lambda vehicle: vehicle.state.y
		return self.tracking_errors(getter, ground_truth, perceived)

	def tracking_yvel_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		getter = lambda vehicle: vehicle.state.y_vel
		return self.tracking_errors(getter, ground_truth, perceived)

	def tracking_yacc_errors(self, ground_truth:TrafficState, perceived:TrafficState):
		getter = lambda vehicle: vehicle.state.y_acc
		return self.tracking_errors(getter, ground_truth, perceived)
import numpy as np
import os

from functools import partial

from requirements.RequirementViolationEvents import SoftRequirement
from sv.SDVTrafficState import TrafficState
from util.Utils import distance_2p

class SoftRequirements:
	def __init__(self, ego_vehicle):
		self.ego_vehicle = ego_vehicle
		self.prev_acc    = np.array([0, 0]) # (s_acc, d_acc) Assuming the vehicle starts standstill

	def all_conditions(self):
		evaluation_mode = os.getenv("GSS_EVALUATION_NAME", "") != ""

		# Soft Requirements are unimportant unless we evaluate the final policy
		if not evaluation_mode:
			return []

		return [
			partial(self.as_soft_requirement, self.distance_with_closest_obstacle),
			partial(self.as_soft_requirement, self.jerk)
		]


	def as_soft_requirement(self, getter, traffic_state:TrafficState):
		agent_id     = self.ego_vehicle.id
		metric_name  = getter.__name__
		metric_value = getter(traffic_state)

		SoftRequirement(agent_id, metric_name, metric_value)

	def distance_with_closest_obstacle(self, traffic_state:TrafficState):
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

		
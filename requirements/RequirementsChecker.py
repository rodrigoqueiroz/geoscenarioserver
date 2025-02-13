from requirements.RequirementViolationEvents import GoalOvershot, ScenarioCompletion, ScenarioEnd
from sv.SDVTrafficState import *
from sv.ruleEngine.constants import *

class RequirementsChecker:
	def __init__(self, goal_ends_simulation):
		self.conditions = [
			self.detect_goal_overshot
		]

		self.goal_ends_simulation = goal_ends_simulation
		self.goal_s_distance = float('inf')

	def analyze(self, traffic_state:TrafficState):
		for condition in self.conditions:
			condition(traffic_state)


	def detect_goal_overshot(self, traffic_state:TrafficState):
		""" Checks if the vehicle has reached or passed the goal point in the frenet frame.
		"""

		if traffic_state.goal_point_frenet is None:
			return

		ego_s = traffic_state.vehicle_state.s
		ego_d = traffic_state.vehicle_state.d

		# max meters away from target
		goal_s_distance = 4.0
		goal_d_distance = 2.0

		goal_s = traffic_state.goal_point_frenet[0]
		goal_d = traffic_state.goal_point_frenet[1]

		s_convergence = abs(goal_s - ego_s)
		d_convergence = abs(goal_d - ego_d)

		if traffic_state.route_complete and \
		   s_convergence < goal_s_distance and \
		   d_convergence < goal_d_distance:

		   # Let the vehicle get closer to its destination...
			if s_convergence < self.goal_s_distance:
				self.goal_s_distance = s_convergence
				return

			if traffic_state.vehicle_state.s_vel > 1.0:
				GoalOvershot(traffic_state.vid)
			
			if self.goal_ends_simulation:
				ScenarioEnd()
				raise ScenarioCompletion()
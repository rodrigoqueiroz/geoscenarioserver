import numpy as np

from requirements.ExplicitRequirements import ExplicitRequirements
from requirements.ImplicitRequirements import ImplicitRequirements
from sv.SDVTrafficState import TrafficState

class RequirementsChecker(ExplicitRequirements, ImplicitRequirements):
	def __init__(self, ego_vehicle, goal_ends_simulation):
		ExplicitRequirements.__init__(self, ego_vehicle, goal_ends_simulation)
		ImplicitRequirements.__init__(self, ego_vehicle)

		self.conditions = ExplicitRequirements.all_conditions(self) +\
			              ImplicitRequirements.all_conditions(self)


	def analyze(self, traffic_state:TrafficState):
		for condition in self.conditions:
			condition(traffic_state)
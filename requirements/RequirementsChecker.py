from requirements.HardRequirements import HardRequirements
from requirements.SoftRequirements import SoftRequirements
from sv.SDVTrafficState import TrafficState

class RequirementsChecker(HardRequirements, SoftRequirements):
	def __init__(self, ego_vehicle, goal_ends_simulation):
		HardRequirements.__init__(self, ego_vehicle, goal_ends_simulation)
		SoftRequirements.__init__(self, ego_vehicle)

		self.conditions = HardRequirements.all_conditions(self) +\
			              SoftRequirements.all_conditions(self)


	def analyze(self, traffic_state:TrafficState):
		for condition in self.conditions:
			condition(traffic_state)
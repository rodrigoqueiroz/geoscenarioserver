from requirements.HardRequirements import HardRequirements
from requirements.SoftRequirements import SoftRequirements
from sv.SDVTrafficState import TrafficState

class RequirementsChecker(HardRequirements, SoftRequirements):
	def __init__(self, ego_vehicle, goal_ends_simulation):
		HardRequirements.__init__(self, ego_vehicle, goal_ends_simulation)
		SoftRequirements.__init__(self, ego_vehicle)

		self.comparable_conditions = HardRequirements.all_comparable_conditions(self) +\
		                             SoftRequirements.all_comparable_conditions(self)
		
		self.direct_conditions     = HardRequirements.all_direct_conditions(self) +\
			                         SoftRequirements.all_direct_conditions(self)


	def analyze(self, traffic_state:TrafficState):
		for condition in self.direct_conditions:
			condition(traffic_state)

	def compare(self, ground_truth:TrafficState, perceived:TrafficState):
		for condition in self.comparable_conditions:
			condition(ground_truth, perceived)

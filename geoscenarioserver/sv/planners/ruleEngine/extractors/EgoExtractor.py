from geoscenarioserver.sv.planners.ruleEngine.Extractor import Extractor

class EgoExtractor(Extractor):
	def __init__(self):
		self.name = "ego"

	def generate_features(self, traffic_state):
		return {
			"speed": traffic_state.vehicle_state.s
		}
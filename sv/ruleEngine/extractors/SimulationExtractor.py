from sv.ruleEngine.Extractor import Extractor

class SimulationExtractor(Extractor):
	def __init__(self):
		self.iteration = 0
		self.name = "simulation"

	def generate_features(self, traffic_state):
		self.iteration += 1
		
		return {
            "sim_time": traffic_state.sim_time,
            "tick":     self.iteration
        }
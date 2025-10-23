from geoscenarioserver.sv.planners.ruleEngine.extractors.ActorExtractor      import ActorExtractor
from geoscenarioserver.sv.planners.ruleEngine.extractors.EgoExtractor        import EgoExtractor
from geoscenarioserver.sv.planners.ruleEngine.extractors.SimulationExtractor import SimulationExtractor
from geoscenarioserver.sv.planners.ruleEngine.extractors.TravelExtractor     import TravelExtractor

class FeatureGenerator:
	def __init__(self):
		self.extractors = [
			ActorExtractor("vehicles"),
			EgoExtractor(),
			SimulationExtractor(),
			TravelExtractor()
		]

	def parse(self, traffic_state):
		situation = {}

		for extractor in self.extractors:
			situation[extractor.name] = extractor.generate_features(traffic_state)

		#print('\nsituation', situation, '\n')

		return situation

from sv.ruleEngine.extractors.ActorExtractor      import ActorExtractor
from sv.ruleEngine.extractors.EgoExtractor        import EgoExtractor
from sv.ruleEngine.extractors.SimulationExtractor import SimulationExtractor
from sv.ruleEngine.extractors.TravelExtractor     import TravelExtractor

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

		return situation

from sv.ruleEngine.Extractor import Extractor

class EgoExtractor(Extractor):
	def __init__(self):
		self.name = "ego"

	def generate_features(self, traffic_state):
		return {}
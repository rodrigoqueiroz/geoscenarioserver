class Extractor:
	def flatten(self, props, keys):
		if props == None or len(props) != len(keys):
			return None

		my_props = {}

		for index, key in enumerate(keys):
			my_props[key] = props[index]

		return my_props

	def to_cartesian_space(self, props):
		return self.flatten(props, keys=['x', 'y'])

	def to_frenet_frame(self, props):
		return self.flatten(props, keys=['s', 'd'])
import json
import os
import time

from multiprocessing import Process, Value

# Global Variables
file_name_folder = os.getenv("GSS_OUTPUTS", os.path.join(os.getcwd(), "outputs"))
file_name_hard   = os.path.join(file_name_folder, "violations.json")
file_name_soft   = os.getenv("GSS_EVALUATION_NAME", "")

class RequirementEventsParser(object):
	def __init__(self, events_queue):
		self.events_queue       = events_queue
		self.ready_to_terminate = Value('b', False)
		self.ticks_required_without_overlaping_this_actor = 7

		if os.path.exists(file_name_hard):
			os.remove(file_name_hard)

	def start(self):
		""" Start the requirement violation events in a subprocess """
		self._process = Process(target=self.run_violations_process,
								args=[self.events_queue],
								daemon=True)
		self._process.start()

	def run_violations_process(self, events_queue):
		self.agent_collisions  = {}
		self.agent_finished    = 0
		self.agent_ticks       = {}
		self.soft_requirements = {}
		self.number_of_agents  = 0
		self.hard_requirements = {}
		self.simulation_time   = 0.0

		while True:
			event      = events_queue.get() # When an event has been received, parse it
			event_name = event.__class__.__name__

			if event_name == "AgentTick":
				self.parse_agent_tick(event.agent_id)

			elif event_name in [ "CollisionWithPedestrian", "CollisionWithVehicle" ]:
				self.parse_collision(event_name, event.agent_id, event.payload, event.uid)

			elif event_name == "ElapsedTime":
				self.simulation_time += event.elapsed_time

			elif event_name == "ScenarioEnd":
				self.parse_termination(event.agent_id)

			elif event_name == "ScenarioTimeout":
				self.parse_hard_requirements_iterable(event_name, event.prior, event.posterior, True)

			elif event_name == "SoftRequirement":
				self.parse_soft_requirements(event.agent_id, event.req_name, event.req_value)

			else:
				self.parse_hard_requirements(event_name, event.agent_id, event.payload)

				if event_name in [ "BrokenScenario", "ScenarioInterrupted" ]:
					self.parse_termination(event.agent_id)

	def parse_agent_tick(self, agent_id):
		if agent_id not in self.agent_ticks:
			self.agent_collisions[agent_id]  = {}
			self.agent_ticks[agent_id]       = 0
			self.hard_requirements[agent_id] = {}
			self.number_of_agents           += 1

		self.agent_ticks[agent_id] += 1

	def parse_collision(self, event_name, agent_id, payload, uid):
		agent_tick      = self.agent_ticks[agent_id]
		collision_state = self.agent_collisions[agent_id]

		# There must be a gap of n ticks without collision between collision with the same agent
		if uid not in collision_state or agent_tick - self.ticks_required_without_overlaping_this_actor > collision_state[uid]:
			self.parse_hard_requirements(event_name, agent_id, payload)

		self.agent_collisions[agent_id][uid] = agent_tick


	def parse_hard_requirements(self, event_name, agent_id, payload):
		if agent_id not in self.agent_ticks:
			raise Exception("Hard requirement unmet before the first tick of an agent...")
		
		agent_tick = self.agent_ticks[agent_id]

		if agent_id not in self.hard_requirements:
			self.hard_requirements[agent_id] = {}

		if agent_tick not in self.hard_requirements[agent_id]:
			self.hard_requirements[agent_id][agent_tick] = {}

		self.hard_requirements[agent_id][agent_tick][event_name] = payload

	def parse_hard_requirements_iterable(self, event_name, event_prior, event_posterior, must_terminate):
		for agent_id in self.agent_ticks:
			payload = {
				'message': event_prior + str(agent_id) + event_posterior
			}
			self.parse_hard_requirements(event_name, agent_id, payload)

			if must_terminate:
				self.parse_termination(agent_id)

	def parse_soft_requirements(self, agent_id, req_name, req_value):
		if agent_id not in self.soft_requirements:
			self.soft_requirements[agent_id] = {}

		if req_name not in self.soft_requirements[agent_id]:
			self.soft_requirements[agent_id][req_name] = []

		self.soft_requirements[agent_id][req_name].append(req_value)

	def parse_termination(self, agent_id):
		print('Scenario Ended in {} seconds'.format(self.simulation_time))
		self.agent_finished += 1

		if file_name_soft != "":				
			if agent_id not in self.soft_requirements:
				self.soft_requirements[agent_id] = {}

			self.soft_requirements[agent_id]['scenario_completion'] = [ self.simulation_time ]

		# Only write the reports once all monitored vehicles called their scenario end		
		if self.agent_finished == self.number_of_agents:
			self.write_files()

	def stop(self):
		# Wait for the violation report to be written
		while not self.ready_to_terminate.value:
			print('Attempting to close RequirementEventsParser, but report not yet written...')
			time.sleep(1)

		self._process.terminate()

	def write_file(self, file_name, content):
		# Clean File
		if os.path.exists(file_name):
			os.remove(file_name)

		# Write File
		os.makedirs(os.path.dirname(file_name), exist_ok=True)
		with open(file_name, "w+") as file:
			file.write(content)
			file.flush()

	def write_files(self):
		# Hard Requirements Report
		self.write_file(file_name_hard, json.dumps(self.hard_requirements))

		# Custom Requirements Reports
		if file_name_soft != "":

			# Soft Requirements Report
			file_name = os.path.join(file_name_folder, file_name_soft + ".json")
			self.write_file(file_name, json.dumps(self.soft_requirements))

			# Hard Requirements Report
			file_name = os.path.join(file_name_folder, file_name_soft + "__violations.json")
			self.write_file(file_name, json.dumps(self.hard_requirements))

		self.ready_to_terminate.value = True
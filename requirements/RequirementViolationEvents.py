import json
import os

from multiprocessing import Manager, Value

from SimConfig import TRAFFIC_RATE

manager = Manager()

# Singleton
agent_collisions = None
agent_finished   = None
agent_ticks      = None
global_tick      = None
metrics          = None
number_of_agents = None
violations       = None

# Global Variables
file_name_folder = os.getenv("GSS_OUTPUTS", os.path.join(os.getcwd(), "outputs"))
file_name_hard   = os.path.join(file_name_folder, "violations.json")
file_name_soft   = os.getenv("GSS_EVALUATION_NAME", "")

# Constants
TICKS_REQUIRED_WITHOUT_OVERLAPING_THIS_ACTOR = 7

# Generic
class InstanciateOnThread:
	def __init__(self):
		global agent_collisions
		global agent_finished
		global agent_ticks
		global global_tick
		global metrics
		global number_of_agents
		global violations

		agent_collisions = manager.dict()
		agent_finished   = Value('i', 0)
		agent_ticks      = manager.dict()
		global_tick      = Value('i', -1)
		metrics          = manager.dict()
		number_of_agents = Value('i', 0)
		violations       = manager.dict()

class SoftRequirement:
	def __init__(self, agent_id, metric_name, metric_value):
		global metrics

		updated_agent = {}

		if agent_id in metrics:
			updated_agent = metrics[agent_id]

		if metric_name not in updated_agent:
			updated_agent[metric_name] = []

		updated_agent[metric_name].append(metric_value)

		# Required because of manager.dict() does not allow nested update
		metrics[agent_id] = updated_agent


class ScenarioEnd:
	def __init__(self, agent_id):
		global agent_finished
		global metrics
		global number_of_agents

		scenario_completion = global_tick.value / TRAFFIC_RATE
		print('Scenario Ended in {} seconds'.format(scenario_completion))

		if file_name_soft != "":
			# This copy is necessary because manager.dict are awefully managing an object state
			metrics_copy = metrics.copy()
			updated_agent = {}
				
			if agent_id in metrics_copy:
				updated_agent = metrics_copy[agent_id]

			updated_agent['scenario_completion'] = [ scenario_completion ]

			# Required because of manager.dict() does not allow nested update
			metrics[agent_id] = updated_agent

		# Only write the reports once all monitored vehicles called their scenario end
		with agent_finished.get_lock():
			agent_finished.value += 1

			with number_of_agents.get_lock():
				if agent_finished.value == number_of_agents.value:
					self.write_files()

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
		global metrics
		global violations

		violations_copy = violations.copy()
		print('violations', violations_copy)

		# Hard Requirements Report
		self.write_file(file_name_hard, json.dumps(violations_copy))

		# Custom Requirements Reports
		if file_name_soft != "":

			# Soft Requirements Report
			file_name = os.path.join(file_name_folder, file_name_soft + ".json")
			self.write_file(file_name, json.dumps(metrics.copy()))

			# Hard Requirements Report
			file_name = os.path.join(file_name_folder, file_name_soft + "__violations.json")
			self.write_file(file_name, json.dumps(violations_copy))


class UnmetRequirement:
	def raise_it(self, agent_id, payload):
		global agent_ticks
		global violations

		agent_logs = violations[agent_id]
		agent_tick = agent_ticks[agent_id]

		if agent_tick not in agent_logs:
			agent_logs[agent_tick] = {}

		agent_logs[agent_tick][self.__class__.__name__] = payload
		
		# That reassignment is necessary for the update to work in multiprocessing
		violations[agent_id] = agent_logs
		

# Specific
class AgentTick:
	def __init__(self, agent_id):
		global agent_collisions
		global agent_ticks
		global number_of_agents
		global violations

		if agent_id not in agent_ticks:
			agent_collisions[agent_id] = {}
			agent_ticks[agent_id]      = -1
			violations[agent_id]       = {}

			with number_of_agents.get_lock():
				number_of_agents.value += 1

		agent_ticks[agent_id] += 1

class BrokenScenario(UnmetRequirement):
	def __init__(self, agent_id, message):
		self.raise_it(agent_id, {
			'message': 'v' + str(agent_id) + ' ' + message
		})
		ScenarioEnd(agent_id)

class CollisionWithPedestrian(UnmetRequirement):
	def __init__(self, agent_id, pid, collision_zone, relative_angle):
		global agent_collisions
		global agent_ticks

		collision_state = agent_collisions[agent_id]

		if pid not in collision_state or agent_ticks[agent_id] - TICKS_REQUIRED_WITHOUT_OVERLAPING_THIS_ACTOR > collision_state[pid]:
			alias_name  = 'p_' + str(pid)
			myself_name = 'v_' + str(agent_id)

			self.raise_it(agent_id, {
				'angle': relative_angle,
				'collections': {
					'pedestrians': {
						'alias': alias_name,
						'id':    pid,
					}
				},
				'message': f'{myself_name} bounding box overlapped with the pedestrian agent {alias_name} on the vehicle {collision_zone} side at {relative_angle} degrees',
				'zone' : collision_zone
			})

		collision_state[pid] = agent_ticks[agent_id]

		# That reassignment is necessary for the update to work in multiprocessing
		agent_collisions[agent_id] = collision_state

class CollisionWithVehicle(UnmetRequirement):
	def __init__(self, agent_id, vid):
		global agent_collisions
		global agent_ticks

		collision_state = agent_collisions[agent_id]

		# There must be a gap of 5 ticks without collision between collision with the same agent
		if vid not in collision_state or agent_ticks[agent_id] - TICKS_REQUIRED_WITHOUT_OVERLAPING_THIS_ACTOR > collision_state[vid]:
			alias_name  = 'v_' + str(vid)
			myself_name = 'v_' + str(agent_id)

			self.raise_it(agent_id, {
				'collections': {
					'vehicles': {
						'alias': alias_name,
						'id':    vid
					}
				},
				'message': myself_name + ' bounding box overlapped with the vehicle agent ' + alias_name
			})

		collision_state[vid] = agent_ticks[agent_id]

		# That reassignment is necessary for the update to work in multiprocessing
		agent_collisions[agent_id] = collision_state


class GlobalTick:
	def __init__(self):
		global global_tick
		
		with global_tick.get_lock():
			global_tick.value += 1


class GoalOvershot(UnmetRequirement):
	def __init__(self, agent_id):
		self.raise_it(agent_id, {
			'message': 'v' + str(agent_id) + ' drove past its target location'
		})

class ScenarioCompletion(Exception):
    pass


class ScenarioInterrupted(UnmetRequirement):
	def __init__(self, agent_id):
		self.raise_it(agent_id, {
			'agentId': agent_id,
			'message': 'v' + str(agent_id) + ' scenario interrupted by the user'
		})
		ScenarioEnd(agent_id)
		

class ScenarioTimeout(UnmetRequirement):
	def __init__(self, timeout):
		for agent_id in agent_ticks:
			self.raise_it(agent_id, {
				'message': 'v' + str(agent_id) + ' did not reach its target location during the ' 
				    		   + str(timeout) + ' seconds allowed by this scenario.'
			})
			ScenarioEnd(agent_id)


# Autoclean to avoid learning on a broken stack
if os.path.exists(file_name_hard):
	os.remove(file_name_hard)
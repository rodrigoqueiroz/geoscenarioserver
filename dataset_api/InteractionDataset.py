# %%
import sys
sys.path.append('..')
import numpy as np
import pandas as pd
from logging import error, warning
from gsw.GSWriter import GSWriter
from constants import CONSTANTS as CONST

# %% [markdown]
# # InteractionDatasetLoader
# ## Task breakdown
# - load data from the dataset
# 

# %%
dataset_config = {}
dataset_config["base_dir"] = CONST.DATASET_BASE_DIR
dataset_config["version"] = CONST.DATASET_VERSION
dataset_config["recording_map"] = CONST.RECORDING_MAP

# %%
class InteractionDataset():
   

    def __init__(self, config, train=True):
        self.config = config
        self.train = train
        
        if self.train:
            self.base_dir = config["base_dir"]
            self.recording_map = config["recording_map"]
            self.csv_dir_list = [self.get_track_directory(recording) for ind, recording in self.recording_map.items()]
            self.osm_dir_list = [self.get_map_directory(recording) for ind, recording in self.recording_map.items()]
            
        else:
            self.base_dir = config["base_dir"]
            self.recording_map = config["recording_map"]
            self.csv_dir_list = [self.get_track_directory(recording) for ind, recording in self.recording_map.items()]
            self.osm_dir_list = [self.get_map_directory(recording) for ind, recording in self.recording_map.items()]
    
    def get_track_directory(self, recording):
        return self.base_dir+'train/'+recording+'_train.csv' if self.train else self.base_dir+'val/'+recording+'_val.csv'
        
    def get_map_directory(self, recording):
        return self.base_dir+'maps/'+recording+'.osm'
            
    def get_recording_df(self, recording):
        assert recording in self.recording_map.values()
        return pd.read_csv(self.get_track_directory(recording))
    
    def get_scenario_df_of_recording(self, scenario_id, recording_df):
        scenario_id_list = recording_df['case_id'].unique()
        assert scenario_id in scenario_id_list
        return recording_df.loc[recording_df['case_id'] == scenario_id]

# %%
class ScenarioData:
    def __init__(self, scenario_df) -> None:
        self.scenario_df = scenario_df
        self.agent_num = self.get_num_of_agent()
        self.duration = self.get_total_time()
        self.vehicle_df_dict = self.get_vehicle_df_dict()
        self.inclusive_timestamps = self.get_inclusive_timestamps()
        
    # get the number of agent in the scenario dataframe
    def get_num_of_agent(self):
        return max(self.scenario_df['track_id'].unique())

    # get the starting and ending timestamps that includes all the agent in the scenario dataframe
    def get_inclusive_timestamps(self):
        min_timestamp = 0
        max_timestamp = self.duration
        for vid, vehicle_df in self.vehicle_df_dict.items():
            vehicle_min_timestamp = min(vehicle_df['timestamp_ms'])
            vehicle_max_timestamp = max(vehicle_df['timestamp_ms'])
            if vehicle_min_timestamp > min_timestamp:
                min_timestamp = vehicle_min_timestamp
            if vehicle_max_timestamp < max_timestamp:
                max_timestamp = vehicle_max_timestamp
        return [min_timestamp, max_timestamp]

    # get the total time of the scenario
    def get_total_time(self):
        return max(self.scenario_df['timestamp_ms'])

    # get the starting and ending timestamps that includes all the agent in the scenario dataframe
    def get_vehicle_df_dict(self):
        vehicle_df_dict = {}
        for i in range(1,self.agent_num+1):
            vehicle_df_dict[i] = self.scenario_df.loc[self.scenario_df['track_id'] == i]
        return vehicle_df_dict

    def construct_gs_file(self, file_name, start_sim_time_ms):
        # const variable. Distance is measured in meters.
        origin = [0,0]
#         origin_icon_position = [0,0]
#         globalconfig_icon_position = [1,1]
        scenario_name = 'default scenario name'
        collision = True
        gsw = GSWriter()
        try:
            if start_sim_time_ms < self.inclusive_timestamps[0]:
                raise Exception
        except Exception:
            warning('One or more vehicles are out of recording range under the starting simulation time.')
        
        try: 
            if start_sim_time_ms >= self.inclusive_timestamps[1]:
                raise Exception
        except Exception:
            error('Starting simulation time is out of recording range.')
            return None
        
        ending_sim_time_ms = self.inclusive_timestamps[1]
        timeout_ms = self.inclusive_timestamps[1] - start_sim_time_ms

        gsw.addGlobalConfig(CONST.GC_ICON_LAT, # global configuration icon position
                            CONST.GC_ICON_LON, 
                            scenario_name,
                            CONST.MAP_DIR, # lanelet map directory
                            CONST.COLLISION, # collision
                            timeout_ms/1000.0)
        for vid, vehicle_df in self.vehicle_df_dict.items():

            # get the vehicle type
            try:
                behavior_type = CONST.VEHICLE_TYPE_DICT[vid]
            except IndexError:
                error('The vehicle type is not defined in the configuration file.')
                return None
            
            if behavior_type == 'SDV':
                ######################################
                # Format of the vehicle state in data frame: 
                # List[case_id, track_id, frame_id, timestamp_ms, agent_type, x, y, vx, vy, psi_rad, length, width]
                ######################################
                # specify the starting and ending state
                starting_state = vehicle_df.loc[vehicle_df['timestamp_ms'] == start_sim_time_ms].values[0].tolist()
                ending_state = vehicle_df.loc[vehicle_df['timestamp_ms'] == ending_sim_time_ms].values[0].tolist()
                # extract the stating and ending positing in cartisian coordinate
                x_0, y_0, x_f, y_f = starting_state[5], starting_state[6], ending_state[5], ending_state[6]
                print('x_0, y_0, x_f, y_f: ', x_0, y_0, x_f, y_f)
                vx_0, vy_0 = starting_state[7], starting_state[8]
                # the altitude is set to be the same, thus we don't need to consider using it.
                lat_0, lon_0, alt = gsw.m2ll(x_0, y_0)
                lat_f, lon_f, alt = gsw.m2ll(x_f, y_f)
                yaw_0_deg = -1*np.rad2deg(starting_state[9])
                gsw.addSDV(
                    vehicle_name = 'v'+str(vid),
                    route_name = 'v'+str(vid)+'_route',
                    starting_yaw_deg = yaw_0_deg,
                    starting_vx = vx_0, starting_vy = vy_0,
                    starting_ax = 0, starting_ay = 0,
                    trajectory_lat = [lat_0, lat_f], trajectory_lon = [lon_0, lon_f],
                    icon_lat = lat_0, icon_lon = lon_0,
                    vehicle_id = vid,
                    behavior_tree_dir = 'drive.btree')
                
            elif behavior_type == 'TV':
                # Get the vehicle trajectory in the data frame
                inclusive_vehicle_df = vehicle_df.loc[(vehicle_df['timestamp_ms'] >= start_sim_time_ms) & (vehicle_df['timestamp_ms'] <= ending_sim_time_ms)]
                # extract the positing, in cartisian coordinate
                traj_x, traj_y = inclusive_vehicle_df['x'].tolist(), inclusive_vehicle_df['y'].tolist()
                # transform the positing to lat and lon
                traj_lat, traj_lon, alt = gsw.m2ll(traj_x, traj_y)
                # extract the time stamp in seconds, starting at 0
                traj_t = (inclusive_vehicle_df['timestamp_ms'] - start_sim_time_ms)/1000.0
                # extract the speed in m/s
                traj_v = np.sqrt(inclusive_vehicle_df['vx']**2 + inclusive_vehicle_df['vy']**2).tolist()
                # extract the yaw angle and convert it to degree
                traj_yaw_deg = (-1*np.rad2deg(inclusive_vehicle_df['psi_rad'])).tolist()
                gsw.addTV(
                    vehicle_name = 'v'+str(vid),
                    trajectory_name = 'v'+str(vid)+'_trajectory',
                    starting_yaw_deg = traj_yaw_deg,
                    trajectory_lat = traj_lat,
                    trajectory_lon = traj_lon,
                    trajectory_yaw= traj_yaw_deg,
                    trajectory_timestamp= traj_t,
                    trajectory_speed= traj_v,
                    icon_lat = traj_lat[0], icon_lon = traj_lon[0],
                    vehicle_id = vid
                )
                
        gsw.writeOSM(file_name)

# %%
ID = InteractionDataset(dataset_config, train=False)
recording_df = ID.get_recording_df(CONST.RECORDING)
scenario_df = ID.get_scenario_df_of_recording(CONST.CASE_ID, recording_df)
SD = ScenarioData(scenario_df)
starting_sim_time_ms = CONST.STARTING_SIM_TIME_MS
saving_dir = CONST.RECORDING + '_CASE_' + str(CONST.CASE_ID) + '_START_TIME_' + str(starting_sim_time_ms) + '.osm'
SD.construct_gs_file(saving_dir, starting_sim_time_ms)


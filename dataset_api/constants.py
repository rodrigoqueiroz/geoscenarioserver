#!/usr/bin/env python3
# jinwei.zhang@uwaterloo.ca
# ---------------------------------------------

"""This class contains the constants used in the dataset_api package."""
class CONSTANTS:

    def __init__(self):
        pass

    # Dataset constants
    DATASET_BASE_DIR = '/mnt/Data/Research_Dataset/interaction_dataset/INTERACTION-Dataset-DR-multi-v1_2/'
    DATASET_VERSION = 'v1.2'
    RECORDING_MAP = {
        0: 'DR_CHN_Merging_ZS0',
        1: 'DR_CHN_Merging_ZS2',
        2: 'DR_CHN_Roundabout_LN',
        3: 'DR_DEU_Merging_MT',
        4: 'DR_DEU_Roundabout_OF',
        5: 'DR_Intersection_CM',
        6: 'DR_LaneChange_ET0',
        7: 'DR_LaneChange_ET1',
        8: 'DR_Merging_TR0',
        9: 'DR_Merging_TR1',
        10: 'DR_Roundabout_RW',
        11: 'DR_USA_Intersection_EP0',
        12: 'DR_USA_Intersection_EP1',
        13: 'DR_USA_Intersection_GL',
        14: 'DR_USA_Intersection_MA',
        15: 'DR_USA_Roundabout_EP',
        16: 'DR_USA_Roundabout_FT',
        17: 'DR_USA_Roundabout_SR'
        }

    # Scene data constants
    RECORDING = RECORDING_MAP[3]
    MAP_DIR = DATASET_BASE_DIR+'maps/'+RECORDING+'.osm'
    CASE_ID = 70
    STARTING_SIM_TIME_MS = 100

    # GeoScenario static icon constants
    ## Global configuration layouts, abbreviated as GC
    ## The default number represents 10 meters away from origin
    GC_ICON_LAT = 9.034831792182725e-05
    GC_ICON_LON = 9.034831792182725e-05

    # GeoScenario Simulation constants
    COLLISION = True


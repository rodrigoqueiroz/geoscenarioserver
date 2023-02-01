#!/usr/bin/env python3
# jinwei.zhang@uwaterloo.ca
# ---------------------------------------------

"""This class contains the constants used in the dataset_api package."""
class CONSTANTS:

    def __init__(self):
        pass

    # Dataset constants
    BASE_DIR = '/mnt/Data/Research_Dataset/interaction_dataset/INTERACTION-Dataset-DR-multi-v1_2/'
    DATASET_VERSION = 'v1.2'
    RECORDING_MAP = ['DR_DEU_Merging_MT', 'DR_DEU_Roundabout_OF']

    # Scene data constants
    RECORDING = 'DR_DEU_Merging_MT'
    MAP_DIR = BASE_DIR+'maps/'+RECORDING+'.osm'
    CASE_ID = 10

    # GeoScenario static icon constants
    ## Global configuration layouts, abbreviated as GC
    ## The default number represents 10 meters away from origin
    GC_ICON_LAT = 9.034831792182725e-05
    GC_ICON_LON = 9.034831792182725e-05

    # GeoScenario Simulation constants
    COLLISION = True


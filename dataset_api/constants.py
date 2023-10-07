#!/usr/bin/env python3
# jinwei.zhang@uwaterloo.ca
# ---------------------------------------------
import os
"""This class contains the constants used in the dataset_api package."""
class CONSTANTS:

    def __init__(self):
        pass

    # Dataset constants
    DATASET_BASE_DIR = '/home/jinwei/datasets/INTERACTION-Dataset-DR-multi-v1_2/'
    MAP_BASE_DIT = '/home/jinwei/workspace/geoscenarioserver/scenarios/maps/interaction_maps_EXT/'
    DATASET_VERSION = 'v1.2'

    # Older version ones
#     RECORDING_MAP = {
#         0: 'DR_CHN_Merging_ZS0',
#         1: 'DR_CHN_Merging_ZS2',
#         2: 'DR_CHN_Roundabout_LN',
#         3: 'DR_DEU_Merging_MT',
#         4: 'DR_DEU_Roundabout_OF',
#         5: 'DR_Intersection_CM',
#         6: 'DR_LaneChange_ET0',
#         7: 'DR_LaneChange_ET1',
#         8: 'DR_Merging_TR0',
#         9: 'DR_Merging_TR1',
#         10: 'DR_Roundabout_RW',
#         11: 'DR_USA_Intersection_EP0',
#         12: 'DR_USA_Intersection_EP1',
#         13: 'DR_USA_Intersection_GL',
#         14: 'DR_USA_Intersection_MA',
#         15: 'DR_USA_Roundabout_EP',
#         16: 'DR_USA_Roundabout_FT',
#         17: 'DR_USA_Roundabout_SR'
#         }
    
    RECORDING_MAP = {
        0: 'DR_CHN_Merging_ZS0',
        1: 'DR_CHN_Merging_ZS2',
        2: 'DR_CHN_Roundabout_LN',
        3: 'DR_DEU_Merging_MT',
        4: 'DR_DEU_Roundabout_OF',
        5: 'DR_USA_Intersection_EP0',
        6: 'DR_USA_Intersection_EP1',
        7: 'DR_USA_Intersection_GL',
        8: 'DR_USA_Intersection_MA',
        9: 'DR_USA_Roundabout_EP',
        10: 'DR_USA_Roundabout_FT',
        11: 'DR_USA_Roundabout_SR'
        }
    
    MAP_DIR_MAP = {
        'DR_CHN_Merging_ZS0': os.path.join(MAP_BASE_DIT, 'DR_CHN_Merging_ZS0_EXT.osm'),
        'DR_CHN_Merging_ZS2': os.path.join(MAP_BASE_DIT, 'DR_CHN_Merging_ZS2_EXT.osm'),
        'DR_CHN_Roundabout_LN': os.path.join(MAP_BASE_DIT, 'DR_CHN_Roundabout_LN_EXT.osm'),
        'DR_DEU_Merging_MT': os.path.join(MAP_BASE_DIT, 'DR_DEU_Merging_MT_EXT.osm'),
        'DR_DEU_Roundabout_OF': os.path.join(MAP_BASE_DIT, 'DR_DEU_Roundabout_OF_EXT.osm'),
        'DR_USA_Intersection_EP0': os.path.join(MAP_BASE_DIT, 'DR_USA_Intersection_EP0_EXT.osm'),
        'DR_USA_Intersection_EP1': os.path.join(MAP_BASE_DIT, 'DR_USA_Intersection_EP1_EXT.osm'),
        'DR_USA_Intersection_GL': os.path.join(MAP_BASE_DIT, 'DR_USA_Intersection_GL_EXT.osm'),
        'DR_USA_Intersection_MA': os.path.join(MAP_BASE_DIT, 'DR_USA_Intersection_MA_EXT.osm'),
        'DR_USA_Roundabout_EP': os.path.join(MAP_BASE_DIT, 'DR_USA_Roundabout_EP_EXT.osm'),
        'DR_USA_Roundabout_FT': os.path.join(MAP_BASE_DIT, 'DR_USA_Roundabout_FT_EXT.osm'),
        'DR_USA_Roundabout_SR': os.path.join(MAP_BASE_DIT, 'DR_USA_Roundabout_SR_EXT.osm')
    }

    # Scene data constants
    # RECORDING = RECORDING_MAP[7]
    # MAP_DIR = MAP_DIR_MAP[RECORDING]
    # MAP_DIR = DATASET_BASE_DIR+'maps/'+RECORDING+'.osm'
    CASE_ID = 31
    STARTING_SIM_TIME_MS = 100
    # VEHICLE_TYPE_DICT = {
    #     1: 'TV',
    #     2: 'TV',
    #     3: 'TV',
    #     4: 'SDV',
    #     5: 'SDV',
    #     }
    VEHICLE_TYPE_DICT = {i: 'SDV' for i in range(1, 50)}

    # GeoScenario static icon constants
    ## Global configuration layouts, abbreviated as GC
    ## The default number represents 10 meters away from origin
    GC_ICON_LAT = 9.034831792182725e-05
    GC_ICON_LON = 9.034831792182725e-05

    # GeoScenario Simulation constants
    COLLISION = True


#Constants to parse the Behavior Tree Files.

#Basics
FALSE = 0
TRUE = 1

#Traffic Lights
GREEN = 1
RED = 2
YELLOW = 3

#Road Zones
FRONT = 1
LEFT_FRONT = 2
LEFT = 3
LEFT_BACK = 4
BACK = 5
RIGHT_BACK = 6
RIGHT = 7
RIGHT_FRONT = 8

#Lanes
LEFT = +1
CURRENT = 0
RIGHT = -1

RIGHT_OF_WAY = 1
ALL_WAY_STOP = 2
TRAFFIC_LIGHT = 3
PEDESTRIAN_CROSS = 4

#Special Vehicles (Codes are NOT their VIDs assigned by scenario design)
EGO = 999.0
LEAD_VEHICLE = 991.0            
TRAILLING_VEHICLE = 995.0       

GOAL_POINT = 2  
STOP_LINE = 3   

LINEAR_DISTRIBUTION = 1      #linear space
UNIFORM_DISTRIBUTION = 2     #random from uniform distribution
NORMAL_DISTRIBUTION = 3      #random from gaussian

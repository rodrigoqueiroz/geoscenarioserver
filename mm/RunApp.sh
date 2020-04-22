#!/bin/bash
now=$(date +"%Y-%m-%d-%R")
Python3 Simulator.py >> ../logs/mmsim_${now}.log &
/Users/rodrigo/Workspace/Unreal\ Projects/VehicleManeuver/Package/MacNoEditor/VehicleManeuver.app/Contents/MacOS/VehicleManeuver -WINDOWED -ResX=800 -ResY=600 -WindowPosX=-1
-WindowPosY=-1 >> ../logs/unreal_${now}.log &

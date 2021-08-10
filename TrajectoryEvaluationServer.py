#!/usr/bin/env python
#slarter@uwaterloo.ca

from argparse import ArgumentParser
from evaluation.TrajectoryEvaluation import *
import glog as log
import os
from gsc.GSParser import GSParser
from lanelet2.projection import UtmProjector
from mapping.LaneletMap import *
from SimConfig import ROOT_DIR

def get_lanelet_map_lines(video_id):
    lanelet_map = LaneletMap()

    eval_scenario_file = 'evaluation/eval_scenarios/base_eval_scenario_{}.osm'.format(video_id)
    full_scenario_paths = [os.path.join(ROOT_DIR, eval_scenario_file)]

    #========= Parse GeoScenario File
    parser = GSParser()
    if not parser.load_and_validate_geoscenario(full_scenario_paths):
        log.error("Error loading GeoScenario file(s)")
        return False
    if parser.globalconfig.tags['version'] < 2.0:
        log.error("GSServer requires GeoScenario 2.0 or newer")
        return False

    #========= Map
    map_file = os.path.join(ROOT_DIR, 'scenarios/maps/lanelet2_university_weber_alt_walkways.osm') #use default map path
    projector = UtmProjector(lanelet2.io.Origin(parser.origin.lat, parser.origin.lon, 0.0))
    parser.project_nodes(projector, 0.0)
    lanelet_map.load_lanelet_map(map_file, projector)

    x_min = -50
    x_max = 50
    y_min = -50
    y_max = 50

    map_lines = lanelet_map.get_lines(x_min,y_min,x_max,y_max)

    return map_lines


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-s", "--scenario", dest="eval_id", default="", help="Scenario Id for evaluation")
    parser.add_argument("-v", "--video-id", dest="video_id", default="769", help="Video file id (default: 769)")
    parser.add_argument("-t", "--type", dest="eval_type", default="", help="Type for batch evaluation")
    parser.add_argument("-tf", "--traj_file", dest="traj_file", default="", help="Single Trajectory analysis")
    parser.add_argument("-a", "--all", dest="eval_all", action="store_true", help="Batch evaluation for all trajectories")
    args = parser.parse_args()

    scenarios = load_all_scenarios(args.video_id)

    map_lines = get_lanelet_map_lines(args.video_id)

    #All scenarios
    if args.eval_all:
        for id in scenarios:
            try:
                evaluate_scenario(scenarios[id], map_lines)
                results = update_results_table(scenarios[id])
            except Exception as e:
                print("ERROR. Can not run evaluation for scenario {}".format(id))
                print(e)
        #generate_boxplots(results)
    #All scenarios from type
    elif args.eval_type != "":
        for id in scenarios:
            if scenarios[id].scenario_type == args.eval_type:
                try:
                    evaluate_scenario(scenarios[id], map_lines)
                    results = update_results_table(scenarios[id])
                except Exception as e:
                    print("ERROR. Can not run evaluation for scenario {}".format(id))
                    print(e)
        #generate_boxplots(results)
    #One scenario
    elif args.eval_id != "":
        try:
            evaluate_scenario(scenarios[args.eval_id], map_lines)
            results = update_results_table(scenarios[args.eval_id])
            #generate_boxplots(results)
        except Exception as e:
            print("ERROR. Can not run evaluation for scenario {}".format(args.eval_id))
            raise e

    #evaluate single trajectory
    elif args.traj_file != "":
        evaluate_trajectory(args.video_id, args.traj_file)

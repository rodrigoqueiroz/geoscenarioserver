#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# GeoScenario Server for Evaluation only
# Controls the problem setup and traffic simulation loop
# --------------------------------------------

from argparse import ArgumentParser
from evaluation.Evaluation import *
import glog as log


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-n", "--no_dash", dest="no_dash", action="store_true", help="run without the dashboard")
    parser.add_argument("-s", "--scenario", dest="gsfile", nargs='*', metavar="FILE", default="", help="GeoScenario file")
    parser.add_argument("-v", "--video-id", dest="video_id", default="769", help="Video file id")
    parser.add_argument("-e", "--eval", dest="eval_id", default="", help="Evaluation scenario ID")
    parser.add_argument("-t", "--type", dest="eval_type", default="", help="Type for batch evaluation")
    parser.add_argument("-rc", "--recalibrate", dest="recalibrate", default="n", help="[y/n/b] Recalibrate behavior to match reference vehicle (b for both)")
    parser.add_argument("-c", "--compare", dest="compare", default="y", help="[y/n/e] Compare trajectories? e=for exclusively")
    parser.add_argument("-a", "--all", dest="eval_all", action="store_true", help="Batch evaluation for all trajectories")


    args = parser.parse_args()

    CLIENT_SHM = False
    WRITE_TRAJECTORIES = True


    # Master csv to guide all experiments.
    scenarios = load_all_scenarios(args.video_id)

    if args.eval_all:
         for id in scenarios:
            try:
                if args.recalibrate == 'b':
                    start_server(args,scenarios[id], False)
                    start_server(args,scenarios[id], True)
                elif args.recalibrate == 'n':
                    start_server(args,scenarios[id], False)
                else:
                    start_server(args,scenarios[id], True)

            except Exception as e:
                print("ERROR. Can not run evaluation for scenario {}".format(id))
    #Run single scenario
    elif args.eval_id != "":
        try:
            if args.recalibrate == 'b':
                start_server(args, scenarios[args.eval_id], False)
                start_server(args, scenarios[args.eval_id], True)
            elif args.recalibrate == 'n':
                start_server(args, scenarios[args.eval_id], False)
            else: #default
                start_server(args, scenarios[args.eval_id], True)

        except Exception as e:
            print("ERROR. Can not run simulation for scenario {}".format(args.eval_id))
            raise e
    #Run all scenarios from type
    elif args.eval_type != "":
        for eval_id in scenarios:
            if scenarios[eval_id].scenario_type == args.eval_type:
                #try:
                if args.recalibrate == 'b':
                    start_server(args, scenarios[eval_id], False)
                    start_server(args, scenarios[eval_id], True)
                elif args.recalibrate == 'n':
                    start_server(args, scenarios[eval_id], False)
                else: #default
                    start_server(args, scenarios[eval_id], True)
                #except Exception as e:
                #    print("ERROR. Can not run simulation for scenario{}".format(eval_id))

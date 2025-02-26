#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# GeoScenario Server
# Starts the Server and controls the traffic simulation loop
# --------------------------------------------

from argparse import ArgumentParser
from TickSync import TickSync
from SimTraffic import SimTraffic
from SimConfig import SimConfig
from dash.Dashboard import *
from mapping.LaneletMap import *
from ScenarioSetup import *
try:
    from lanelet2.projection import LocalCartesianProjector
    use_local_cartesian=True
except ImportError:
    from lanelet2.projection import UtmProjector
    use_local_cartesian=False

import glog as log
import screeninfo
from pynput import keyboard

def start_server(args, m=MVelKeepConfig()):
    # log.setLevel("INFO")
    log.info('GeoScenario server START')
    lanelet_map = LaneletMap()
    sim_config = SimConfig()

    base_btree_location = os.path.join(ROOT_DIR, "btrees") #default btree folders location
    btree_locations = []
    if len(args.btree_locations) > 0:
        btree_locations.extend(args.btree_locations.split(":"))
        btree_locations.append(base_btree_location)
    else:
        btree_locations = [base_btree_location]
    log.info ("Btree search locations set (in order) as: " + str(btree_locations))

    if args.verify_map != "":
        verify_map_file(args.verify_map, lanelet_map)
        return

    if args.no_dash:
        sim_config.show_dashboard = False

    if args.wait_for_input:
        sim_config.wait_for_input = True

    if args.wait_for_client:
        sim_config.wait_for_client = True

    # use sim_config after all modifications
    traffic = SimTraffic(lanelet_map, sim_config)

    # SCENARIO SETUP
    if args.gsfiles:
        if all(['.osm' in file for file in args.gsfiles]):
            #GeoScenario XML files (GSParser)
            res = load_geoscenario_from_file(args.gsfiles, traffic, sim_config, lanelet_map, args.map_path, btree_locations)
        elif len(args.gsfiles) > 1:
            log.error("Can only load multiple scenarios from .osm files.")
            return
        else:
            #Direct setup
            res = load_geoscenario_from_code(args.gsfiles[0], traffic, sim_config, lanelet_map)
    else:
        res = load_geoscenario_from_code("", traffic, sim_config, lanelet_map)

    if not res:
        log.error("Failed to load scenario")
        return

    sync_global = TickSync(rate=sim_config.traffic_rate, realtime=True, block=True, verbose=False, label="EX")
    sync_global.set_timeout(sim_config.timeout)

    #find screen info 
    monitors = screeninfo.get_monitors()
    primary_monitor = None
    for monitor in monitors:
        if monitor.is_primary:
            primary_monitor = monitor
            break
    
    screen_param = [primary_monitor.x, primary_monitor.y, primary_monitor.width, primary_monitor.height]
    
    if args.dash_pos:
        screen_param = args.dash_pos

    if sim_config.wait_for_input:
        if not sim_config.show_dashboard:
            input("Press [ENTER] to start...")
        else:
            #create a small window
            def on_enter(key):
                if key == keyboard.Key.enter:
                    start_window.after(0, start_window.quit())
            
            pos_x = screen_param[0]
            pos_y = screen_param[1]
            
            start_window = tk.Tk()
            set_width = 300
            set_height = 200

            if args.dash_pos:
                #place in the middle of the dashboard
                pos_x = args.dash_pos[0] + args.dash_pos[2] // 2 - set_width // 2
                pos_y = args.dash_pos[1] + args.dash_pos[3] // 2 - set_height // 2
            else:
                pos_x += (screen_param[2] - set_width) // 2
                pos_y += (screen_param[3] - set_height) // 2
            
            # Apply position
            start_window.geometry(f"{set_width}x{set_height}+{int(pos_x)}+{int(pos_y)}")

            #set window text
            instructions = tk.Label(start_window, text="Press [ENTER] to start...")
            instructions.pack(expand=True)

            start_window.lift()
            start_window.attributes('-topmost', True)
            start_window.focus_force()

            listener = keyboard.Listener(on_press=on_enter)
            listener.start()

            start_window.mainloop()
            start_window.destroy()

    #SIM EXECUTION START
    log.info('SIMULATION START')
    traffic.start()

    #GUI / Debug screen
    dashboard = Dashboard(traffic, sim_config, screen_param)

    if sim_config.show_dashboard:
        dashboard.start()
    else:
        log.warn("Dashboard will not start")

    while sync_global.tick():
        if sim_config.show_dashboard and not dashboard._process.is_alive(): # might/might not be wanted
            break
        try:
            #Update Traffic
            sim_status = traffic.tick(
                sync_global.tick_count,
                sync_global.delta_time,
                sync_global.sim_time
            )
            if sim_status < 0:
                break
        except Exception as e:
            log.error(e)
            break
    sync_global.write_peformance_log()
    traffic.stop_all()

    if sim_config.show_dashboard:
        dashboard.quit()

    #SIM END
    log.info('SIMULATION END')
    log.info('GeoScenario server shutdown')

def verify_map_file(map_file, lanelet_map:LaneletMap):
    if use_local_cartesian:
        projector = LocalCartesianProjector(lanelet2.io.Origin(43.4681668322, -80.5436763174, 302))
        log.info("Using LocalCartesianProjector")
    else:
        projector = UtmProjector(lanelet2.io.Origin(43.4681668322, -80.5436763174, 302))
        log.info("Using UTMProjector")
    lanelet_map.load_lanelet_map(map_file, projector)



if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-s", "--scenario", dest="gsfiles", nargs='*', metavar="FILE", default="", help="GeoScenario file. If no file is provided, the GSServer will load a scenario from code")
    parser.add_argument("--verify_map", dest="verify_map", metavar="FILE", default="", help="Lanelet map file")
    parser.add_argument("-q", "--quiet", dest="verbose", default=True, help="don't print messages to stdout")
    parser.add_argument("-n", "--no-dash", dest="no_dash", action="store_true", help="run without the dashboard")
    parser.add_argument("-m", "--map-path", dest="map_path", default="", help="Set the prefix to append to the value of the attribute `globalconfig->lanelet`")
    parser.add_argument("-b", "--btree-locations", dest="btree_locations", default="", help="Add higher priority locations to search for btrees by agent btypes")
    parser.add_argument("-wi", "--wait-for-input", dest="wait_for_input", action="store_true", help="Wait for the user to press [ENTER] to start the simulation")
    parser.add_argument("-wc", "--wait-for-client", dest="wait_for_client", action="store_true", help="Wait for a valid client state to start the simulation")
    parser.add_argument("--dash-pos", default=[], dest="dash_pos", type=float, nargs=4, help="Set the position of the dashboard window (x y width height)")

    args = parser.parse_args()
    start_server(args)
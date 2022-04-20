#!/usr/bin/env python3
# j2younge@uwaterloo.ca
# ---------------------------------------------
# Simulation Dashboard and Trajectory Plots V2
# --------------------------------------------
import numpy as np
from SimTraffic import *
import sv.SDVTrafficState
from sv.Vehicle import *
from Actor import *
from TrafficLight import *
from sp.Pedestrian import *
from mapping.LaneletMap import LINE_FORMAT

from matplotlib import colors
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import pyqtgraph as pg


LINE_STYLE_MAPPING = {
    'dashdot': Qt.DashDotLine,
    'dashed': Qt.DashLine,
    'dotted': Qt.DotLine,
    'solid': Qt.SolidLine
}


class ScrollLabel(QScrollArea):
    # https://www.geeksforgeeks.org/pyqt5-scrollable-label/
    def __init__(self, *args, **kwargs):
        QScrollArea.__init__(self, *args, **kwargs)
        self.setWidgetResizable(True)
        content = QWidget(self)
        self.setWidget(content)
        lay = QVBoxLayout(content)
        self.label = QLabel(content)
        self.label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.label.setWordWrap(True)
        lay.addWidget(self.label)

    def setText(self, text):
        self.label.setText(text)


class VehicleBox(pg.GraphicsObject):
    def __init__(self):
        self.pos = (0, 0)
        self.yaw = 0
        pg.GraphicsObject.__init__(self)
        self.generatePicture()

    def setPose(self, x, y, yaw):
        self.pos = (x, y)
        self.yaw = yaw
        self.generatePicture()

    def generatePicture(self):
        self.picture = QPicture()
        p = QPainter(self.picture)
        p.setPen(pg.mkPen('w'))
        p.setBrush(pg.mkBrush(color='b'))
        p.translate(*self.pos)
        p.rotate(self.yaw)
        # TODO Fix width centering problem
        rx = -VEHICLE_LENGTH / 2
        ry = -VEHICLE_WIDTH / 2
        p.drawRect(QRect(rx, ry, VEHICLE_LENGTH, VEHICLE_WIDTH))
        p.end()
        self.informViewBoundsChanged()
        self.prepareGeometryChange()

    def paint(self, p, *args):
        p.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        return QRectF(self.picture.boundingRect())

"""
Plots both global map and cartesian (local) map.
"""
class MapPlot(pg.PlotWidget):
    def __init__(self, lanelet_map, sim_traffic, local=False):
        super().__init__(name="Vehicle Map" if local else "Global Map")
        self.lanelet_map = lanelet_map
        self.sim_traffic = sim_traffic
        self.local = local

        # Configure
        self.addLegend()
        self.setAspectLocked(True)
        self.map_static = self.plot(
            pen=None, symbolBrush=(0, 0, 200),
            symbolPen='w', symbol='x', symbolSize=14, name="Static Objects")

        # If Local
        if local:
            self.map_path = pg.PlotDataItem(pen=pg.mkPen(color='b', width=4, style=Qt.DashLine), name='Path')
            self.addItem(self.map_path)
            self.map_path.setZValue(-1)
        # Plot Item Containers
        self.traffic_lights = {}
        self.controlled_stop_lines = {}
        self.vehicles = {}

        # Construct plot for vehicle centers
        self.vehicle_centers = self.plot(
            pen=None,
            brush=pg.mkBrush(color='b'),
            symbol='o',
            symbolSize=10,
            name="Vehicles (Centers)", connect=False)

        # Create lane line plot items (per type/subtype)
        self.map_roads = {}
        for type, fmt in LINE_FORMAT.items():
            if fmt is None:
                continue
            color, style, width = fmt
            if style in LINE_STYLE_MAPPING:
                style = LINE_STYLE_MAPPING[style]
            else:
                style = LINE_STYLE_MAPPING["solid"]
            color = np.rint(np.dot(colors.to_rgba(color), 255))
            if max(color) < 51:
                color = (191, 191, 191)
            if isinstance(type, tuple):
                type_name = "%s (%s)" % type
            else:
                type_name = str(type)
            self.map_roads[type] = self.plot(
                pen=pg.mkPen(color=color, width=width, style=style),
                name=type_name, connect='finite')

    def plot_map_chart(self, vehicles, pedestrians, traffic_light_states, static_objects, first_frame):
        # Boundaries (center is GeoScenario origin)
        x_min = -(MPLOT_SIZE / 2)
        y_min = -(MPLOT_SIZE / 2)
        x_max = (MPLOT_SIZE / 2)
        y_max = (MPLOT_SIZE / 2)

        if first_frame:
            self.plot_road(x_min, x_max, y_min, y_max)
            self.plot_static_objects(static_objects, x_min, x_max, y_min, y_max)
            self.plot_traffic_lights(traffic_light_states)
            self.setRange(xRange=(x_min, x_max), yRange=(y_min, y_max))
        else:
            self.update_traffic_lights(traffic_light_states)
        self.plot_vehicles(vehicles, x_min, x_max, y_min, y_max)
        # TODO
        # self.plot_pedestrians(pedestrians, x_min, x_max, y_min, y_max)

        # self.plotItem.vb.setLimits(xMin=x_min, xMax=x_max, yMin=y_min, yMax=y_max)

    def plot_cartesian_chart(self, center_id, vehicles, pedestrians, reference_path=None, traffic_lights=None,
                             static_objects=None, first_frame=False):
        # Boundaries
        x_min = vehicles[center_id].state.x - (CPLOT_SIZE / 2)
        x_max = vehicles[center_id].state.x + (CPLOT_SIZE / 2)
        y_min = vehicles[center_id].state.y - (CPLOT_SIZE / 2)
        y_max = vehicles[center_id].state.y + (CPLOT_SIZE / 2)

        if first_frame:
            self.plot_road(x_min, x_max, y_min, y_max)
            self.plot_static_objects(static_objects, x_min, x_max, y_min, y_max)
            self.plot_traffic_lights(traffic_lights)
        else:
            self.update_traffic_lights(traffic_lights)

        if REFERENCE_PATH and reference_path is not None:
            path_x, path_y = zip(*reference_path)
            self.map_path.setData(path_x, path_y)

        self.plot_vehicles(vehicles, x_min, x_max, y_min, y_max, True)
        # TODO
        # self.plot_pedestrians(pedestrians, x_min, x_max, y_min, y_max)
        self.plotItem.vb.setLimits(xMin=x_min, xMax=x_max, yMin=y_min, yMax=y_max)

    def plot_road(self, x_min, x_max, y_min, y_max):
        # Road Markings
        data = self.lanelet_map.get_lines(x_min, y_min, x_max, y_max)
        for xs, ys, type, subtype in data:
            xs.append(np.nan)
            ys.append(np.nan)
            if (type, subtype) in self.map_roads:
                plot_item = self.map_roads[(type, subtype)]
            elif type in self.map_roads:
                plot_item = self.map_roads[type]
            else:
                continue
            X, Y = plot_item.getData()
            if X is not None:
                plot_item.setData(x=np.append(X, xs), y=np.append(Y, ys))
            else:
                plot_item.setData(x=xs, y=ys)

        # stop lines
        X = []
        Y = []
        for stop_line in self.lanelet_map.get_stop_lines():
            X.extend([pt.x for pt in stop_line])
            X.append(np.nan)
            Y.extend([pt.y for pt in stop_line])
            Y.append(np.nan)
        self.map_roads["stop_line"].setData(x=X, y=Y)

        # stop signs
        X = []
        Y = []
        for stop_sign in self.lanelet_map.get_stop_signs():
            X.append(stop_sign.x)
            Y.append(stop_sign.y)
        self.map_roads["stop_sign"].setData(x=X, y=Y, pen=None, symbol="o", color='r', symbolBrush='r')

    def plot_static_objects(self, static_objects, x_min, x_max, y_min, y_max):
        if static_objects:
            X = []
            Y = []
            for oid, obj in static_objects.items():
                x = obj.x
                y = obj.y
                if (x_min <= x <= x_max) and (y_min <= y <= y_max):
                    label = "{}".format(oid)
                    X.append(x)
                    Y.append(y)
                    text = pg.TextItem(label, anchor=(0, 1), angle=0)
                    text.setPos(x, y)
                    text.setColor('w')
                    self.addItem(text)
            self.map_static.setData(X, Y, clear=True)

    def plot_traffic_lights(self, traffic_light_states=None):
        if traffic_light_states:
            for lid, state in traffic_light_states.items():
                self.plot_traffic_light(lid, state)

    def plot_traffic_light(self, lid, state):
        tl = self.sim_traffic.traffic_lights[lid]
        # find physical light locations
        x, y, line = self.lanelet_map.get_traffic_light_pos(lid)
        print("[{}] Traffic light {} at {}, {}, with state {}".format(self, lid, x, y, state))

        colorcode, alpha = Dashboard2.get_color_by_type('trafficlight', state)
        tl_type = tl.type
        symbol_size = 14
        if tl_type == TrafficLightType.pedestrian:
            symbol_size = 8
        if tl_type == TrafficLightType.left:
            symbol = 't3'
        elif tl_type == TrafficLightType.right:
            symbol = 't2'
        else:
            symbol = 'o'
        self.traffic_lights[lid] = self.plot(
            x=[x], y=[y], pen=None, symbolBrush=colorcode,
            symbolPen=pg.mkPen(color='y', width=2), symbol=symbol, symbolSize=symbol_size)

        if tl_type != TrafficLightType.pedestrian:
            label = "{}".format(tl.name)
            text = pg.TextItem(label, anchor=(0, 1), color='w')
            text.setPos(x, y)
            self.addItem(text)
            self.controlled_stop_lines[lid] = self.plot(
                x=line[0], y=line[1], pen=pg.mkPen(color=colorcode, width=4), symbol=None)

    def update_traffic_lights(self, traffic_light_states=None):
        if traffic_light_states:
            for lid, state in traffic_light_states.items():
                try:
                    tfl = self.traffic_lights[lid]
                    colorcode, alpha = Dashboard2.get_color_by_type('trafficlight', state)
                    tfl.setSymbolBrush(colorcode)
                    tfl_line = self.controlled_stop_lines[lid]
                    tfl_line.setPen(pg.mkPen(color=colorcode, width=4), symbol=None)
                except KeyError:
                    self.plot_traffic_light(lid, state)

    def plot_vehicles(self, vehicles, x_min, x_max, y_min, y_max, show_arrow=False):
        if vehicles:
            XC = []
            YC = []
            for vid, vehicle in vehicles.items():
                if vehicle.sim_state is ActorSimState.INACTIVE:
                    continue
                colorcode, alpha = Dashboard2.get_color_by_type('vehicle', vehicle.type, vehicle.sim_state, vehicle.name)
                x = vehicle.state.x
                y = vehicle.state.y
                if (x_min - 5 <= x <= x_max + 5) and (y_min - 5 <= y <= y_max + 5):
                    if vid not in self.vehicles:
                        self.vehicles[vid] = {}
                    vplots = self.vehicles[vid]
                    XC.append(x)
                    YC.append(y)

                    if SHOW_VEHICLE_SHAPE:
                        if 'box' not in vplots:
                            box = VehicleBox()
                            box.setPose(x, y, vehicle.state.yaw)
                            vplots['box'] = box
                            self.addItem(box)
                        vplots['box'].setPose(x, y, vehicle.state.yaw)
                    if SHOW_VEHICLE_RADIUS:
                        pass  # TODO

                    # label
                    label = "ego ({})".format(int(vid)) if vehicle.name.lower() == 'ego' else "v{}".format(int(vid))
                    label_shift = 2 if SHOW_VEHICLE_SHAPE else 1
                    if 'label' not in vplots:
                        text = pg.TextItem(label, anchor=(0, 1), color='w')
                        text.setZValue(10)
                        self.addItem(text)
                        vplots['label'] = text
                    else:
                        text = vplots['label']
                    text.setPos(x, y)

                    # arrow
                    if show_arrow:
                        vx = vehicle.state.x_vel
                        vy = vehicle.state.y_vel
                        if vehicle.state.s_vel < 0:
                            vx = -vx
                            vy = -vy
                        if 'vel' not in vplots:
                            vel = pg.ArrowItem(
                                tipAngle=30, baseAngle=20, headLen=2, tailLen=4,
                                tailWidth=.4,
                                pen=None, brush='b',
                                pxMode=False)
                            self.addItem(vel)
                            vplots['vel'] = vel
                        else:
                            vel = vplots['vel']
                        # The Arrow Item points at its position, so
                        # it takes some manipulating to make a velocity vector.
                        vel.setPos(x + vx, y + vy)
                        # Subtract headLen from tailLen so total length is speed.
                        vel.setStyle(
                            angle=np.rad2deg(np.arctan2(-vy, -vx)),
                            tailLen=max(0, vehicle.state.get_cartesian_speed() - 2))

                else:
                    if vid in self.vehicles:
                        for plot in self.vehicles.pop(vid).values():
                            self.removeItem(plot)
            self.vehicle_centers.setData(XC, YC)
            self.vehicle_centers.setZValue(1)


class Dashboard2(object):
    TITLE = "GeoScenario Server"
    MAP_FIG_ID = 1
    CART_FIG_ID = 2
    FRE_FIG_ID = 3
    TRAJ_FIG_ID = 4

    def __init__(self, sim_traffic: SimTraffic, sim_config: SimConfig):
        self.sim_traffic: SimTraffic = sim_traffic
        self.center_id = int(sim_config.plot_vid)
        self.sim_config = sim_config
        self.window = None
        self.center_pedestrian = False
        self.lanelet_map: LaneletMap = None

    def start(self):
        """ Start Dashboard2 in subprocess.
            global constant SHOW_Dashboard2 must be true
            Traffic must have started, otherwise the shared array is not ready
        """

        if not self.sim_traffic:
            log.error("Dashboard2 requires a traffic to start")
            return

        if not self.sim_traffic.traffic_state_sharr:
            log.error("Dashboard2 can not start before traffic")
            return

        self.last_time = time.time()

        self.lanelet_map = self.sim_traffic.lanelet_map
        self._process = Process(target=self.run_dash_process,
                                args=(self.sim_traffic.traffic_state_sharr, self.sim_traffic.debug_shdata),
                                daemon=True)
        self._process.start()

    def run_dash_process(self, traffic_state_sharr, debug_shdata):

        self.window, w = self.create_gui(traffic_state_sharr, debug_shdata)
        self.window.exec()

    def update(self, traffic_state_sharr, debug_shdata, first_frame):
        # Calculate Display Rate
        current_time = time.time()
        display_delta_time = current_time - self.last_time
        self.last_time = current_time

        # Get New Data
        header, vehicles, pedestrians, traffic_lights, static_objects = self.sim_traffic.read_traffic_state(
            traffic_state_sharr, False)
        tickcount, delta_time, sim_time = header[0:3]
        sim_time_formated = str(datetime.timedelta(seconds=sim_time))
        display_rate = 1 / display_delta_time if display_delta_time > 0 else 0

        # Config/Stats
        config_txt = "Traffic Rate: {}Hz   |   Planner Rate: {}Hz   |   Dashboard2 Rate: {:7.2f} ({}) Hz".format(
            TRAFFIC_RATE, PLANNER_RATE, display_rate, DASH_RATE)
        config_txt += "\nTick#: {}   |   SimTime: {}   |   DeltaTime: {:.2} s".format(
            tickcount, sim_time_formated, delta_time)

        self.l.setText(config_txt)

        # Global Map
        if SHOW_MPLOT:
            self.global_map.plot_map_chart(vehicles, pedestrians, traffic_lights, static_objects, first_frame)

        # Vehicle's table
        self.update_table(vehicles)
        # Pedestrians at the bottom of vehicle's table
        self.update_pedestrian_table(pedestrians)

        # find valid vehicle to focus plots and btree (if available)
        vid = None

        if type(self.center_id) == str:
            if self.center_id[0] == 'p':
                self.center_pedestrian = True
            else:
                self.center_pedestrian = False
            self.center_id = int(self.center_id[1:])  # remove first letter

        if self.center_pedestrian is False and self.center_id in vehicles:
            if vehicles[self.center_id].sim_state is not ActorSimState.INACTIVE:
                vid = int(self.center_id)
                # vehicles with planner: cartesian, frenet chart and behavior tree
                if vid in debug_shdata:
                    # read vehicle planning data from debug_shdata
                    planner_state, btree_snapshot, ref_path, traj, cand, unf, traj_s_shift = debug_shdata[vid]
                    if SHOW_CPLOT:  # cartesian plot with lanelet map
                        self.local_map.plot_cartesian_chart(vid, vehicles, pedestrians, ref_path, traffic_lights,
                                                  static_objects, first_frame=first_frame)
                    if SHOW_FFPLOT:  # frenet frame plot
                        self.plot_frenet_chart(vid, planner_state, ref_path, traj, cand, unf, traj_s_shift)
                    # if VEH_TRAJ_CHART:  # vehicle traj plot
                    #     self.plot_vehicle_sd(traj, cand)

                    # behavior tree
                    self.btree_text.setText(
                        "==== Behavior Tree. Vehicle {} ====\n\n {} ".format(vid, btree_snapshot))
                else:
                    # vehicles without planner:
                    self.local_map.plot_cartesian_chart(vid, vehicles, pedestrians, first_frame=first_frame)
        # elif self.center_pedestrian and self.center_id in pedestrians: TODO
        #     if pedestrians[self.center_id].sim_state is not ActorSimState.INACTIVE:
        #         pid = int(self.center_id)
        #         if SHOW_CPLOT:  # cartesian plot with lanelet map
        #             self.local_map.plot_pedestrian_cartesian_chart(
        #                 pid, vehicles, pedestrians, traffic_lights, static_objects, first_frame=first_frame)

    def quit(self):
        self._process.terminate()

    def change_tab_focus(self, arg: pg.TreeWidgetItem):
        print(arg)
        if arg is not None:
            self.center_id = arg.text(0)  # sets center_id to an int or string
            print("Changed focus to {}".format(self.center_id))

    def update_table(self, vehicles):
        # ('id', 'sim_st') +
        # ('x', 'x_vel', 'x_acc', 'y', 'y_vel', 'y_acc', 's', 's_vel', 's_acc', 'd', 'd_vel', 'd_acc', 'yaw')
        added = set()

        # Update
        for i in range(self.table.topLevelItemCount()):
            item = self.table.topLevelItem(i)
            vid = int(item.text(0)[1:])
            vehicle = vehicles[vid]
            sim_state = vehicle.sim_state
            sv = vehicle.state.get_state_vector()
            truncate_vector(sv, 2)
            for i, x in enumerate(sv):
                item.setText(i + 2, str(x))
            added.add(vid)

        # Add
        for vid in set(vehicles.keys()).difference(added):
            vehicle = vehicles[vid]
            sim_state = vehicle.sim_state
            sv = vehicle.state.get_state_vector()
            truncate_vector(sv, 2)
            sv = ['v' + str(vid), sim_state, *sv]
            sv = [str(x) for x in sv]
            item = QTreeWidgetItem(sv)
            self.table.addTopLevelItem(item)

    def update_pedestrian_table(self, pedestrians):
        # TODO
        if len(pedestrians) == 0:
            return
        for pid in pedestrians:
            pedestrian = pedestrians[pid]
            sim_state = pedestrians[pid].sim_state
            sp = pedestrian.state.get_state_vector()
            truncate_vector(sp, 2)

            sp = ['p' + str(pid)] + [sim_state] + sp
            # self.tab.insert('', 'end', 'p' + str(pid), values=(sp))

    def plot_pedestrian_cartesian_chart(self, center_id, vehicles, pedestrians, traffic_lights=None,
                                        static_objects=None):
        # boundaries
        x_min = pedestrians[center_id].state.x - (CPLOT_SIZE / 2)
        x_max = pedestrians[center_id].state.x + (CPLOT_SIZE / 2)
        y_min = pedestrians[center_id].state.y - (CPLOT_SIZE / 2)
        y_max = pedestrians[center_id].state.y + (CPLOT_SIZE / 2)

        # TODO
        # self.plot_road(x_min, x_max, y_min, y_max, traffic_lights)
        # self.plot_static_objects(static_objects, x_min, x_max, y_min, y_max)
        #
        # self.plot_vehicles(vehicles, x_min, x_max, y_min, y_max, True)
        # self.plot_pedestrians(pedestrians, x_min, x_max, y_min, y_max)

        # layout
        # plt.xlim(x_min, x_max)
        # plt.ylim(y_min, y_max)
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.gca().xaxis.set_visible(True)
        # plt.gca().yaxis.set_visible(True)
        # plt.margins(0, 0)

    def plot_pedestrians(self, pedestrians, x_min, x_max, y_min, y_max, show_arrow=True):
        # TODO
        if pedestrians:
            for pid, pedestrian in pedestrians.items():
                if pedestrian.sim_state is ActorSimState.INACTIVE:
                    continue
                colorcode, alpha = self.get_color_by_type('pedestrian', pedestrian.type, pedestrian.sim_state)
                x = pedestrian.state.x
                y = pedestrian.state.y

                # show pedestrians' goals on map
                x_goal = self.sim_traffic.sim_config.pedestrian_goal_points[pid][-1][0]
                y_goal = self.sim_traffic.sim_config.pedestrian_goal_points[pid][-1][1]
                # plt.plot(x_goal, y_goal, 'r.', markersize=2, zorder=10)
                # plt.gca().text(x_goal + 1, y_goal + 1, "p{} goal".format(pid), style='italic', zorder=10)

                if (x_min <= x <= x_max) and (y_min <= y <= y_max):
                    # plt.plot(x, y, colorcode + '.', markersize=1, zorder=10)
                    # circle1 = plt.Circle((x, y), Pedestrian.PEDESTRIAN_RADIUS, color=colorcode, fill=False, zorder=10,
                    #                      alpha=alpha)
                    # plt.gca().add_artist(circle1)
                    label = "p{}".format(pid)
                    # plt.gca().text(x + 1, y + 1, label, style='italic', zorder=10)

                    # arrow
                    if show_arrow:
                        vx = pedestrian.state.x_vel
                        vy = pedestrian.state.y_vel
                        # plt.arrow(x, y, vx / 2, vy / 2, head_width=0.5, head_length=0.5, color=colorcode, zorder=10)

    def plot_frenet_chart(self, center_id, planner_state, debug_ref_path, traj, cand, unf, traj_s_shift):
        # TODO
        vehicle_state = planner_state.vehicle_state
        vehicles = planner_state.traffic_vehicles
        pedestrians = planner_state.pedestrians
        lane_config = planner_state.lane_config
        static_objects = planner_state.static_objects
        regulatory_elements = planner_state.regulatory_elements
        goal_point_frenet = planner_state.goal_point_frenet

        # road
        # plt.axhline(lane_config.left_bound, color="k", linestyle='-', zorder=0)
        # plt.axhline(lane_config.right_bound, color="k", linestyle='-', zorder=0)
        # plt.title("Frenet Frame. Vehicle {}:".format(center_id))
        # plt.xlabel('S')
        # plt.ylabel('D')

        # stop line (if exists)
        if lane_config.stopline_pos is not None:
            x, y = lane_config.stopline_pos
            # plt.axvline(x, color='r', linestyle='-', zorder=0)

        # Regulatory Elements
        for re in regulatory_elements:
            if isinstance(re, sv.SDVTrafficState.TrafficLightState):
                colorcode, _ = self.get_color_by_type('trafficlight', re.color)
                x, y = re.stop_position
                # plt.axvline(x, color=colorcode, linestyle='-', zorder=1)
            elif isinstance(re, sv.SDVTrafficState.RightOfWayState):
                pass
                # for ll_id in re.row_lanelets:
                # colorfillcode = 'r' if re.row_lanelets[ll_id] > 0 else 'g'
                # ll = self.lanelet_map.laneletLayer[ll_id]
            elif isinstance(re, sv.SDVTrafficState.AllWayStopState):
                pass

        # other vehicles, from main vehicle POV:
        for vid, vehicle in vehicles.items():
            colorcode, alpha = self.get_color_by_type('vehicle', vehicle.type, vehicle.sim_state, vehicle.name)
            vs = vehicle.state
            # plt.plot(vs.s, vs.d, colorcode + ".", zorder=5)
            # circle1 = plt.Circle((vs.s, vs.d), VEHICLE_RADIUS, color=colorcode, fill=False, zorder=5, alpha=alpha)
            # gca.add_artist(circle1)
            label = label = "ego ({})".format(int(vid)) if vehicle.name.lower() == 'ego' else "v{}".format(int(vid))
            # gca.text(vs.s, vs.d + 1.5, label)

        # pedestrian
        for pid, pedestrian in pedestrians.items():
            if pedestrian.sim_state is ActorSimState.INACTIVE:
                continue
            colorcode, alpha = self.get_color_by_type('pedestrian', pedestrian.type, pedestrian.sim_state)
            x = pedestrian.state.s
            y = pedestrian.state.d
            # if (x_min <= x <= x_max) and (y_min <= y <= y_max):
            # plt.plot(x, y, colorcode + '.', markersize=1, zorder=10)
            # circle1 = plt.Circle((x, y), Pedestrian.PEDESTRIAN_RADIUS, color=colorcode, fill=False, zorder=10,
            #                      alpha=alpha)
            # plt.gca().add_artist(circle1)
            label = "p{}".format(pid)
            # plt.gca().text(x + 1, y + 1, label, style='italic', zorder=10)

        # objects
        for oid, obj in static_objects.items():
            x = obj.s
            y = obj.d
            # # if (x_min <= x <= x_max) and (y_min <= y <= y_max):
            # plt.plot(x, y, 'kx', markersize=6, zorder=10)
            label = "{}".format(oid)
            # plt.gca().text(x + 1, y + 1, label, style='italic', zorder=10)

        # Main Vehicle
        if goal_point_frenet is not None:
            x, y = goal_point_frenet[0], goal_point_frenet[1]
            # # plt.plot(x, 'go',markersize=6, zorder=10)
            # plt.axvline(x, color="k", linestyle='-', zorder=0)
            #plt.gca().text(x + 1, y + 1, "goal", style='italic', zorder=10)
        # if cand:
        #     for t in cand:
        #         Dashboard2.plot_trajectory(t[0], t[1], t[2], traj_s_shift, 'grey')
        # if unf:
        #     for t in unf:
        #         Dashboard2.plot_trajectory(t[0], t[1], t[2], traj_s_shift, 'red')
        # if traj:
        #     Dashboard2.plot_trajectory(traj[0], traj[1], traj[2], traj_s_shift, 'blue')

        vs = vehicle_state
        vid = center_id

        # plt.plot(vs.s, vs.d, ".", zorder=20)
        # circle1 = plt.Circle((vs.s, vs.d), VEHICLE_RADIUS, color='b', fill=False, zorder=20)
        # gca.add_artist(circle1)
        # # label = "id{}| [ {:.3}m, {:.3}m/s, {:.3}m/ss] ".format(center_id, float(vs.s), float(vs.s_vel), float(vs.s_acc))
        # label = "v{}".format(int(vid))
        # gca.text(vs.s, vs.d + 1.5, label, zorder=20)

        # update lane config based on current (possibly outdated) reference frame
        # lane_config = self.read_map(vehicle_state, self.reference_path)

        # # layout
        # plt.grid(True)
        # # center plot around main vehicle
        # x_lim_a = vs.s - ((1 / 5) * FFPLOT_LENGTH)  # 1/3 before vehicle
        # x_lim_b = vs.s + ((4 / 5) * FFPLOT_LENGTH)  # 2/3 ahead
        # plt.xlim(x_lim_a, x_lim_b)
        # plt.ylim(vs.d - 8, vs.d + 8)
        # # plt.gca().set_aspect('equal', adjustable='box')
        # # fig.patch.set_facecolor('lightgray')
        # # fig.tight_layout(pad=0.05)

    @staticmethod
    def get_color_by_type(actor, a_type, sim_state=None, name=''):
        # color
        colorcode = 'k'  # black
        if actor == 'vehicle':
            if name.lower() == 'ego':
                colorcode = 'g'  # always green for Ego
            elif a_type == Vehicle.SDV_TYPE:
                colorcode = 'b'  # blue
            elif a_type == Vehicle.EV_TYPE:
                colorcode = 'c'  # cyan
            elif a_type == Vehicle.TV_TYPE:
                colorcode = 'k'  # black
        elif actor == 'pedestrian':
            if a_type == Pedestrian.EP_TYPE:
                colorcode = 'r'  # red
            elif a_type == Pedestrian.TP_TYPE:
                colorcode = 'r'  # bred
            elif a_type == Pedestrian.PP_TYPE:
                colorcode = 'r'  # red
        elif actor == 'trafficlight':
            if a_type == TrafficLightColor.Red:
                colorcode = 'r'
            if a_type == TrafficLightColor.Green:
                colorcode = 'g'
            if a_type == TrafficLightColor.Yellow:
                colorcode = 'y'
        # alpha
        alpha = 1.0
        if sim_state:
            if sim_state == ActorSimState.ACTIVE:
                alpha = 1.0
            elif sim_state == ActorSimState.INACTIVE:
                alpha = 0
            elif sim_state == ActorSimState.INVISIBLE:
                alpha = 0.5

        return colorcode, alpha

    def create_gui_title(self):
        title_W = QWidget()
        title_W.setStyleSheet("background-color:black;")

        hbox = QHBoxLayout()
        title_L = QLabel(Dashboard2.TITLE)
        title_L.setStyleSheet("background-color: black;"
                              "color: white;"
                              "font-family: OpenSans; font-size: 40px;")
        hbox.addWidget(title_L)
        hbox.addStretch()
        hbox.setSpacing(5)
        hbox.setContentsMargins(20, 0, 0, 0)
        logo = QLabel()
        logo.setPixmap(QPixmap(ROOT_DIR + "/dash/img/logos.png"))
        hbox.addWidget(logo)
        title_W.setLayout(hbox)
        return title_W

    def create_gui_stats(self):
        stats_W = QWidget()

        self.l = QLabel()
        self.l.setAlignment(Qt.AlignRight)

        self.r = QLabel()
        config_txt = "Scenario: {}\nMap: {}".format(
            self.sim_traffic.sim_config.scenario_name,
            self.sim_traffic.sim_config.map_name)
        self.r.setText(config_txt)

        hbox = QHBoxLayout()
        hbox.setSpacing(20)
        hbox.setContentsMargins(20, 0, 20, 0)
        hbox.addWidget(self.r)
        hbox.addStretch()
        hbox.addWidget(self.l)
        stats_W.setLayout(hbox)

        return stats_W

    def create_gui(self, traffic_state_sharr, debug_shdata):
        # Main containers:
        # title frame
        # stats frame
        # global frame  [  map  | table  ]
        # vehicle frame [  cart | frenet | btree ]
        app = QApplication([])
        window = QMainWindow()
        root = QWidget()
        window.setCentralWidget(root)
        window.setWindowTitle(Dashboard2.TITLE + " (Dashboard V2)")

        stack = QVBoxLayout()
        stack.setSpacing(5)
        stack.setContentsMargins(0, 0, 0, 0)

        title_W = self.create_gui_title()
        title_W.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
        stack.addWidget(title_W)

        stats_W = self.create_gui_stats()
        stats_W.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
        stack.addWidget(stats_W)

        if SHOW_MPLOT:
            self.global_map = MapPlot(self.lanelet_map, self.sim_traffic)
            self.global_map.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding))
            stack.addWidget(self.global_map)

        # Local Vehicle Map
        self.local_map = MapPlot(self.lanelet_map, self.sim_traffic, local=True)
        self.local_map.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding))
        stack.addWidget(self.local_map)

        # # vehicle frenet
        # fig_fren = plt.figure(Dashboard2.FRE_FIG_ID)
        # # fig_fren.set_size_inches(6,4,forward=True)
        # self.fren_canvas = FigureCanvasTkAgg(fig_fren, fren_frame)
        # self.fren_canvas.get_tk_widget().pack()
        #
        # # vehicle traj
        # fig_traj = plt.figure(Dashboard2.TRAJ_FIG_ID)
        # fig_traj.set_size_inches(4, 4, forward=True)
        # self.traj_canvas = FigureCanvasTkAgg(fig_traj, traj_frame)
        # self.traj_canvas.get_tk_widget().pack()
        #
        # tree_msg = tk.Message(bt_frame, text='', anchor='s',
        #                       width=300,
        #                       bg='white', foreground='black')
        # self.tree_msg = tree_msg
        # tree_msg.grid(row=0, column=0, sticky='nsew')
        #

        root.setLayout(stack)

        # Actor Table
        table_dock = QDockWidget("Actors", window)
        self.table = table = QTreeWidget()
        table.setHeaderLabels((
            'id', 'sim_st',
            'x', 'x_vel', 'x_acc',
            'y', 'y_vel', 'y_acc',
            's', 's_vel', 's_acc',
            'd', 'd_vel', 'd_acc',
            'yaw'
        ))
        for col in range(table.columnCount()):
            table.setColumnWidth(col, 65)

        table.itemDoubleClicked.connect(self.change_tab_focus)

        table.setMaximumSize(800, 800)
        table.setMinimumSize(600, 400)
        table.setSizePolicy(QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed))
        table_dock.setWidget(table)
        table_dock.setFloating(False)
        window.addDockWidget(Qt.RightDockWidgetArea, table_dock)

        # B Tree Text
        tree_dock = QDockWidget("B Tree", window)
        self.btree_text = ScrollLabel()
        self.btree_text.setMaximumSize(800, 800)
        self.btree_text.setMinimumSize(600, 400)
        self.btree_text.setSizePolicy(QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed))
        tree_dock.setWidget(self.btree_text)
        tree_dock.setFloating(False)
        window.addDockWidget(Qt.RightDockWidgetArea, tree_dock)

        window.show()

        self.update(traffic_state_sharr, debug_shdata, True)

        self.timer = QTimer()
        self.timer.setInterval(1000 / DASH_RATE)
        self.timer.timeout.connect(lambda: self.update(traffic_state_sharr, debug_shdata, False))
        self.timer.start()

        return app, window

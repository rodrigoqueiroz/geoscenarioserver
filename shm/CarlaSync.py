# Synchronization with Carla in co-simulation
from SimConfig import *
from sv.Vehicle import *
from TickSync import *
from util.Utils import *
import xml.etree.ElementTree as ET

import logging
log = logging.getLogger(__name__)

if CARLA_COSIMULATION:
    import carla as carla
    
ATTACH_SPECTATOR_TO = None          #vehicle ID or None for manual Camera
USE_SPAWN_POINTS = False
ENABLE_PHYSICS = True
SYNCHRONOUS_MODE = False        # Enables synchronous mode

class CarlaSync(object):

    def __init__(self, server= CARLA_SERVER, port=CARLA_PORT):
        self.server = server
        self.port = port
        self.client = None
        self.world = None
        self.carla_map = None
        self.actor_list = []
        self.spectator = None
        self.focus_vid = 1
        self.carla_vehicles = {} #dictionary
        
        try:
            log.info('Connecting to Carla...')
            self.client = carla.Client( self.server,  self.port )
            self.client.set_timeout(2.0)
            self.is_connected = True
            log.info('Connected')
        except:
            log.error('Can not connect to Carla. Check if Carla Server is running')
            self.is_connected = False
            return

        #Initialize
        self.world = self.client.get_world()
        
        settings = self.world.get_settings()
        settings.synchronous_mode = SYNCHRONOUS_MODE
        self.world.apply_settings(settings)

        #Map
        #self.change_map()
        self.carla_map = self.world.get_map()
        log.info(f"Carla Map: {self.carla_map.name}")
        #Carla maps use simulation 0,0 as gps origin. 
        #In case, ther is a mismatch, use the following and adjust all scenario points (reload scenarios setup)
        #ref_lat, ref_lon, _ = self.get_gps_origin()
        #map_z =  self.carla_map.get_spawn_points()[0].location.z
        
        #BPs to spawn actors and sensors
        self.blueprint_library = self.world.get_blueprint_library()

    def quit(self):
        pass

    def create_gs_actors(self, vehicles):
        """ Creates gs agents inside Carla.
            @param vehicles: dictionary of type <int, Vehicle>
        """
        if not self.is_connected:
            return

        world = self.world

        #reference z based on athe first spawn point
        map_z =  self.carla_map.get_spawn_points()[0].location.z

        #============== VEHICLES

        i = 0
        for vid,vehicle in vehicles.items():
            vehicle:Vehicle

            #Find vehicle model
            if vehicle.model == '': 
                #carla.ActorBlueprint(id=vehicle.lincoln.mkz_2020,tags=[mkz_2020, lincoln, vehicle])
                blueprints = self.blueprint_library.filter('vehicle.lincoln.mkz_2020') 
            else:
                blueprints = self.blueprint_library.filter(vehicle.model) 
            if len(blueprints) > 0:
                vbp = blueprints[0]
            else:
                vbp = random.choice(self.blueprint_library.filter('vehicle'))
            
            if vbp.has_attribute('color'):
                #color = random.choice(bp.get_attribute('color').recommended_values)
                #print(vbp.get_attribute('color').recommended_values)
                vbp.set_attribute('color', '255,255,255' )

            if USE_SPAWN_POINTS:
                #Using different spawning points as start position
                transform =  self.carla_map.get_spawn_points()[i]  #random.choice(world.get_map().get_spawn_points())
                log.info("Spawn point Location {} GeoLocation {}".format(transform.location, self.carla_map.transform_to_geolocation(transform.location)))
                i+1
            else:
                #manual method (can fail if collision happens)
                transform = self.get_carla_transform(vehicle.state.x,vehicle.state.y,map_z,
                                                    0,vehicle.state.yaw,0)
                
            log.debug(f"Yaw :{transform.rotation.yaw}")
            #= EV
            log.debug("trying to spawn vehicle {} at {} {} yaw {}".format(vid,vehicle.state.x,vehicle.state.y,vehicle.state.yaw))
            if vehicle.type == Vehicle.EV_TYPE:
                if vehicle.bsource == "carla_autopilot":
                    carla_vehicle = world.spawn_actor(vbp, transform)
                    carla_vehicle.set_autopilot(True)
                else:
                    #Do not spawn, but find actor spawned by Carla
                    carla_vehicle = world.spawn_actor(vbp, transform) #temp
            #= SDV, TV, PV or neutral
            else:
                carla_vehicle = world.spawn_actor(vbp, transform) #temp
                carla_vehicle.set_autopilot(False)
            #keep all vehicle actors in local list
            self.carla_vehicles[vehicle.id] = carla_vehicle

            if ENABLE_PHYSICS:
                carla_vehicle.set_simulate_physics(True)
                carla_vehicle.set_enable_gravity(True) #depends on simulate physics

            log.info('created gs vid {} in carla as {}'.format(vehicle.id,carla_vehicle.type_id))
            time.sleep(2)
            
        if ATTACH_SPECTATOR_TO is not None:
            self.set_spectator(self.carla_vehicles[ATTACH_SPECTATOR_TO]) 

        #TODO: get other vehicles and create mirrors on GSServer side

        #vehicles = world.get_actors().filter('vehicle.*')

    def write_server_state(self, tick_count, delta_time, vehicles):
        """ Updates pose data for each gs agent inside Carla.
            @param vehicles:      dictionary of type <int, Vehicle>
        """
        if not self.is_connected:
            return

        cmd = []
        for vid,vehicle in vehicles.items():
            vehicle:Vehicle
            carla_vehicle = self.carla_vehicles[vid]
            curr_transform =  carla_vehicle.get_transform()

            #if vehicle.type == Vehicle.EV_TYPE:
            #    continue

            new_transform = self.get_carla_transform(vehicle.state.x,vehicle.state.y, curr_transform.location.z ,
                                                    0,vehicle.state.yaw,0, curr_transform)
            #carla_vehicle.set_transform(new_transform)
            
            cmd.append(
                carla.command.ApplyTransform(carla_vehicle.id, new_transform))

            #carla_vehicle.apply_control(carla.VehicleControl(throttle = 1, steer = 1))
            #Vehicle control needs a simulated controller +trajectory for that
            #steer = 0.0
            #throttle = logistic(vehicle.state.s_vel/10) if vehicle.state.s_vel > 0 else 0
            #print("Control t{}".format(throttle))
            #cmd.append(
            #    carla.command.ApplyVehicleControl(carla_vehicle.id,carla.VehicleControl(throttle = throttle )))
            #carla_vehicle.set_target_angular_velocity(carla.Vector3D(x=vehicle.state.x_vel,y=vehicle.state.y_vel*-1))

            #log.info("Vehicle {} new pose write".format(vid))

        if ATTACH_SPECTATOR_TO is not None:
            self.update_spectator(self.carla_vehicles[ATTACH_SPECTATOR_TO],cmd)
        
        
        self.client.apply_batch(cmd)
        #self.client.apply_batch([carla.command.ApplyVelocity(x.id, carla.Vector3D(z=5)) for x in vehicles])

        #TickSync.clock_log("carla write dx{:3g} ds{:3g} dt{:3g} read dx {:3g} ".format(deltax,deltas,delta_time,deltax_c ))
        
        if SYNCHRONOUS_MODE:
            self.world.tick()


    def read_carla_state(self,nvehicles):
        """ Reads from pose data from carla for each agent to update physics.
            GS agents update physics, while carla agents get pose exclusively from this method.
        """
        vstates = {}
        disabled_vehicles = []

        if self.is_connected:
            for vid,carla_vehicle in  self.carla_vehicles.items():
                if vid ==1:
                    continue
                transform = carla_vehicle.get_transform()
                vel3d = carla_vehicle.get_velocity()
                vs = self.get_gs_transform(transform,vel3d)
                vstates[vid] = vs
                
                #if not int(is_active):
                #    disabled_vehicles.append(vid)
        
        return vstates, disabled_vehicles



    def get_carla_transform(self,x,y,z,pitch,yaw,roll, curr_transform = None):
        x = x
        y = y*-1
        yaw = yaw*-1 # + 180
        
        if curr_transform:
            #change and return
            curr_transform.location.x = x
            curr_transform.location.y = y
            #curr_transform.location.z = z
            curr_transform.rotation.yaw = yaw
            return curr_transform
        else:
            #WARNING: Performance hit if a new transform is sent to the server
            #pitch, yaw, roll are y, z, x axis
            new_transform = carla.Transform(carla.Location(x,y,z),carla.Rotation(pitch,yaw,roll)) 
            return new_transform
            #print( self.carla_map.transform_to_geolocation(transform.location)) #Converts a location to carla.GeoLocation
            #print(" Map 0,0.0 to carla.GeoLocation {}".format(self.carla_map.transform_to_geolocation(carla.Location(0,0,0)))) #Converts a location to carla.GeoLocation

    def get_gs_transform(self, transform, vel3d):
        vs = VehicleState()
        vs.x = transform.location.x
        vs.y = transform.location.y*-1
        vs.yaw = transform.rotation.yaw * -1
        vs.x_vel = vel3d.x
        vs.y_vel = vel3d.y * -1
        #print("read yaw {} {}".format(transform.rotation.yaw, vs.yaw))
        return vs

    def change_map(self, town):
        self.client.load_world(town)
        time.sleep(2)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()

    def get_gps_origin(self):
        opendrive_str = self.carla_map.to_opendrive()
        xml_tree = ET.fromstring(opendrive_str)
        lat_ref, lon_ref = None, None
        for geo_elem in xml_tree.find('header').find('geoReference').text.split(' '):
            if geo_elem.startswith('+lat_0'):
                lat_ref = float(geo_elem.split('=')[-1])
            elif geo_elem.startswith('+lon_0'):
                lon_ref = float(geo_elem.split('=')[-1])
        log.info("open drive origin is {} {} {}".format(lat_ref, lon_ref, 0))
        return lat_ref, lon_ref, 0.

    def __del__(self):
        self.is_connected = False
        if self.client:
            if SYNCHRONOUS_MODE:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
            self.client.apply_batch([carla.command.DestroyActor(self.carla_vehicles[x]) for x in self.carla_vehicles])
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])
        
    
    #SPECTATOR CAMERA

    def set_spectator(self,carla_vehicle):
        #TODO: 
        #add some flexibility to avoid small changes in yaw
        #adjust yaw and position to focus on vehicle of interest instead of vehicle behind
        #add option to panoramic view based on fixed point (location in geoscenario)
        world = self.world
        spectator = world.get_spectator()
        spectator.set_transform(carla_vehicle.get_transform())

        #create dummy camera sensor attached to vehicle and set as spectator
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_transform = carla.Transform( carla.Location(x=-5, z=2), carla.Rotation(yaw=0, pitch=-15) )
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=carla_vehicle)
        self.actor_list.append(camera)
        self.spectator = spectator        
    
    def update_spectator(self,carla_vehicle,cmd = None):
        if carla_vehicle:
            transform =  carla_vehicle.get_transform()
            transform.location.z += 4 #meters on top
            transform.rotation.pitch -= 15 #look down
            cmd.append(
                carla.command.ApplyTransform(self.spectator.id, transform)
                )
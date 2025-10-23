import sysv_ipc
from core.Actor import *
from SimConfig import *

import logging
log = logging.getLogger(__name__)

# Class defining shared memory structure used to sync with
# an external simulator (client)
class SimSharedMemoryServer(object):

    def __init__(self, ss_shm_key=SHM_KEY, ss_sem_key=SEM_KEY, cs_shm_key=CS_SHM_KEY, cs_sem_key=CS_SEM_KEY):
        # Server state shared memory
        self.ss_shm_key = ss_shm_key
        self.ss_sem_key = ss_sem_key
        # Client state shared memory
        self.cs_shm_key = cs_shm_key
        self.cs_sem_key = cs_sem_key
        try:
            # create a semaphore and SHM for server state (SS)
            self.ss_sem = sysv_ipc.Semaphore(self.ss_sem_key, flags=sysv_ipc.IPC_CREAT, initial_value=1)
            log.debug("ShM SS semaphore created")
            self.ss_shm = sysv_ipc.SharedMemory(self.ss_shm_key, flags=sysv_ipc.IPC_CREAT, mode=int(str(666), 8), size=SHM_SIZE)
            log.debug("ShM SS memory created")
            # create a semaphore and SHM for client state (CS)
            self.cs_sem = sysv_ipc.Semaphore(self.cs_sem_key, flags=sysv_ipc.IPC_CREAT, initial_value=1)
            log.debug("ShM CS semaphore created")
            self.cs_shm = sysv_ipc.SharedMemory(self.cs_shm_key, flags=sysv_ipc.IPC_CREAT, mode=int(str(666), 8), size=SHM_SIZE)
            log.debug("ShM CS memory created")
            log.info("Shared memory and semaphores created")
            self.is_connected = True
        except sysv_ipc.Error:
            log.error("Error creating Shared Memory")
            self.is_connected = False

    def write_server_state(self, tick_count, sim_time, delta_time, origin, vehicles, pedestrians):
        """ Writes to shared memory pose data for each agent.
            @param vehicles:      dictionary of type <int, Vehicle>
            @param pedestrians:   dictionary of type <int, Pedestrian>
            Shared memory format:
                tick_count simulation_time delta_time n_vehicles n_pedestrians
                origin_lat origin_lon origin_alt
                vid v_type l w h x y z vx vy yaw steering_angle
                pid p_type l w h x y z vx vy yaw
                ...
        """
        if not self.is_connected:
            return

        # write tick count, deltatime, numbers of vehicles and pedestrians
        write_str = f"{int(tick_count)} {sim_time} {delta_time} {len(vehicles)} {len(pedestrians)}\n"
        # write origin
        (lat, lon, alt, area) = origin
        write_str += f"{lat} {lon} {alt} {area}\n"
        # write vehicle states, rounding the numerical data to reasonable significant figures
        for svid in vehicles:
            vid, v_type, dimensions, position, velocity, yaw, steering_angle = vehicles[svid].get_sim_state()
            l = round(dimensions[0], 4)
            w = round(dimensions[1], 4)
            h = round(dimensions[2], 4)
            x = round(position[0], 4)
            y = round(position[1], 4)
            z = round(position[2], 4)
            vx = round(velocity[0], 4)
            vy = round(velocity[1], 4)
            yaw = round(yaw * math.pi / 180, 6)
            sa = round(steering_angle, 6)
            write_str += f"{vid} {v_type} {l} {w} {h} {x} {y} {z} {vx} {vy} {yaw} {sa}\n"

        # write pedestrian states, rounding the numerical data to reasonable significant figures
        for spid in pedestrians:
            pid, p_type, dimensions, position, velocity, yaw = pedestrians[spid].get_sim_state()
            l = round(dimensions[0], 4)
            w = round(dimensions[1], 4)
            h = round(dimensions[2], 4)
            x = round(position[0], 4)
            y = round(position[1], 4)
            z = round(position[2], 4)
            vx = round(velocity[0], 4)
            vy = round(velocity[1], 4)
            yaw = round(yaw * math.pi / 180, 6)
            write_str += f"{pid} {p_type} {l} {w} {h} {x} {y} {z} {vx} {vy} {yaw}\n"

        # sysv_ipc.BusyError needs to be caught
        try:
            self.ss_sem.acquire(timeout=0)
            self.ss_shm.write(write_str.encode('utf-8'))
            self.ss_sem.release()
        except sysv_ipc.BusyError:
            log.warning("server state semaphore locked...")
            return
        # log.info("Shared Memory write\n{}".format(write_str))

    def read_client_state(self, nvehicles, npedestrians):
        """ Reads from shared memory pose data for each agent.
            @param nvehicles:         number of vehicles
            @param npedestrians:      number of pedestrians
            Shared memory format:
                tick_count delta_time n_vehicles n_pedestrians
                vid x y z vx vy active
                pid x y z vx vy active
                ...
        """
        # header is [tick_count, delta_time, n_vehicles, n_pedestrians]
        header = None
        vstates = {}
        pstates = {}
        disabled_vehicles = []
        disabled_pedestrians = []

        if not self.is_connected or (nvehicles == 0 and npedestrians == 0):
            return header, vstates, pstates, disabled_vehicles, disabled_pedestrians

        # Read client shared memory
        try:
            # according to docs this raises a BusyError, so it should be handled.
            # but it hasn't been a problem yet?
            self.cs_sem.acquire(timeout=0)
            data = self.cs_shm.read()
            self.cs_sem.release()
        except sysv_ipc.ExistentialError:
            self.is_connected = False
            return header, vstates, pstates, disabled_vehicles, disabled_pedestrians
        except sysv_ipc.BusyError:
            log.error("Cannot acquire client state semaphore...")
            return header, vstates, pstates, disabled_vehicles, disabled_pedestrians

        # Parse client data
        data_str = data.decode("utf-8")
        data_arr = data_str.split('\n')

        if len(data_arr) == 0 or int.from_bytes(data, byteorder='big') == 0:
            # memory is garbage
            return header, vstates, pstates, disabled_vehicles, disabled_pedestrians

        try:
            header_str = data_arr[0].split(' ')
            header = [int(header_str[0]), float(header_str[1]), int(header_str[2]), int(header_str[3])]
            nclient_vehicles = header[2]
            nclient_pedestrians = header[3]
        except Exception as e:
            log.error("Header parsing exception")
            log.error(f"data_arr[0]: {data_arr[0]}")
            log.error(e)
            pass

        try:
            # the client must see the same number of vehicles as server
            if nclient_vehicles == nvehicles:
                for ri in range(1, nvehicles + 1):
                    vid, x, y, z, x_vel, y_vel, active = data_arr[ri].split()
                    vs = VehicleState()
                    vid = int(vid)
                    vs.x = float(x)
                    vs.y = float(y)
                    vs.z = float(z)
                    vs.x_vel = float(x_vel)
                    vs.y_vel = float(y_vel)
                    #Estimating yaw because it is not being published by client.
                    #We use the velocity vectors and only if vehicle is moving at least 10cm/s to avoid noise
                    if (abs(vs.y_vel) > 0.01) or (abs(vs.x_vel) > 0.01):
                        vs.yaw = math.degrees(math.atan2(vs.y_vel,vs.x_vel))
                    vstates[vid] = vs
                    if not int(active):
                        disabled_vehicles.append(vid)
            else:
                log.warning("Client state error: No. client vehicles ({}) not the same as server vehicles ({}).".format(
                    nclient_vehicles,
                    nvehicles
                ))
                log.warning(data_str)
        except Exception as e:
            log.error("VehicleState parsing exception")
            log.error(e)
            pass

        try:
            if nclient_pedestrians == npedestrians:
                for ri in range(nvehicles + 1, nvehicles + 1 + npedestrians):
                    pid, x, y, z, x_vel, y_vel, active = data_arr[ri].split()
                    ps = PedestrianState()
                    pid = int(pid)
                    ps.x = float(x)
                    ps.y = float(y)
                    ps.z = float(z)
                    ps.x_vel = float(x_vel)
                    ps.y_vel = float(y_vel)
                    pstates[pid] = ps
                    if not int(active):
                        disabled_pedestrians.append(pid)
            else:
                log.warning(f"Client state error: No. client pedestrians ({nclient_pedestrians}) not the same as server pedestrians ({npedestrians}).")
                log.warning(data_str)
        except Exception as e:
            log.error("PedestrianState parsing exception")
            log.error(e)
            pass

        return header, vstates, pstates, disabled_vehicles, disabled_pedestrians

    def __del__(self):
        self.is_connected = False
        self.ss_shm.detach()
        self.ss_shm.remove()
        self.ss_sem.remove()
        self.cs_shm.detach()
        self.cs_shm.remove()
        self.cs_sem.remove()

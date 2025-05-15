import sysv_ipc
import math
import time

SHM_KEY = 123456
SEM_KEY = 346565
CS_SHM_KEY = 333943
CS_SEM_KEY = 933433
SHM_SIZE = 2048

# Class defining shared memory structure used to read data from GeoScenario server
class SimSharedMemoryClient(object):

    def __init__(self, ss_shm_key=SHM_KEY, ss_sem_key=SEM_KEY, cs_shm_key=CS_SHM_KEY, cs_sem_key=CS_SEM_KEY):
        self.is_connected = False
        # Server state shared memory
        self.ss_shm_key = ss_shm_key
        self.ss_sem_key = ss_sem_key
        # Client state shared memory
        self.cs_shm_key = cs_shm_key
        self.cs_sem_key = cs_sem_key

        # Semaphore initialization according to: https://semanchuk.com/philip/sysv_ipc/#sem_init
        # Server
        try:
            self.ss_sem = sysv_ipc.Semaphore(self.ss_sem_key, sysv_ipc.IPC_CREX)
        except sysv_ipc.ExistentialError:
            # One of my peers created the semaphore already
            self.ss_sem = sysv_ipc.Semaphore(self.ss_sem_key)
            # Waiting for that peer to do the first acquire or release
            while not self.ss_sem.o_time:
                time.sleep(.1)
        else:
            # Initializing sem.o_time to nonzero value
            self.ss_sem.release()
        # Client
        try:
            self.cs_sem = sysv_ipc.Semaphore(self.cs_sem_key, sysv_ipc.IPC_CREX)
        except sysv_ipc.ExistentialError:
            # One of my peers created the semaphore already
            self.cs_sem = sysv_ipc.Semaphore(self.cs_sem_key)
            # Waiting for that peer to do the first acquire or release
            while not self.cs_sem.o_time:
                time.sleep(.1)
        else:
            # Initializing sem.o_time to nonzero value
            self.cs_sem.release()


    def connect_to_shared_memory(self):
        # Shared memory initialization
        try:
            self.ss_shm = sysv_ipc.SharedMemory(self.ss_shm_key, mode=int(str(666), 8), size=SHM_SIZE)
            self.cs_shm = sysv_ipc.SharedMemory(self.cs_shm_key, mode=int(str(666), 8), size=SHM_SIZE)
            self.is_connected = True
        except sysv_ipc.Error:
            self.is_connected = False
            return False

        return True


    def read_server_state(self):
        """ Reads from shared memory pose data for each agent.
            Shared memory format:
                tick_count simulation_time delta_time n_vehicles n_pedestrians
                origin_lat origin_lon origin_alt
                vid v_type l w h x y z vx vy yaw steering_angle
                pid p_type l w h x y z vx vy yaw
                ...
        """
        header = {}
        origin = {}
        vehicles = []
        pedestrians = []

        if not self.is_connected:
            # Not yet connected to shared memory, maybe the server isn't started yet
            if not self.connect_to_shared_memory():
                return header, origin, vehicles, pedestrians

        # Read server shared memory
        try:
            # according to docs this raises a BusyError, so it should be handled.
            # but it hasn't been a problem yet?
            self.ss_sem.acquire(timeout=0)
            data = self.ss_shm.read()
            self.ss_sem.release()
        except sysv_ipc.ExistentialError:
            self.is_connected = False
            return header, origin, vehicles, pedestrians
        except sysv_ipc.BusyError:
            print("Cannot acquire client state semaphore...")
            return header, origin, vehicles, pedestrians

        # Parse server data
        data_str = data.decode("utf-8")
        data_arr = data_str.split('\n')

        if len(data_arr) == 0 or int.from_bytes(data, byteorder='big') == 0:
            # memory is garbage
            return header, origin, vehicles, pedestrians

        # Parse header
        try:
            header_str = data_arr[0].split(' ')
            header["tick_count"] = int(header_str[0])
            header["simulation_time"] = float(header_str[1])
            header["delta_time"] = float(header_str[2])
            header["n_vehicles"] = int(header_str[3])
            header["n_pedestrians"] = int(header_str[4])
        except Exception as e:
            print("Header parsing exception")
            print("data_arr[0]: %s ", data_arr[0])
            print(e)

        # Parse origin
        try:
            origin_str = data_arr[1].split(' ')
            origin["origin_lat"] = float(origin_str[0])
            origin["origin_lon"] = float(origin_str[1])
            origin["origin_alt"] = float(origin_str[2])
        except Exception as e:
            print("Origin parsing exception")
            print("data_arr[1]: %s ", data_arr[1])
            print(e)

        # Parse vehicles and pedestrians
        try:
            for ri in range(2, header["n_vehicles"] + 2):
                vehicle = {}
                id, type, l, w, h, x, y, z, vx, vy, yaw, str_angle = data_arr[ri].split()
                vehicle["id"] = int(id)
                vehicle["type"] = type
                vehicle["l"] = float(l)
                vehicle["w"] = float(w)
                vehicle["h"] = float(h)
                vehicle["x"] = float(x)
                vehicle["y"] = float(y)
                vehicle["z"] = float(z)
                vehicle["vx"] = float(vx)
                vehicle["vy"] = float(vy)
                vehicle["yaw"] = float(yaw)
                vehicle["steering_angle"] = float(str_angle)
                vehicles.append(vehicle)
        except Exception as e:
            print("VehicleState parsing exception")
            print(e)

        try:
            for ri in range(header["n_vehicles"] + 2, header["n_vehicles"] + 2 + header["n_pedestrians"]):
                pedestrian = {}
                id, type, l, w, h, x, y, z, vx, vy, yaw = data_arr[ri].split()
                pedestrian["id"] = int(id)
                pedestrian["type"] = type
                pedestrian["l"] = float(l)
                pedestrian["w"] = float(w)
                pedestrian["h"] = float(h)
                pedestrian["x"] = float(x)
                pedestrian["y"] = float(y)
                pedestrian["z"] = float(z)
                pedestrian["vx"] = float(vx)
                pedestrian["vy"] = float(vy)
                pedestrian["yaw"] = float(yaw)
                pedestrians.append(pedestrian)
        except Exception as e:
            print("PedestrianState parsing exception")
            print(e)

        return header, origin, vehicles, pedestrians

    def write_client_state(self, tick_count, delta_time, vehicles, pedestrians):
        """ Writes to shared memory of the client pose data for each agent.
            @param vehicles:      [{"id", "x", "y", "z", "vx", "vy", "active"}]
            @param pedestrians:   [{"id", "x", "y", "z", "vx", "vy", "active"}]
            Shared memory format (no origin, sim_time, type, yaw but extra active compared to server state):
                tick_count delta_time n_vehicles n_pedestrians
                vid x y z vx vy active
                pid x y z vx vy active
                ...
        """
        if not self.is_connected:
            return

        # write tick count, deltatime, numbers of vehicles and pedestrians
        write_str = f"{int(tick_count)} {delta_time} {len(vehicles)} {len(pedestrians)}\n"
        # write vehicle states, rounding the numerical data to reasonable significant figures
        for vehicle in vehicles:
            write_str += "{} {} {} {} {} {} {}\n".format(
                vehicle["id"], 
                round(vehicle["x"], 4),
                round(vehicle["y"], 4),
                round(vehicle["z"], 4),
                round(vehicle["vx"], 4),
                round(vehicle["vy"], 4),
                int(vehicle["active"])
            )

        # write pedestrian states, rounding the numerical data to reasonable significant figures
        for pedestrian in pedestrians:
            write_str += "{} {} {} {} {} {} {}\n".format(
                pedestrian["id"], 
                round(pedestrian["x"], 4),
                round(pedestrian["y"], 4),
                round(pedestrian["z"], 4),
                round(pedestrian["vx"], 4),
                round(pedestrian["vy"], 4),
                int(vehicle["active"])
            )

        # sysv_ipc.BusyError needs to be caught
        try:
            self.cs_sem.acquire(timeout=0)
            self.cs_shm.write(write_str.encode('utf-8'))
            self.cs_sem.release()
        except sysv_ipc.BusyError:
            print("server state semaphore locked...")
            return
        # log.info("Shared Memory write\n{}".format(write_str))

    def __del__(self):
        if self.is_connected:
            # Only detach, leave it up to the server to remove shared memory
            self.ss_shm.detach()
            self.is_connected = False
            print("Disconnected from server state shared memory")

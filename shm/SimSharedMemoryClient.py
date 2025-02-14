import sysv_ipc
import time

SHM_KEY = 123456
SEM_KEY = 346565
CS_SHM_KEY = 333943
CS_SEM_KEY = 933433
SHM_SIZE = 2048

# Class defining shared memory structure used to read data from GeoScenario server
class SimSharedMemoryClient(object):

    def __init__(self, ss_shm_key=SHM_KEY, ss_sem_key=SEM_KEY, cs_shm_key=CS_SHM_KEY, cs_sem_key=CS_SEM_KEY):
        # Server state shared memory
        self.ss_shm_key = ss_shm_key
        self.ss_sem_key = ss_sem_key
        # Client state shared memory
        self.cs_shm_key = cs_shm_key
        self.cs_sem_key = cs_sem_key

        # Semaphore initialization according to: https://semanchuk.com/philip/sysv_ipc/#sem_init
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

        print("ShM SS semaphore created")

        if self.connect_to_shared_memory():
            print("Connected to server state shared memory")
        else:
            print("Could not connect to server state shared memory")


    def connect_to_shared_memory(self):
        # Shared memory initialization
        try:
            self.ss_shm = sysv_ipc.SharedMemory(self.ss_shm_key, mode=int(str(666), 8), size=SHM_SIZE)
            self.is_connected = True
        except sysv_ipc.Error:
            self.is_connected = False
            return False

        return True


    def read_server_state(self):
        """ Reads from shared memory pose data for each agent.
            Shared memory format:
                tick_count simulation_time delta_time n_vehicles n_pedestrians
                vid v_type x y z vx vy yaw steering_angle
                pid p_type x y z vx vy yaw
                ...
        """
        header = {}
        vehicles = []
        pedestrians = []

        if not self.is_connected:
            # Not yet connected to shared memory, maybe the server isn't started yet
            if not self.connect_to_shared_memory():
                return header, vehicles, pedestrians

        # Read server shared memory
        try:
            # according to docs this raises a BusyError, so it should be handled.
            # but it hasn't been a problem yet?
            self.ss_sem.acquire(timeout=0)
            data = self.ss_shm.read()
            self.ss_sem.release()
        except sysv_ipc.ExistentialError:
            self.is_connected = False
            return header, vehicles, pedestrians
        except sysv_ipc.BusyError:
            print("Cannot acquire client state semaphore...")
            return header, vehicles, pedestrians

        # Parse server data
        data_str = data.decode("utf-8")
        data_arr = data_str.split('\n')

        if len(data_arr) == 0 or int.from_bytes(data, byteorder='big') == 0:
            # memory is garbage
            return header, vehicles, pedestrians

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

        try:
            for ri in range(1, header["n_vehicles"] + 1):
                vehicle = {}
                id, type, x, y, z, vx, vy, yaw, str_angle = data_arr[ri].split()
                vehicle["id"] = int(id)
                vehicle["type"] = type
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
            for ri in range(header["n_vehicles"] + 1, header["n_vehicles"] + 1 + header["n_pedestrians"]):
                pedestrian = {}
                id, type, x, y, z, vx, vy, yaw = data_arr[ri].split()
                pedestrian["id"] = int(id)
                pedestrian["type"] = type
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

        return header, vehicles, pedestrians


    def __del__(self):
        if self.is_connected:
            # Only detach, leave it up to the server to remove shared memory
            self.ss_shm.detach()
            self.is_connected = False
            print("Disconnected from server state shared memory")

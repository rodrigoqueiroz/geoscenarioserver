import sysv_ipc
from sv.VehicleState import *
from SimConfig import *


# Class defining shared memory structure used to sync with
# an external simulator (client)
class SimSharedMemory(object):

    def __init__(self, ss_shm_key=SHM_KEY, ss_sem_key=SEM_KEY, cs_shm_key=CS_SHM_KEY, cs_sem_key=CS_SEM_KEY):
        self.ss_shm_key = ss_shm_key
        self.ss_sem_key = ss_sem_key
        self.cs_shm_key = cs_shm_key
        self.cs_sem_key = cs_sem_key
        try:
            # create a semaphore and SHM for for Serve State
            self.ss_sem = sysv_ipc.Semaphore(self.ss_sem_key, flags=sysv_ipc.IPC_CREAT, initial_value=1)
            print("ShM SS semaphore created")
            self.ss_shm = sysv_ipc.SharedMemory(self.ss_shm_key, flags=sysv_ipc.IPC_CREAT, mode=int(str(666), 8), size=1024)
            print("ShM SS memory created")
            # create a semaphore and SHM for for Client State
            self.cs_sem = sysv_ipc.Semaphore(self.cs_sem_key, flags=sysv_ipc.IPC_CREAT, initial_value=1)
            print("ShM CS semaphore created")
            self.cs_shm = sysv_ipc.SharedMemory(self.cs_shm_key, flags=sysv_ipc.IPC_CREAT, mode=int(str(666), 8), size=1024)
            print("ShM CS memory created")
            self.is_connected = True
        except sysv_ipc.Error:
            print("Error creating Shared Memory")
            self.is_connected = False

    def write_server_state(self, tick_count, delta_time, vehicles):
        """ Writes to shared memory pose data for each Vehicle.
            @param vehicles:      dictionary of type <int, Vehicle>
        """
        if not self.is_connected:
            return

        # write tick count and deltatime
        write_str = "{} {} {}\n".format(tick_count, delta_time, len(vehicles))
        # write vehicle states
        for svid in vehicles:
            vid, remote, position, velocity, yaw, steering_angle = vehicles[svid].get_sim_state()
            write_str += "{} {} {} {} {} {} {} {} {}\n".format(
                vid, remote, position[0], position[1], position[2],
                yaw, velocity[0], velocity[1], steering_angle)

        self.ss_sem.acquire(timeout=0)
        self.ss_shm.write(write_str.encode('utf-8'))
        self.ss_sem.release()
        # print("Shared Memory write\n{}".format(write_str))

    def read_client_state(self, nvehicles):

        if not self.is_connected:
            return

        if nvehicles == 0:
            return

        try:
            # according to docs this raises a BusyError, so it should be handled.
            # but it hasn't been a problem yet?
            self.cs_sem.acquire(timeout=0)
            data = self.cs_shm.read()
            self.cs_sem.release()
        except sysv_ipc.ExistentialError:
            self.is_connected = False
            return

        data_arr = data.decode("utf-8").split()
        #print(data_arr)

        vstates = {}
        size = 4
        if (len(data_arr) < size):
            #memory is garbage
            return

        for ri in range(0,nvehicles):
            i = ri * size
            vs = VehicleState()
            vid  = int(data_arr[i])
            vs.x = float(data_arr[i+1]) / CLIENT_METER_UNIT
            vs.y = float(data_arr[i+2]) / CLIENT_METER_UNIT
            vs.z = float(data_arr[i+3]) / CLIENT_METER_UNIT
            vstates[vid] = vs

        #print("VSTATES")
        #print(vstates)
        return vstates

    def __del__(self):
        self.is_connected = False
        self.ss_shm.detach()
        self.ss_shm.remove()
        self.ss_sem.remove()
        self.cs_shm.detach()
        self.cs_shm.remove()
        self.cs_sem.remove()

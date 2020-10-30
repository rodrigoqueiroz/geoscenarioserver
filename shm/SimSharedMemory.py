import sysv_ipc
from sv.VehicleState import *
from SimConfig import *
import glog as log

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
            log.info("ShM SS semaphore created")
            self.ss_shm = sysv_ipc.SharedMemory(self.ss_shm_key, flags=sysv_ipc.IPC_CREAT, mode=int(str(666), 8), size=1024)
            log.info("ShM SS memory created")
            # create a semaphore and SHM for for Client State
            self.cs_sem = sysv_ipc.Semaphore(self.cs_sem_key, flags=sysv_ipc.IPC_CREAT, initial_value=1)
            log.info("ShM CS semaphore created")
            self.cs_shm = sysv_ipc.SharedMemory(self.cs_shm_key, flags=sysv_ipc.IPC_CREAT, mode=int(str(666), 8), size=1024)
            log.info("ShM CS memory created")
            self.is_connected = True
        except sysv_ipc.Error:
            log.error("Error creating Shared Memory")
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

        # sysv_ipc.BusyError needs to be caught
        try:
            self.ss_sem.acquire(timeout=0)
            self.ss_shm.write(write_str.encode('utf-8'))
            self.ss_sem.release()
        except sysv_ipc.BusyError:
            log.warn("server state semaphore locked...")
            return
        # log.info("Shared Memory write\n{}".format(write_str))

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
        except sysv_ipc.BusyError:
            log.warn("client state semaphore locked...")
            return

        data_str = data.decode("utf-8")
        data_arr = data_str.split('\n')
        if len(data_arr) == 0:
            # memory is garbage
            return

        # the client must see the same number of vehicles as server
        try:
            nclient_vehicles = int(data_arr[0])
            if nclient_vehicles != nvehicles:
                log.warn("Client state error: No. client vehicles ({}) not the same as server vehicles ({}).".format(
                    nclient_vehicles,
                    nvehicles
                ))
                log.warn(data_str)
                return

            vstates = {}
            # size = 4
            # if (len(data_arr) < size):
            #     #memory is garbage
            #     return

            for ri in range(1, nvehicles + 1):
                vid, x, y, z = data_arr[ri].split()
                # i = ri * size
                vs = VehicleState()
                vid = int(vid)
                vs.x = float(x) / CLIENT_METER_UNIT
                vs.y = -float(y) / CLIENT_METER_UNIT
                vs.z = float(z) / CLIENT_METER_UNIT
                vstates[vid] = vs

        except Exception:
            # log.warn("Client state error: garbage memory:")
            # log.warn(data_str)
            return

        # log.info("VSTATES")
        # log.info(vstates)
        return vstates

    def __del__(self):
        self.is_connected = False
        self.ss_shm.detach()
        self.ss_shm.remove()
        self.ss_sem.remove()
        self.cs_shm.detach()
        self.cs_shm.remove()
        self.cs_sem.remove()

import sysv_ipc

SHM_KEY = 123456
SEM_KEY = 346565

# Class defining shared memory structure used to sync with
# an external simulator 
class SVSharedMemory(object):
    
    def __init__(self, shm_key=SHM_KEY, sem_key=SEM_KEY):
        # TODO: error handling for failing to create shm & sem
        self.shm_key = shm_key
        self.sem_key = sem_key

        # create a semaphore for this memory
        self.sem = sysv_ipc.Semaphore(self.sem_key, flags=sysv_ipc.IPC_CREAT, initial_value=1)
        print("ShM semaphore created")
        self.shm = sysv_ipc.SharedMemory(self.shm_key, flags= sysv_ipc.IPC_CREAT,  mode=int(str(666), 8), size=1024)
        print("ShM memory created")


    def write_vehicle_stats(self, tick_count, delta_time, simulated_vehicles):
        """ Writes to shared memory pose data for each SV in simulated vehicles.
            @param simulated_vehicles:      dictionary of type <int, SV>
        """
        # write tick count and deltatime
        write_str = "{} {}\n".format(tick_count, delta_time)

        # write vehicle states
        for svid in simulated_vehicles:
            vid, position, velocity, yaw, steering_angle = simulated_vehicles[svid].get_sim_state()
            write_str += "{} {} {} {} {} {} {} {}\n".format(
                vid, position[0], position[1], position[2],
                yaw, velocity[0], velocity[1], steering_angle)

        self.sem.acquire(timeout=0)
        self.shm.write(write_str.encode('ascii'))
        self.sem.release()

        # print("Shared Memory write\n{}".format(write_str))

    
    def __del__(self):
        self.shm.detach()
        self.shm.remove()
        self.sem.remove()

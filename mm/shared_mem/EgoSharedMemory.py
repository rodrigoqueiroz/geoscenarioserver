import sysv_ipc
import threading


EGO_SHM_KEY = 333943
EGO_SEM_KEY = 933433


class EgoSharedMemory(object):
    def __init__(self, shm_key=EGO_SHM_KEY, sem_key=EGO_SEM_KEY):
        # TODO: error handling for failing to create shm & sem
        self.shm_key = shm_key
        self.sem_key = sem_key

        self.is_connected = False
        self.attempt_connection()

    
    def attempt_connection(self, interval=1.0):
        """ Will attempt to open the semaphore and shared memory defined by
            EGO_SEM_KEY and EGO_SHM_KEY every 'interval' seconds.
        """
        try:
            # open the semaphore for this memory
            # NOTE: should we be creating the ego shm for consistency?
            self.sem = sysv_ipc.Semaphore(self.sem_key, flags=0)
            print("Ego ShM semaphore opened")
            self.shm = sysv_ipc.SharedMemory(self.shm_key, flags=0)
            print("Ego ShM memory with key {} opened".format(self.shm_key))
            self.is_connected = True
        
        except sysv_ipc.ExistentialError:
            print("Failed to open ego semaphore or shared memory.")
            self.timer = threading.Timer(interval, self.attempt_connection)
            self.timer.start()
    

    def read_memory(self):
        assert self.is_connected
        
        try:
            # according to docs this raises a BusyError, so it should be handled.
            # but it hasn't been a problem yet?
            self.sem.acquire(timeout=0)
            data = self.shm.read()
            self.sem.release()
        except sysv_ipc.ExistentialError:
            self.is_connected = False
            return

        data_arr = data.decode("utf-8").split()

        print("Ego pose:")
        print("x={} y={} z={}".format(data_arr[0], data_arr[1], data_arr[2]))


    def __del__(self):
        self.shm.detach()

from sysv_ipc import *


# Class defining shared memory structure used to sync with
# an external simulator 
class SVSharedMemory(object):
    
    def __init__(self):
        # TODO: unique keys per instance and pass them to unreal
        # TODO: error handling for failing to create shm & sem
        self.shm_key = 123456
        self.sem_key = 346565

        # create a semaphore for this memory
        self.sem = Semaphore(self.sem_key, flags=IPC_CREAT, initial_value=1)
        self.shm = SharedMemory(self.shm_key, flags=IPC_CREAT, size=1024)


    def write(self, position, yaw):
        """ Writes a string to the shared memory
            Params:
                position:       [x, y, z]
                orientation:    [pitch, yaw, roll]
        """

        writestr = "{},{},{},{},{},{}".format(
            position[0], position[1], position[2],
            0, 0, yaw)
        # print(writestr)
        self.sem.acquire(timeout=0)
        self.shm.write(writestr.encode('ascii'))
        self.sem.release()
    
    def __del__(self):
        self.shm.detach()
        self.shm.remove()
        self.sem.remove()

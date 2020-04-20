import sysv_ipc

SHM_KEY_BASE = 123456
SEM_KEY_BASE = 346565

# Class defining shared memory structure used to sync with
# an external simulator 
class SVSharedMemory(object):
    
    def __init__(self, id):
        # TODO: error handling for failing to create shm & sem
        self.shm_key = SHM_KEY_BASE + id
        self.sem_key = SEM_KEY_BASE + id

        # create a semaphore for this memory
        self.sem = sysv_ipc.Semaphore(self.sem_key, flags=sysv_ipc.IPC_CREAT, initial_value=1)
        print("ShM semaphore created")
        self.shm = sysv_ipc.SharedMemory(self.shm_key, flags= sysv_ipc.IPC_CREAT,  mode=int(str(666), 8), size=1024)
        print("ShM memory created")
        

    def write(self, position, yaw):
        """ Writes a string to the shared memory in the format
            "<x> <y> <z> <yaw>"
            Params:
                position:       [x, y, z]
                orientation:    [pitch, yaw, roll]
        """
        writestr = "{} {} {} {}".format(
            position[0], position[1], position[2], yaw)
        # print(writestr)
        self.sem.acquire(timeout=0)
        self.shm.write(writestr.encode('ascii'))
        self.sem.release()
    
    def __del__(self):
        self.shm.detach()
        self.shm.remove()
        self.sem.remove()

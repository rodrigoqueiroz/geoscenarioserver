import numpy as np
from multiprocessing import shared_memory, Process, Lock, Array, Value, cpu_count, current_process
import time
from dataclasses import dataclass
import sys

class TestMultiProcess(object):
    def __init__(self):
        self.lock = Lock()
        self.DIMA = 2
        self.DIMB = 10  

    def f(self, n, a):
        n.value = 3.1415927
        for i in range(len(a)):
            a[i] = -a[i]        
        print(self.DIMA)

    def create_process(self):
        print("Creating shared block")
        #a = Array('d',n_vehicles*vs_size)
        #shm = shared_memory.SharedMemory(create=True, size=sys.getsizeof(arr))
        #Shared Memory Block for Vehicle State
        nv = 1
        r = 5
        c = 3
        arr = Array('i', range(r*c))
        
        print(arr[:])


        for ri in range(r):
            a = ri*c #first for row
            b = a+c  #last index for row
            arr[a+1:b] = [9,9]
            print(arr[a:b])
         

                #for j in range(c):
                #print(i+ (j*i))
                #print( arr[i*j] )

        #for i in range(r):
            #print(i*r)
            #print(arr[ (i*c):c])
            
        #arr[0:3] = [99,99,99] #header
        #for i in range(3):
        #    arr[i*c] = vid
        
        #for ri in range(r):
        #    print(arr[ri:c])

        #num = Value('f', 0.0)
        #arr = Array('i', 10)
        #p = Process(target=self.f, args=(num, arr))
        #p.start()
        #p.join()
        #print(num.value)
        #print(arr[:])

        #print(num.value)
        #print(arr[:])
        
        #a_vs = np.zeros(shape=(self.nvehicles, self.vs_size), dtype=np.float)  
        #shm_vs = shared_memory.SharedMemory(create=True, size=a_vs.nbytes)
        #shdata_vs = np.ndarray(a_vs.shape, dtype=np.float, buffer=shm_vs.buf)
        #shdata_vs[:] = a_vs[:] 
        #Shared Memory Block for Planner
        #a_vp = np.ones(shape=(self.nvehicles, self.vp_size), dtype=np.float)  
        #shm_vp = shared_memory.SharedMemory(create=True, size=a_vp.nbytes)
        #shdata_vp = np.ndarray(a_vp.shape, dtype=np.float, buffer=shm_vp.buf)
        #shdata_vp[:] = a_vp[:]

        #processes = []
        #for i in range(self.nvehicles):
        #    _process = Process(target=self.run_process, args=(shm_vs.name,shm_vp.name,))
        #    processes.append(_process)
        #    _process.start()

        #for _process in processes:
        #    _process.join()

        #print("Final vstate")
        #print(shdata_vs)
        #print("Final vplan")
        #print(shdata_vp)
        
        #shm_vs.close()
        #shm_vs.unlink()

        #shm_vp.close()
        #shm_vp.unlink()
        

    def run_process(self, shm_name_vs, shm_name_vp):
        shm_vs = shared_memory.SharedMemory(name=shm_name_vs)
        shdata_vs = np.ndarray((self.nvehicles, self.vs_size), dtype=np.float, buffer=shm_vs.buf)
        self.lock_vs.acquire()
        shdata_vs[:] = shdata_vs[0] + 1
        time.sleep(3) #pause
        self.lock_vs.release()
        shm_vs.close()

        shm_vp = shared_memory.SharedMemory(name=shm_name_vp)
        shdata_vp = np.ndarray((self.nvehicles, self.vp_size), dtype=np.float, buffer=shm_vp.buf)
        self.lock_vp.acquire()
        shdata_vp[:] = shdata_vp[0] + 1
        time.sleep(1) #pause
        self.lock_vp.release()
        shm_vp.close()
        
        #print(shm_buffer)
        #print(type(shm_buffer))
        #memoryview(vs).nbytes
        #memoryview(vs).tobytes                 


def add_one(shr_name):
    #print(shm_data[:])
    #print(shm_data[2:])

    # shr, np_array = create_shared_block()
    # print('cpu count {}'.format(cpu_count()))
    # processes = []
    # for i in range(2):
    #     _process = Process(target=add_one, args=(shr.name,))
    #     processes.append(_process)
    #     _process.start()
    # for _process in processes:
    #     _process.join()
    # print("Final array")
    # print(np_array[:10])
    # print(np_array[10:])

    existing_shm = shared_memory.SharedMemory(name=shr_name)
    np_array = np.ndarray((2, 2,), dtype=np.int64, buffer=existing_shm.buf)
    lock.acquire()
    np_array[:] = np_array[0] + 1
    lock.release()
    time.sleep(10) # pause, to see the memory usage in top
    print('added one')
    existing_shm.close()



@dataclass
class Trajectory:
    s_coef = []
    d_coef = []
    t = 0                       #duration
    t_start = 0                 #clock start time
    #Boundaries in Frenet Frame
    s_start = [0.0,0.0,0.0]     #start state in s
    d_start = [0.0,0.0,0.0]     #start state in d
    s_target = [0.0,0.0,0.0]    #target state in s
    d_target = [0.0,0.0,0.0]    #target state in d

@dataclass
class VehicleState:
    #sim frame
    x = 0
    y = 0
    z = 0
    #vel
    x_vel = 0
    y_vel = 0
    #acc
    x_acc = 0
    y_acc = 0
    #orientation
    yaw = 0
    steer = 0

if __name__ == "__main__":
    tmp = TestMultiProcess()
    tmp.create_process()
import sysv_ipc
import time


#Shared Memory
SHM_KEY = 123456
SEM_KEY = 346565
CS_SHM_KEY = 333943
CS_SEM_KEY = 933433
SHM_SIZE = 2048

if __name__ == "__main__":
    client_connected = False

    while not client_connected:
        try:
            # get the semaphore and SHM created for for Client State
            cs_sem = sysv_ipc.Semaphore(CS_SEM_KEY)#, flags=sysv_ipc.IPC_CREAT, initial_value=1)
            print("ShM CS semaphore created")
            cs_shm = sysv_ipc.SharedMemory(CS_SHM_KEY, mode=int(str(666), 8), size=SHM_SIZE)
            print("ShM CS memory created")
            client_connected = True
        except sysv_ipc.ExistentialError as e:
            print("Can't connect to client shared memory.")
            time.sleep(2)

    vids = []

    while True:
        if client_connected:
            cs_sem.acquire()
            data = cs_shm.read()
            cs_sem.release()

            # Shared memory format (no origin, sim_time, type, yaw but extra active compared to server state):
            #     tick_count delta_time n_vehicles n_pedestrians
            #     vid x y z vx vy active
            #     pid x y z vx vy active
            data_arr = data.decode("utf-8").split('\n')
            print(data_arr[0])
            for line in data_arr[1:]:
                linedata = line.split()
                if len(linedata) == 0:
                    continue
                try:
                    id = int(linedata[0])
                except ValueError:
                    continue
                print(line)

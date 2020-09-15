import sysv_ipc


#Shared Memory
SHM_KEY = 123456
SEM_KEY = 346565
CS_SHM_KEY = 333943
CS_SEM_KEY = 933433


if __name__ == "__main__":
    # create a semaphore and SHM for for Serve State
    ss_sem = sysv_ipc.Semaphore(SEM_KEY)
    print("ShM SS semaphore connected")
    ss_shm = sysv_ipc.SharedMemory(SHM_KEY)#, mode=int(str(666), 8))
    print("ShM SS memory connected")
    # create a semaphore and SHM for for Client State
    # self.cs_sem = sysv_ipc.Semaphore(self.cs_sem_key)
    # print("ShM CS semaphore created")
    # self.cs_shm = sysv_ipc.SharedMemory(self.cs_shm_key)#, flags=sysv_ipc.IPC_CREAT, mode=int(str(666), 8), size=1024)
    # print("ShM CS memory created")
    # self.is_connected = True

    vids = []

    while True:
        ss_sem.acquire()
        data = ss_shm.read()
        ss_sem.release()

        # print("NEW MESSAGE")

        data_arr = data.decode("utf-8").split('\n')
        tick, delta_time, nvehicles = data_arr[0].split()
        nvehicles = int(nvehicles)

        # read lines corresponding to vehicle ids
        # there may be lots fo garbage lines
        new_id = False
        for line in data_arr[1:nvehicles + 1]:
            linedata = line.split()
            if len(linedata) == 0:
                continue

            try:
                vid = int(linedata[0])
            except Exception:
                continue

            if vid not in vids:
                vids.append(vid)
                new_id = True
            # print(line)
        if new_id:
            print(vids)
            # print(data_arr)

        # print("END MESSAGE")

import sysv_ipc


#Shared Memory
SHM_KEY = 123456
SEM_KEY = 346565
CS_SHM_KEY = 333943
CS_SEM_KEY = 933433


if __name__ == "__main__":
    server_connected = False
    client_connected = False

    try:
        # create a semaphore and SHM for for Client State
        cs_sem = sysv_ipc.Semaphore(CS_SEM_KEY)#, flags=sysv_ipc.IPC_CREAT, initial_value=1)
        print("ShM CS semaphore created")
        cs_shm = sysv_ipc.SharedMemory(CS_SHM_KEY)#, flags=sysv_ipc.IPC_CREAT, mode=int(str(666), 8), size=1024)
        print("ShM CS memory created")
        client_connected = True
    except sysv_ipc.ExistentialError:
        print("Can't connect to client shared memory.")

    vids = []

    while True:

        if client_connected:
            cs_sem.acquire()
            data = cs_shm.read()
            cs_sem.release()

            data_str = data.decode("utf-8")
            print(data_str)

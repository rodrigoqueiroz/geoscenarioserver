# Used to test the shared memory being written by the
# GeoScenario server.

import sysv_ipc
import time

#Shared Memory
SHM_KEY = 123456
SEM_KEY = 346565
CS_SHM_KEY = 333943
CS_SEM_KEY = 933433
SHM_SIZE = 2048

if __name__ == "__main__":
    server_connected = False

    while not server_connected:
        try:
            # create a semaphore and SHM for for Serve State
            ss_sem = sysv_ipc.Semaphore(SEM_KEY)
            print("ShM SS semaphore connected")
            ss_shm = sysv_ipc.SharedMemory(SHM_KEY, mode=int(str(666), 8), size=SHM_SIZE)
            print("ShM SS memory connected")
            server_connected = True
        except sysv_ipc.ExistentialError:
            print("Can't connect to server shared memory.")
            time.sleep(2)

    vids = []
    pids = []

    while True:
        if not server_connected:
            continue

        ss_sem.acquire()
        data = ss_shm.read()
        ss_sem.release()

        # Shared memory format:
        # tick_count simulation_time delta_time n_vehicles n_pedestrians
        # origin_lat origin_lon origin_alt area
        # vid v_type l w h x y z vx vy yaw steering_angle
        # pid p_type l w h x y z vx vy yaw

        data_arr = data.decode("utf-8").split('\n')
        print(f"header: {data_arr[0]}")
        tick, simulation_time, delta_time, nvehicles, npedestrians = data_arr[0].split()
        nvehicles = int(nvehicles)
        npedestrians = int(npedestrians)
        # print(data_arr[1])
        origin_lat, origin_lon, origin_alt, area = data_arr[1].split()

        # read lines corresponding to vehicle ids
        # there may be lots fo garbage lines
        new_vid = False
        print(f"{nvehicles} vehicles")
        for line in data_arr[2:nvehicles+2]:
            linedata = line.split()
            if len(linedata) == 0:
                continue

            try:
                vid = int(linedata[0])
            except Exception:
                continue

            if vid not in vids:
                vids.append(vid)
                print(f"New vid line: {line}")
                new_vid = True
            else:
                print(line)

        if new_vid:
            print(f"New vid: {vids}")
            # print(data.decode("utf-8"))

        new_pid = False
        print(f"{npedestrians} pedestrians")
        for line in data_arr[2+nvehicles:nvehicles+npedestrians+2]:
            linedata = line.split()
            if len(linedata) == 0:
                continue

            try:
                pid = int(linedata[0])
            except Exception:
                continue

            if pid not in pids:
                pids.append(pid)
                print(f"New pid line: {line}")
                new_pid = True
            else:
                print(line)

        if new_pid:
            print(f"New pid: {pids}")

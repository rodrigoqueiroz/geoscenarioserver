from TickSync import *
import time
import datetime
import threading, queue

def my_cont_thread(q):
    print("my thread")
    try:
        req = q.get(False)
        print(req)
    except:
        print("nothing in q")

    #trajectory = ([1,2,3],[1])
    #q_in.put(trajectory)

def my_work_thread(q):
    print("work thread")
    #do something
    time.sleep(5)
    print("work thread done. put q")
    q.put("result")
        
if __name__ == "__main__":
        q = queue.Queue()
        thread1 = threading.Thread(target=my_work_thread,args=([q]))
        print("main thread. start worker")
        thread1.start()
        
        while(True):
            if (thread1.is_alive()):
                pass
            res = q.get()
            print(res)
            break
        




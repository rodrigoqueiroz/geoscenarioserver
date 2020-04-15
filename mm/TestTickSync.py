from TickSync import *
import time
import datetime

if __name__ == "__main__":
    ticksync = TickSync(60,3,True)
    print('SIMULATION START')
    while ticksync.tick():
        time.sleep(0.09)s
        ticksync.resync() 
    print('SIMULATION END')
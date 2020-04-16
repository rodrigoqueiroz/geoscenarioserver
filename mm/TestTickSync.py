from TickSync import *
import time
import datetime

if __name__ == "__main__":
    sync_global   = TickSync(rate=2, block=True, verbose=True, label="EX") #Global Tick blocks if too fasst
    sync_global.set_timeout(10)
    sync_bt     = TickSync(rate=1, block=False, verbose=True, label="BT")
    sync_mm     = TickSync(rate=1, block=False, verbose=True, label="MM")
    print('SIMULATION START')
    #while ticksync.tick():
    while True:
        if sync_global.tick():
            print('Run EX')
            if sync_bt.tick():
                print('plan BT')
            if sync_mm.tick():
                print('plan MM')
        else:
            break

    print('SIMULATION END')

    
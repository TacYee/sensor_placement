#!/usr/bin/env python3

import time
import argparse
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E8"

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default=DEFAULT_URI)
    parser.add_argument("--ccw", action="store_true")
    parser.add_argument("--cw", action="store_true")
    args = parser.parse_args()

    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache="./cache")

    with SyncCrazyflie(args.uri, cf=cf) as scf:
        time.sleep(2.0)

        if args.ccw:
            print("Sending CCW command")
            scf.cf.param.set_value("servotest.cmd", "1")
            time.sleep(1.5)

        elif args.cw:
            print("Sending CW command")
            scf.cf.param.set_value("servotest.cmd", "2")
            time.sleep(1.5)

        else:
            print("Connected. No command sent.")
            time.sleep(2.0)

if __name__ == "__main__":
    main()
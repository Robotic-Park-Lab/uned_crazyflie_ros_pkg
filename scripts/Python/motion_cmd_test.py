# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
import cflib.crtp
from cflib.crazyflie import Crazyflie
import os
import time
import logging


## Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache=os.path.expanduser("~") + "/.cache")
    cf.open_link('radio://0/80/2M/E7E7E7E7E7')
    time.sleep(3)  # give some time to establish connection; 3 sec seems to work

    # Send thrust=0 first so that the crazyflie-firmware's safety protection requirements are met
    cf.commander.send_setpoint(0.0, 0.0, 0, 0)
    time.sleep(0.01)

    for i in range(300):
        cf.commander.send_setpoint(0.0, 0.0, 0, 30000)
        time.sleep(0.01)  # send the setpoint once every 10 ms as per the user guide
    cf.commander.send_stop_setpoint()

    cf.close_link()

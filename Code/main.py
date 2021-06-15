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
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.crazyflie.syncLogger import SyncLogger
import threading

global drone1Working
global drone2Working
drone1Working = False
drone2Working = False
DEFAULT_HEIGHT = 0.5

URI1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
URI2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')


def foodSearch(uri):
    global drone2Working
    global drone1Working
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        if uri == URI1:
            drone1Working = True
        else:
            drone2Working = True

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            time.sleep(1)
            mc.forward(0.5)
            time.sleep(1)
            mc.turn_left(180)
            time.sleep(1)
            mc.forward(0.5)
            time.sleep(1)

        if uri == URI1:
            drone1Working = False
        else:
            drone2Working = False

def foodGet(uri):
    global drone2Working
    global drone1Working
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        if uri == URI1:
            drone1Working = True
        else:
            drone2Working = True

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            time.sleep(1)
            mc.forward(0.5)
            time.sleep(1)
            mc.turn_left(180)
            time.sleep(1)
            mc.forward(0.5)
            time.sleep(1)

        if uri == URI1:
            drone1Working = False
        else:
            drone2Working = False

def dance(uri):
    global drone2Working
    global drone1Working
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        if uri == URI1:
            drone1Working = True
        else:
            drone2Working = True

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            time.sleep(1)
            mc.forward(0.5)
            time.sleep(1)
            mc.turn_left(180)
            time.sleep(1)
            mc.forward(0.5)
            time.sleep(1)

        if uri == URI1:
            drone1Working = False
        else:
            drone2Working = False

def droneOrder(drone, order):
    cflib.crtp.init_drivers()
    if drone == 'uri1':
        drone = URI1
        if drone1Working:
            print('drone 1 bussy')
            quit()
    else:
        drone = URI2
        if drone2Working:
            print('drone 2 bussy')
            quit()
    
    if order == 'foodGet':
        t = threading.Thread(target=foodGet, args=[drone])
        t.start()
    if order == 'dance':
        t = threading.Thread(target=dance, args=[drone])
        t.start()
    if order == 'foodSearch':
        t = threading.Thread(target=foodSearch, args=[drone])
        t.start()

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    droneOrder('uri1','foodSearch')
    time.sleep(5)
    droneOrder('uri2','dance')
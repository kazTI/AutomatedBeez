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
import math

#matrix for testing pathfinding coordinates
matrix = [
  [1, 2, 3, 4],
  [5, 6, 7, 8],
  [9, 10, 11, 12],
  [13, 14, 15, 16],
]


global drone1Working
global drone2Working
global stopDrone1
global stopDrone2
global drone1Position
global drone2Position
drone1Position = 1
drone2Position = 4
stopDrone1 = False
stopDrone2 = False
drone1Working = False
drone2Working = False
DEFAULT_HEIGHT = 0.5
flightTime = 3
gridLenght = 1/2

URI1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
URI2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')

#####       various functions to reset, stabalise and callibrate the crazyflie          #####
def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)

#function to help test pathfinding coordinates by turning the matrix number position into coordinates
def findLocation(number):
    matrix_dim = len(matrix[0])
    item_index = 0
    for row in matrix:
        for i in row:
            if i == number:
                break
            item_index += 1
        if i == number:
            break
    x = int(item_index / matrix_dim)
    y = int(item_index % matrix_dim)

    #the current coordinates of x and y are in meters, becoues thats how the high level commander sees its coordinates
    #but the lenght of the grid squares is not that long, thus they need to be recalculated to fit the lenght of the squares
    x = x*gridLenght
    y = y*gridLenght

    location = [x, y]
    return location

#testing the high level commander
def goTo(position, destination, commander, uri):
        global drone2Working
        global drone1Working
        a = findLocation(position)
        b = findLocation(destination)
        x = b[0] - a[0]
        y = b[1] - a[1]
        commander.takeoff(0.5, flightTime)
        time.sleep(flightTime)
        print('this is fine')
        commander.go_to(x, y, 0, 0, flightTime, relative=True)
        time.sleep(flightTime)
        commander.land(0.0, flightTime)
        time.sleep(flightTime)
        commander.stop()
        if uri == URI1:
            drone1Working = False
        else:
            drone2Working = False


def foodSearch(uri, x, y):
    global stopDrone1
    global stopDrone2
    global drone2Working
    global drone1Working
    # connecting to the crazyflie
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        if uri == URI1:
            drone1Working = True
        else:
            drone2Working = True

        activate_high_level_commander(scf)
        reset_estimator(scf)
        activate_mellinger_controller(scf, False)
        commander = scf.cf.high_level_commander
        print('now im here')
        #controlling crazyflie from thread to test the best way of making it stop when collision is imminent
        goThread = threading.Thread(target=goTo, args=[x, y, commander, uri])
        goThread.start()

        loop = True
        while loop:
            if uri == URI1:
                if stopDrone1 == True:
                    commander.land(0.0, flightTime)
                    time.sleep(flightTime)
                    commander.stop()
                    drone1Working = False
                    stopDrone1 = False
                if drone1Working == False:
                    loop = False
            if uri == URI2:
                if stopDrone2 == True:
                    commander.land(0.0, flightTime)
                    time.sleep(flightTime)
                    commander.stop()
                    drone2Working = False
                    stopDrone2 = False
                if drone2Working == False:
                    loop = False
        

def foodGet(uri):
    global drone2Working
    global drone1Working
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        if uri == URI1:
            drone1Working = True
        else:
            drone2Working = True
        #testing how the crazyflie responds when switching from high level commander to motioncommander during operation
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

#testing the bee dance
def dance(uri):
    global drone2Working
    global drone1Working
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        if uri == URI1:
            drone1Working = True
        else:
            drone2Working = True

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        commander = scf.cf.high_level_commander
        commander.go_to(0, 0, 0, 2*math.pi, flightTime, relative=True)
        time.sleep(flightTime)
        commander.go_to(0, 0, 0, -2*math.pi, flightTime, relative=True)
        time.sleep(flightTime)

        if uri == URI1:
            drone1Working = False
        else:
            drone2Working = False

#testing emergency stop
def stopDrone(drone):
    global stopDrone1
    global stopDrone2
    if drone == 'uri1':
        stopDrone1 = True
    else:
        stopDrone2 = True

#testing crazyflie with threads
def droneOrder(drone, order, x, y):
    cflib.crtp.init_drivers()
    #used for testing how the crazyflie accepts its commands (does it overwrite previous commands, or put them in a que)
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
        t = threading.Thread(target=foodSearch, args=[drone, x, y])
        t.start()

#main for changing test commands
if __name__ == '__main__':
    cflib.crtp.init_drivers()
    destination = 9


    droneOrder('uri1','foodSearch', drone1Position, destination)
    time.sleep(5)
    # droneOrder()
    time.sleep(10)
    stopDrone('uri1')
    #droneOrder('uri2','dance')
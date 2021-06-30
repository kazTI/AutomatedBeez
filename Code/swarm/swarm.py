import sys
import os
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
import threading as td
import multiprocessing as mp
import random as rd
import cflib.crtp
from lib import timer as tm
from lib import credentials as cr
from lib import services as sv
from lib import path_generator as pg
from lib import sim_drone_interface as sdi
from lib import drone_interface as di
import time

from cflib.utils import uri_helper
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

#mqtt related stuf
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient('food', ['food',])
mqttClient.startConnection()

#globals for readycheck
ready = False
ready_counter = 0

#timer related stuf
time_passed = 0
response_time = 0.2 #s
departure_delay = 2
departure_time = 0
timer = tm.Timer()
departure_timer = tm.Timer()

#interface related stuf
interfaces = []
uris = ['radio://0/80/2M/E7E7E7E7E8', 'radio://0/80/2M/E7E7E7E7E7']
crazyflies = 1
sim_drones = 3
sim_drone_start = [[20, 24], [18, 26], [16, 24]]

#food related stuf
global gathering
gathering = [False, False, False]
index = 0
path_generator = pg.PathGenerator(30, 30)
position_handler = sv.PositionHandler()
food_position = None
food_found = False
food_gathering = False

def executeSimDroneMovement(interface, current_position, next_position):
    # fix this when values are higher then normal
    pos = (next_position[0] - current_position[0], next_position[1] - current_position[1])
    if pos == (1, 0):
        interface.move_right()
    if pos == (0, 1):
        interface.move_backwards()
    if pos == (-1, 0):
        interface.move_left()
    if pos == (0, -1):
        interface.move_forward()
    if current_position == next_position:
        interface.stop()

def gatherFood(index, interface):
    if isinstance(interface, sdi.SimDroneInterface):
        print('now its time to gather food!!')
        # print(interface.drone_current_position)
        print(interface.gathering_food)
        while interface.gathering_food:
            if not gathering[index]:
                gathering_timer = tm.Timer()
                gathering[index] = True
                gathering_time = 0
            gathering_time += gathering_timer.tick()
            while gathering_time > response_time:
                if interface.drone_state == 'available':
                    interface.takeoff()
                    if interface.drone_current_position[1] == 1:
                        interface.drone_state = 'gathering'
                        interface.flying = True
                if interface.flying and interface.drone_state == 'gathering':
                    current_position = [interface.drone_current_position[0], interface.drone_current_position[2]]
                    if current_position != food_position:
                        _, _, next_position = path_generator.generateAStarPath(current_position, food_position)
                        executeSimDroneMovement(interface, current_position, next_position)
                    else:
                        interface.land()
                        if interface.drone_current_position[1] == 0:
                            interface.flying = False
                            interface.drone_state = 'returning'
                elif interface.drone_state == 'returning':
                    if not interface.flying:
                        interface.takeoff()
                        if interface.drone_current_position[1] == 1:
                            interface.flying = True
                    else:
                        current_position = [interface.drone_current_position[0], interface.drone_current_position[2]]
                        if current_position != interface.drone_start_position:
                            _, _, next_position = path_generator.generateAStarPath(current_position, interface.drone_start_position)
                            executeSimDroneMovement(interface, current_position, next_position)
                        else:
                            interface.land()
                            if interface.drone_current_position[1] == 0:
                                interface.flying = False
                                interface.drone_state = 'available'
                                interface.gathering_food = False
                gathering_time = 0
    
    elif isinstance(interface, di.DroneInterface):
        interface.drone_state = 'gathering'
        if not interface.flying:
            interface.takeoff()
        loop = True
        command_time = 0
        command_delay = 0.2
        while loop:
            command_time += timer.tick()
            if command_time > command_delay and interface.flying:
                if interface.drone_current_position != food_position and interface.drone_state == 'gathering':
                    _, _, next_position = path_generator.generateAStarPath(interface.drone_current_position, food_position)
                    interface.droneMove(next_position)
                elif interface.drone_current_position == food_position:
                    interface.droneLand()
                    interface.drone_state = 'returning'

                elif interface.drone_state == 'returning' and not interface.flying:
                    interface.takeoff()
                elif interface.drone_state == 'returning' and interface.flying:
                    if interface.drone_current_position != interface.drone_start_position:
                        _, _, next_position = path_generator.generateAStarPath(interface.drone_current_position, interface.drone_start_position)
                        interface.droneMove(next_position)
                    elif interface.drone_current_position == interface.drone_start_position:
                        interface.droneLand()
                        interface.drone_state = 'available'
                        loop = False
                command_time = 0



for i in range(sim_drones):
    drone = 'drone_' + str(i)
    sim_drone_interface = sdi.SimDroneInterface(drone)
    sim_drone_interface.drone_start_position = sim_drone_start[i]
    #interfaces.append(sim_drone_interface)

for i in range(crazyflies):
    crazyflie = 'crazyflie_' + str(i)
    drone_interface = di.DroneInterface(crazyflie, uris[i])
    interfaces.append(drone_interface)
    while drone_interface.connected != 'connected':
        pass



while food_position is None:
    try:
        _, message_food = mqttClient.messages.pop(0)
        food_position = message_food['food']
        print(food_position)
    except:
        pass

while True:
    
    for i in range(len(interfaces)):
        if interfaces[i].ready:
            ready_counter += 1
            if ready_counter == len(interfaces):
                ready = True

    time_passed += timer.tick()
    if time_passed > response_time and ready:
        if not initialized:
            for interface in interfaces:
                # prepare the simulation by giving the drones a start position
                if isinstance(interface, sdi.SimDroneInterface):
                    message = [round(position_handler.getDestCoordX(interface.drone_start_position[0]), 2), 0,
                            round(position_handler.getDestCoordZ(interface.drone_start_position[1]), 2)]
                    interface.drone_current_position = [interface.drone_start_position[0], 0, interface.drone_start_position[1]]
                    interface.start(message)
                elif isinstance(interface, di.DroneInterface):
                    if interface.drone_current_position:
                        interface.drone_start_position = interface.drone_current_position
                        print('drone start position is: ', interface.drone_start_position)
            initialized = True

        if initialized and not assigned_scout and not food_found:
            for drone in interfaces:
                if isinstance(interface, di.DroneInterface):
                    interface = drone
                    assigned_scout = True

        if assigned_scout and not food_found:
            if interface.drone_state == 'available' and not interface.flying:
                interface.takeoff()
                if isinstance(interface, sdi.SimDroneInterface):
                    if interface.drone_current_position[1] == 1:
                        interface.flying = True
                        interface.drone_state = 'scouting'
                elif isinstance(interface, di.DroneInterface):
                    interface.drone_state = 'scouting'

            if isinstance(interface, sdi.SimDroneInterface):
                if interface.flying and interface.drone_state == 'scouting':
                    current_position = [interface.drone_current_position[0], interface.drone_current_position[2]]
                    if current_position != food_position:
                        _, _, next_position = path_generator.generateAStarPath(current_position, food_position)
                        executeSimDroneMovement(interface, current_position, next_position)
                    else:
                        interface.land()
                        if interface.drone_current_position[1] == 0:
                            interface.flying = False
                            interface.drone_state = 'returning'
                elif interface.drone_state == 'returning' and not interface.flying:
                    interface.takeoff()
                    if interface.drone_current_position[1] == 1:
                        interface.flying = True
                elif interface.drone_state == 'returning' and interface.flying:
                        current_position = [interface.drone_current_position[0], interface.drone_current_position[2]]
                        if current_position != interface.drone_start_position:
                            _, _, next_position = path_generator.generateAStarPath(current_position, interface.drone_start_position)
                            executeSimDroneMovement(interface, current_position, next_position)
                        else:
                            if not dancing:
                                dancing_timer = tm.Timer()
                                dancing = True
                            
                            dance_time_passed += dancing_timer.tick()
                            if dance_time_passed < 14:
                                interface.dance()
                            else:
                                interface.land()
                                if interface.drone_current_position[1] == 0:
                                    interface.flying = False
                                    interface.drone_state = 'available'
                                    dance_time_passed = 0
                                    dancing = False
                                    food_found = True
            elif isinstance(interface, di.DroneInterface):
                if interface.drone_state == 'scouting' and interface.flying:
                    if interface.drone_current_position != food_position:
                        print(interface.drone_current_position, food_position)
                        _, _, next_position = path_generator.generateAStarPath(interface.drone_current_position, food_position)
                        print(next_position)
                        interface.droneMove(next_position)
                    else:
                        interface.droneLand()
                        interface.drone_state = 'returning'
                elif interface.drone_state == 'returning' and not interface.flying:
                    interface.takeoff()
                elif interface.drone_state == 'returning' and interface.flying:
                    if interface.drone_current_position != interface.drone_start_position:
                        _, _, next_position = path_generator.generateAStarPath(interface.drone_current_position, interface.drone_start_position)
                        interface.droneMove(next_position)
                    elif interface.drone_current_position == interface.drone_start_position:
                        interface.droneDance()
                        interface.droneLand()
                        interface.drone_start_position = 0
                        interface.drone_state = 'available'
                        food_found = True

        if food_found and not food_gathering:
            if not departure:
                departure_timer = tm.Timer()
                departure = True
                for interface in interfaces:
                    if isinstance(interface, sdi.SimDroneInterface):
                        interface.gathering_food = True


            departure_time += departure_timer.tick()
            if departure_time > departure_delay:
                interface = interfaces[index]
                if isinstance(interface, sdi.SimDroneInterface):
                    gathering_thread = td.Thread(target = gatherFood, args=[index, interface])
                    gathering_thread.start()
                else:
                    if index == len(interfaces) - 1:
                        gatherFood(index, interface)
            index += 1
            if index == len(interfaces):
                food_gathering = True
                index = 0
            departure_time = 0
        time_passed = 0
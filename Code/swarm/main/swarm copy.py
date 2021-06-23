import sys
from threading import Timer
sys.path.append('C:\\Users\\Robert\\Desktop\\TINLabs AS\\Code')
import lib.services as sv
import lib.credentials as cr
import lib.path_generator as pg
import lib.timer as tm
import lib.sim_drone_interface as sdi
import threading as td
import random as rd


def getCurrentPosition(interface):
    message = ''
    if not len(interface.mqttClient.messages) <= 0:
        message = interface.mqttClient.messages.pop(0)
        if isinstance(interface, sdi.DroneInterface):
            _, [_, [x, y, z]] = message
            message = (position_handler.getAbsoluteCoordX(x), y, position_handler.getAbsoluteCoordZ(z))
            print('Message from controller: ', message)
            interface.drone_current_position = message

def executeSimDroneMovement(interface, current_position, next_position):
    # fix this when values are higher then normal
    pos = (next_position[0] - current_position[0], next_position[1] - current_position[1])
    print(pos)
    if pos == (1, 0):
        interface.move_right()
    elif pos == (0, 1):
        interface.move_backwards()
    elif pos == (-1, 0):
        interface.move_left()
    elif pos == (0, -1):
        interface.move_forward()
    else:
        interface.stop()


# initialize drone interfaces and assign start positions for the simulated drones
drone_interfaces = []
drone_interface_0 = sdi.DroneInterface('drone_0')
drone_interface_0.drone_start_position = [20, 24]
drone_interfaces.append(drone_interface_0)
drone_interface_1 = sdi.DroneInterface('drone_1')
drone_interface_1.drone_start_position = [18, 26]
# drone_interfaces.append(drone_interface_1)
drone_interface_2 = sdi.DroneInterface('drone_2')
drone_interface_2.drone_start_position = [16, 24]
# drone_interfaces.append(drone_interface_2)

position_handler = sv.PositionHandler()
path_generator = pg.PathGenerator(30, 30)

commands = [('start', [0.05, 0.005, 0.05]), ('move_right', None), ('move_backwards', None), ('move_left', None), ('move_forward', None)]

initialized = False
assigned_scout = False
food_position = [24, 28]

time_passed = 0
response_time = 0.2 #s
timer = tm.Timer()

running = True
while running:
    # receive messages from every component
    getCurrentPosition(drone_interface_0)
    getCurrentPosition(drone_interface_1)
    getCurrentPosition(drone_interface_2)

    # execution is only done after specific amount of time has passed
    time_passed += timer.tick()
    if time_passed > response_time:

        # prepare the simulation by giving the drones a start position
        if not initialized:
            for interface in drone_interfaces:
                message = [round(position_handler.getDestCoordX(interface.drone_start_position[0]), 2), 0,
                           round(position_handler.getDestCoordZ(interface.drone_start_position[1]), 2)]
                interface.drone_current_position = message
                interface.start(message)
            initialized = True

        if not assigned_scout:
            interface = rd.choice(drone_interfaces)
            assigned_scout = True
        
        if interface.drone_state == 'available':
            interface.takeoff()
        
        if interface.drone_current_position[1] == 1:
            interface.drone_state = 'scouting'
            current_position = [interface.drone_current_position[0], interface.drone_current_position[2]]
            if current_position != food_position:
                _, _, next_position = path_generator.generateAStarPath(current_position, food_position)
                executeSimDroneMovement(interface, current_position, next_position)
            else:
                interface.stop()
            # if interface.drone_state == 'returning':
            #     _, _, next_position = path_generator.generateAStarPath(interface.drone_current_position, food_position)
            #     executeSimDroneMovement(interface, next_position)

        time_passed = 0

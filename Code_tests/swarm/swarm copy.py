import sys
print(sys.path)
sys.path.append('C:\\Users\\Robert\\Desktop\\TINLab AS\\AutomatedBeez\\Code')
import lib.services as sv
import lib.credentials as cr
import lib.path_generator as pg
import lib.timer as tm
import lib.sim_drone_interface as sdi
import lib.drone_interface as di
import threading as td
import random as rd

from cflib.utils import uri_helper


def getCurrentPosition(interface):
    global message
    message = ''
    if isinstance(interface, sdi.SimDroneInterface):
        if not len(interface.mqttClient.messages) <= 0:
            message = interface.mqttClient.messages.pop(0)
            _, [_, [x, y, z]] = message
            message = (position_handler.getAbsoluteCoordX(x), y, position_handler.getAbsoluteCoordZ(z))
            print('Message from controller: ', message)
            interface.drone_current_position = message
    elif isinstance(interface, di.DroneInterface):
        if not len(interface.mqttClient.messages) <= 0:
            _, message = interface.mqttClient.messages.pop(0)
            print('Message from controller: ', message['drone_1'])
            interface.drone_current_position = message['drone_1']
        interface.mqttClient.messages = []


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
sim_drone_interface_0 = sdi.SimDroneInterface('drone_0')
sim_drone_interface_0.drone_start_position = [20, 24]
# drone_interfaces.append(sim_drone_interface_0)
sim_drone_interface_1 = sdi.SimDroneInterface('drone_1')
sim_drone_interface_1.drone_start_position = [18, 26]
# drone_interfaces.append(sim_drone_interface_1)
sim_drone_interface_2 = sdi.SimDroneInterface('drone_2')
sim_drone_interface_2.drone_start_position = [16, 24]
# drone_interfaces.append(sim_drone_interface_2)
drone_3_URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
drone_interface_3 = di.DroneInterface('drone_3', drone_3_URI)
drone_interfaces.append(drone_interface_3)



position_handler = sv.PositionHandler()
path_generator = pg.PathGenerator(30, 30)
commands = [('start', [0.05, 0.005, 0.05]), ('move_right', None), ('move_backwards', None), ('move_left', None), ('move_forward', None)]


initialized = False
assigned_scout = False
food_position = [4, 4]


time_passed = 0
response_time = 0.2 #s
timer = tm.Timer()

running = True
while running:
    # receive messages from every component
    getCurrentPosition(sim_drone_interface_0)
    getCurrentPosition(sim_drone_interface_1)
    getCurrentPosition(sim_drone_interface_2)
    getCurrentPosition(drone_interface_3)

    # execution is only done after specific amount of time has passed
    time_passed += timer.tick()
    if time_passed > response_time:
        print('Drone current position: ', drone_interface_3.drone_current_position)
        # prepare the simulation by giving the drones a start position
        if not initialized:
            for interface in drone_interfaces:
                if isinstance(interface, sdi.SimDroneInterface):
                    message = [round(position_handler.getDestCoordX(interface.drone_start_position[0]), 2), 0,
                               round(position_handler.getDestCoordZ(interface.drone_start_position[1]), 2)]
                    interface.drone_current_position = message
                    interface.start(message)
                elif isinstance(interface, di.DroneInterface):
                    interface.drone_start_position = interface.drone_current_position
                    print('drone start position is: ', interface.drone_start_position)
            initialized = True
                    

        if not assigned_scout:
            interface = rd.choice(drone_interfaces)
            assigned_scout = True

        if message and assigned_scout:
            if interface.drone_state == 'available':
                interface.takeoff()
                
            if isinstance(interface, sdi.SimDroneInterface):
                if interface.drone_current_position[1] == 1:
                    interface.drone_state = 'idle'
                    current_position = [interface.drone_current_position[0], interface.drone_current_position[2]]
                    if current_position != food_position:
                        _, _, next_position = path_generator.generateAStarPath(current_position, food_position)
                        executeSimDroneMovement(interface, current_position, next_position)
                    else:
                        interface.stop()
            elif isinstance(interface, di.DroneInterface):
                if interface.drone_state == 'idle':
                    if interface.drone_current_position != food_position:
                        _, _, next_position = path_generator.generateAStarPath(interface.drone_current_position, food_position)
                        interface.droneMove(next_position)
                    else:
                        interface.droneLand()
                        interface.drone_state = 'returning'
                elif interface.drone_state == 'returning':
                    if interface.drone_current_position != interface.drone_start_position:
                        _, _, next_position = path_generator.generateAStarPath(interface.drone_current_position, interface.drone_start_position)
                        interface.droneMove(next_position)
                    else:
                        interface.droneDance()
                        interface.droneLand()
                        interface.drone_state = 'available'
                
                # if interface.drone_state == 'returning':
                #     _, _, next_position = path_generator.generateAStarPath(interface.drone_current_position, food_position)
                #     executeSimDroneMovement(interface, next_position)

        time_passed = 0

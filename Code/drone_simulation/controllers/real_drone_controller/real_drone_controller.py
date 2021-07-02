from controller import Supervisor
import sys
import os
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir + '..\..\..\..')
import lib.services as sv
import lib.credentials as cr
import drone_simulation.controllers.movement_interface.movement_controllerV2 as mi
import math


# initiate the Supervisor and handler for movement of the drone.
drone = Supervisor()
drone_node = drone.getSelf()
translation_handler = drone_node.getField("translation")
rotation_handler = drone_node.getField("rotation")

# get the time step of the current world.
timestep = int(drone.getBasicTimeStep())

# create communication link using mqtt protocol
clientName = 'real_drone'
assigned_topics = [clientName + '_instructions', 'camera']
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient(clientName, assigned_topics)
mqttClient.startConnection()

drone_speed = 0.2
flight_height = 0.3
simulated_drone_speed = timestep * drone_speed / 1000
rotation_speed = 2*math.pi * timestep / 4000

flying = False
taking_off = False
dancing = False
landing = False
movement = False

execute_command = False

rotation = [0, 1, 0, 0]
reverse = False

x = 0.7
z = 0.5

message = ['', '']
while drone.step(timestep) != -1:
    current_position = drone_node.getPosition()
    # if there are messages process them
    if not len(mqttClient.messages) <= 0:
        message = mqttClient.messages.pop(0)
        topic, message = message
        print('Received message: ', message)

        if message[0] == 'takeoff':
            taking_off = True
        elif message[0] == 'dance':
            dancing = True
        elif message[0] == 'land':
            landing = True
        movement = False

        if message[0] == 'movement' and not taking_off and not dancing and not landing:
            movement = True
        # mqttClient.messages = []


    if movement:
        x, z = message[1]
        x = (x - 305) * (200 / 660) / 100
        z = (z - 24) * (200 / 683) / 100
        # print(x, z)
        current_position[0] = x
        current_position[2] = z
        if not taking_off or not dancing or not landing:
            translation_handler.setSFVec3f(current_position)

    if taking_off:
        if current_position[1] < flight_height:
            current_position[1] += simulated_drone_speed
            translation_handler.setSFVec3f(current_position)
        else:
            taking_off = False

    if dancing:
        if rotation[3] < 2*math.pi and not reverse:
            rotation[3] += rotation_speed
            rotation_handler.setSFRotation(rotation)
            
            if rotation[3] > 2*math.pi:
                reverse = True
        elif rotation[3] > 0 and reverse:
            rotation[3] -= rotation_speed
            rotation_handler.setSFRotation(rotation)
            
            if reverse and round(rotation[3], 4) == 0:
                rotation[3] = 0
                dancing = False
    
    if landing:
        if current_position[1] > 0.005:
            current_position[1] -= simulated_drone_speed
            translation_handler.setSFVec3f(current_position)
        else:
            landing = False

    # print(current_position)

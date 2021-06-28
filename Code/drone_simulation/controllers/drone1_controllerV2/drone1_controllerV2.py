from controller import Supervisor
import sys
import os
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir + '..\..\..\..')
import lib.services as sv
import lib.credentials as cr
import drone_simulation.controllers.movement_interface.movement_controllerV2 as mi


# initiate the Supervisor and handler for movement of the drone.
drone = Supervisor()
drone_node = drone.getSelf()
translation_handler = drone_node.getField("translation")
rotation_handler = drone_node.getField("rotation")


# get the time step of the current world.
timestep = int(drone.getBasicTimeStep())


# create drone controller to control the movement of the drone
movement_controller = mi.MovementController(timestep)


# create communication link using mqtt protocol
clientName = 'drone_1'
assigned_topics = [clientName + '_instructions']
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient(clientName, assigned_topics)
mqttClient.startConnection()
received_message_count = 0
sent_message_count = 0
time_passed = 0
response_time = 200 #ms

# variable used for processing a single path step
command = ''
while drone.step(timestep) != -1:
    current_position = drone_node.getPosition()
    movement_controller.current_position = current_position


    # every 200ms current position is sent to the server
    time_passed += timestep
    if time_passed > response_time:
        absY = 1 if movement_controller.hover else 0
        message = (sent_message_count, (current_position[0], absY, current_position[2]))
        mqttClient.sendPublish(clientName + '_response', message, 0)
        # print('Following message sent: ', message)
        sent_message_count += 1
        time_passed = 0


    # if there are messages process them
    if not len(mqttClient.messages) <= 0:
        message = mqttClient.messages.pop(0)
        (_, message) = message
        command = message['command']
        start_position = message['position']
        mqttClient.messages = []
        print('Received message: ', message)
        # process the command for the movement of the drone
    
    if command == 'start':
        movement_controller.start([start_position[0], 0.005, start_position[2]])
    else:
        movement_controller.processCommands(command)


    # process the movement of the drone (only if drone controller started)
    if movement_controller.started:
        # print(movement_controller.rotating)
        # print(movement_controller.current_position)
        if movement_controller.rotating:
            rotation_handler.setSFRotation(movement_controller.rotation)
        else:
            translation_handler.setSFVec3f(movement_controller.current_position)
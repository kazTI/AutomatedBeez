from controller import Supervisor
import sys
sys.path.append('C:\\Users\\Robert\\Desktop\\TINLabs AS\\Code\\drone_simulation\\controllers')
import lib.services as sv
import lib.credentials as cr
from multiprocessing import Process

# define grid specifications
grid_length = 1
grid_tile_size = 0.1
offset = [0.05, 0.005, 0.05]

# initiate the Supervisor and handler for movement of the drone.
drone = Supervisor()
drone_node = drone.getSelf()
translation_handler = drone_node.getField("translation")

# # get the time step of the current world.
timestep = int(drone.getBasicTimeStep())

# create communication link using mqtt protocol
clientName = 'drone_1'
assigned_topics = [clientName + '_instructions']
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient(clientName, assigned_topics)
mqttClient.startConnection()

# variable used for processing a single path step
processing_message = False

# create PositionHandler to handle movement to received position
positionHandler = sv.PositionHandler([0, 0, 0], offset)
positionHandler.generateAbsoluteGrid(grid_length, grid_tile_size)
current_position = []

# variables used for drone control
flight_height = 0.3                                         # m
drone_speed = 0.1                                           # m/s
simulated_drone_speed = timestep * drone_speed / 1000       # m/0.032s

while drone.step(timestep) != -1:
    # this block will processs all messages received one by one
    if not len(mqttClient.messages) <= 0 and not processing_message:
        print('Total messages: ', len(mqttClient.messages))
        message = mqttClient.messages.pop(0)
        print('Current message: ', message)
        print('')
    
        # absolute position updated based on the received command + position
        x = positionHandler.getDestCoordX(message['body']['position'][0])
        z = positionHandler.getDestCoordZ(message['body']['position'][1])
        print('target coords: ', (x, z))

        processing_message = True

    if processing_message:
        current_position = drone_node.getPosition()
        # print(current_position)
        
        if message['body']['state'] == 'on':
            current_position = [x, positionHandler.start_position[1], z]
            processing_message = False
        elif message['body']['state'] == 'takeoff':
            if current_position[1] <= flight_height:
                current_position[1] += simulated_drone_speed
            else:
                processing_message = False
        elif message['body']['state'] == 'moving':
            if current_position[0] <= x:
                current_position[0] += simulated_drone_speed
            elif current_position[2] <= z:
                current_position[2] += simulated_drone_speed
            else:
                processing_message = False
        elif message['body']['state'] == 'landing':
            if current_position[1] >= positionHandler.start_position[1]:
                current_position[1] -= simulated_drone_speed
            else:
                processing_message = False
        
        translation_handler.setSFVec3f(current_position)
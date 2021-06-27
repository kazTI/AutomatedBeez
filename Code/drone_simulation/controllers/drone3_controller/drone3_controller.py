from controller import Supervisor
import sys
sys.path.append('C:\\Users\\Robert\\Desktop\\TINLabs AS\\Code')
import lib.services as sv
import lib.credentials as cr
import drone_simulation.controllers.drone_controller as dc


# initiate the Supervisor and handler for movement of the drone.
drone = Supervisor()
drone_node = drone.getSelf()
translation_handler = drone_node.getField("position")


# get the time step of the current world.
timestep = int(drone.getBasicTimeStep())


# create drone controller to control the movement of the drone
drone_controller = dc.DroneController(timestep)


# create PositionHandler to handle movement to received position
grid_length = 3
grid_tile_size = 0.1
offset = [0.05, 0.005, 0.05]
positionHandler = sv.PositionHandler([0, 0, 0], offset)
positionHandler.generateAbsoluteGrid(grid_length, grid_tile_size)
current_position = []


# create communication link using mqtt protocol
clientName = 'drone_3'
assigned_topics = [clientName + '_instructions']
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient(clientName, assigned_topics)
mqttClient.startConnection()


# variable used for processing a single path step
process_message = False

while drone.step(timestep) != -1:
    # this block will processs all messages received one by one
    if not len(mqttClient.messages) <= 0 and not process_message:
        message = mqttClient.messages.pop(0)
        print(' ')
        print(f'[{clientName}] Total messages: ', len(mqttClient.messages))
        print(f'[{clientName}] Current message: ', message)
    
        # real position updated based on the received command + position
        x = positionHandler.getDestCoordX(message['body']['position'][0])
        z = positionHandler.getDestCoordZ(message['body']['position'][1])
        print('target coords: ', (x, z))

        process_message = True

    # the function processes the movement of the drone
    if process_message:
        current_position = drone_node.getPosition()
        target_position = (x, z)

        process_message, current_position = drone_controller.process_movement(process_message, message, positionHandler, current_position, target_position)
        translation_handler.setSFVec3f(current_position)
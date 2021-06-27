from controller import Supervisor
import sys
sys.path.append('C:\\Users\\Robert\\Desktop\\TINLabs AS\\Code')
import lib.services as sv
import lib.credentials as cr
import drone_simulation.controllers.drone_controller as dc


# initiate the Supervisor and handler for movement of the drone.
drone = Supervisor()
drone_node = drone.getSelf()
translation_handler = drone_node.getField("translation")


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
clientName = 'drone_1'
assigned_topics = [clientName + '_instructions']
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient(clientName, assigned_topics)
mqttClient.startConnection()


# variable used for processing a single path step
process_message = False

response_time = 100 #ms
time_passed = 0
message_count = 0
while drone.step(timestep) != -1:
    current_position = drone_node.getPosition()
    # this block will processs all messages received one by one
    if not len(mqttClient.messages) <= 0:# and not process_message:
        if not process_message:
            message = mqttClient.messages.pop(0)
    
        # real position updated based on the received command + position
        x = positionHandler.getDestCoordX(message[1]['body']['position'][0])
        z = positionHandler.getDestCoordZ(message[1]['body']['position'][1])
        process_message = True

    # the function processes the movement of the drone
    if process_message:
        target_position = (x, z)
        process_message, current_position = drone_controller.process_movement(process_message, message[1], positionHandler, current_position, target_position)
        translation_handler.setSFVec3f(current_position)
    
    # this block send every 100ms a response to the server with the current position of the drone
    time_passed += timestep
    if time_passed > response_time:
        absX = positionHandler.getAbsoluteCoordX(current_position[0])
        absY = positionHandler.getAbsoluteCoordX(current_position[2])
        mqttClient.sendPublish(clientName + '_response', (message_count, (absX, absY)), 0)
        message_count += 1
        time_passed = 0
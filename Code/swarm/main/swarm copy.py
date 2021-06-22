import sys
from threading import Timer
sys.path.append('C:\\Users\\Robert\\Desktop\\TINLabs AS\\Code')
import lib.services as sv
import lib.credentials as cr
import lib.timer as tm
import lib.sim_drone_interface as sdi
import threading as td


def receive_message(interface):
    message = ''
    if not len(interface.mqttClient.messages) <= 0:
        message = interface.mqttClient.messages.pop(0)
    return message


drone_interface_0 = sdi.DroneInterface('drone_0')
drone_interface_1 = sdi.DroneInterface('drone_1')
drone_interface_2 = sdi.DroneInterface('drone_2')
drone_interface_3 = None

commands = [('start', [0.05, 0.005, 0.05]), ('move_right', None), ('move_backwards', None), ('move_left', None), ('move_forward', None)]

time_passed = 0
response_time = 1 #s
timer = tm.Timer()

running = True
while running:
    # receive messages from every drone
    sim_drone0_message = receive_message(drone_interface_0)
    sim_drone1_message = receive_message(drone_interface_1)
    sim_drone2_message = receive_message(drone_interface_2)
    real_drone3_message = receive_message(drone_interface_3)

    # execution is only done after specific amount of time has passed
    if time_passed > response_time:
        
        
        drone_interface_0.mqttClient.sendPublish(drone_interface_0.drone_publish_topic, commands[0], 0)
        
        
        
        
        time_passed = 0

    time_passed += timer.tick()

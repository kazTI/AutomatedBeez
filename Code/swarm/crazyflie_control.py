import sys
import os
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
import time
from lib import credentials as cr
from lib import services as sv
from lib import timer as tm
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.crazyflie.syncLogger import SyncLogger
import threading as td
import math

class Periferal:
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
        Periferal.wait_for_position_estimator(scf)


    def activate_high_level_commander(scf):
        scf.cf.param.set_value('commander.enHighLevel', '1')


    def activate_mellinger_controller(scf, use_mellinger):
        controller = 1
        if use_mellinger:
            controller = 2
        scf.cf.param.set_value('stabilizer.controller', controller)

credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient('crazyflie_control', ['crazyflie_control',])
mqttClient.startConnection()


DEFAULT_HEIGHT = 0.3
flightTime = 1
message_connect = []
timer = tm.Timer()
flying = False

def createMessageDrone():
    message =   {
                    "connection": 'connected',
                    "flying": True
                }
    return message


def drone_orders(uri, name):
    global message_orders
    global flying
    cflib.crtp.init_drivers()
    message_orders = []
    # t = td.Thread(target=receive_orders, daemon=True)
    # t.start()
    time_passed = 0
    response_time = 0.2
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        connected = createMessageDrone()
        mqttClient.sendPublish('drone_control', connected, 0)
        print(connected['connection'])
        while True:
            while message_orders == []:
                try:
                    _, message = mqttClient.messages.pop(0)
                    message_orders.append(message['orders'])
                    message_orders = message_orders[0]
                except:
                    pass
            time_passed += timer.tick()
            if message_orders:# and time_passed > response_time:
                if message_orders[0] == name:
                    if message_orders[1] == 'takeoff' and not flying:
                        Periferal.activate_high_level_commander(scf)
                        Periferal.reset_estimator(scf)
                        #Periferal.activate_mellinger_controller(scf, False)
                        commander = scf.cf.high_level_commander
                        commander.takeoff(DEFAULT_HEIGHT, 1.5)
                        time.sleep(flightTime)
                        connected = createMessageDrone()
                        mqttClient.sendPublish('drone_control', connected, 0)
                        flying = True
                    if message_orders[1] == 'move' and flying:
                        #Periferal.activate_mellinger_controller(scf, False)
                        commander = scf.cf.high_level_commander
                        commander.go_to(message_orders[2], message_orders[3], 0, 0, flightTime, relative=True)
                        time.sleep(0.2)
                    if message_orders[1] == 'dance':
                        #Periferal.activate_mellinger_controller(scf, False)
                        commander = scf.cf.high_level_commander
                        commander = scf.cf.high_level_commander
                        commander.go_to(0, 0, 0, 2*math.pi, 3*flightTime, relative=True)
                        time.sleep(3*flightTime)
                        commander.go_to(0, 0, 0, -2*math.pi, 3*flightTime, relative=True)
                        time.sleep(3*flightTime)
                    if message_orders[1] == 'land' and flying:
                        #Periferal.activate_mellinger_controller(scf, False)
                        commander = scf.cf.high_level_commander
                        commander.land(0.0, flightTime)
                        time.sleep(flightTime)
                        commander.stop()
                        flying = False
                message_orders = []
            #time_passed = 0
while True:
    while message_connect == []:
        try:
            _, message = mqttClient.messages.pop(0)
            message_connect = message['connection']
        except:
            pass

    drone_orders(message_connect[0], message_connect[1])

import sys
sys.path.append('C:\\Users\\Robert\\Desktop\\TINLabs AS\\Code')
import lib.services as sv
import lib.credentials as cr

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.crazyflie.syncLogger import SyncLogger
import threading
import multiprocessing

global drone1Working
global drone2Working
global stopDrone1
global stopDrone2
global drone1Position
global drone2Position
drone1Position = 1
drone2Position = 4
stopDrone1 = False
stopDrone2 = False
drone1Working = False
drone2Working = False
DEFAULT_HEIGHT = 0.5
flightTime = 5
flightTimeSlow = 50
gridLenght = 1

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
    wait_for_position_estimator(scf)


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)

class DroneInterface:
    def __init__(self, name, uri, position, destination):
        # self.name_dict = {'drone_1':'controller_1',
        #                   'drone_2':'controller_2',
        #                   'drone_3':'controller_3'
        #                  }

        # self.clientName = self.name_dict[name]

        # self.drone_publish_topic = name + '_instructions'
        # self.drone_subscribe_topic = name + '_response'

        # self.credentials = cr.getCredentials()
        # self.mqttClient = sv.MqttClient(self.credentials[0], self.credentials[1], self.credentials[2], self.credentials[3])
        # self.mqttClient.createClient(self.clientName, self.drone_subscribe_topic)
        # self.mqttClient.startConnection()
        
        # self.drone_states = ['on', 'takeoff', 'moving', 'dancing', 'landing', 'off']
        self.URI = uri_helper.uri_from_env(default=uri)
        self.position = position
        self.destination = destination
        self.drone_busy = False

    def createMessage(self, state, nextPosition):
        message =   {
                        "state": self.drone_states[state],
                        "position": nextPosition,
                    }
        return message

    def droneTakeoff(self, URI):
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            activate_high_level_commander(scf)
            reset_estimator(scf)
            activate_mellinger_controller(scf, False)
            commander = scf.cf.high_level_commander
            commander.takeoff(DEFAULT_HEIGHT, flightTime)
            time.sleep(flightTime)

    def droneMove(self, URI, position, destination):
        x = destination[0] - position[0]
        y = destination[1] - position[1]
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            activate_high_level_commander(scf)
            reset_estimator(scf)
            activate_mellinger_controller(scf, False)
            commander = scf.cf.high_level_commander
            commander.go_to(x, y, 0, 0, flightTime, relative=True)
            time.sleep(flightTime)

    def droneDance(self, URI):
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
                mc.turn_left(360)
                time.sleep(1)
                mc.turn_right(360)
                time.sleep(1)

    def droneLand(self, URI):
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            activate_high_level_commander(scf)
            reset_estimator(scf)
            activate_mellinger_controller(scf, False)
            commander = scf.cf.high_level_commander
            commander.land(0.0, flightTime)
            time.sleep(flightTime)
            commander.stop()


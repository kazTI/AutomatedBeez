import sys
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

class Periferal:
    def __init__(self, URI):
        self.URI = URI

    def wait_for_position_estimator(self, scf):
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


    def reset_estimator(self, scf):
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        self.wait_for_position_estimator(scf)


    def activate_high_level_commander(self, scf):
        scf.cf.param.set_value('commander.enHighLevel', '1')


    def activate_mellinger_controller(self, scf, use_mellinger):
        controller = 1
        if use_mellinger:
            controller = 2
        scf.cf.param.set_value('stabilizer.controller', controller)

class DroneInterface:
    DEFAULT_HEIGHT = 0.3
    flightTime = 2
    wait_time = 1.5
    def __init__(self, name, URI):
        cflib.crtp.init_drivers()
        self.URI = URI
        self.periferal = Periferal(self.URI)
        self.cells = 5

        self.drone_start_position = []
        self.drone_current_position = []
        self.drone_destination = None

        self.drone_states = ['available', 'idle', 'scouting', 'returning' 'dancing', 'gathering']
        self.drone_state = self.drone_states[0]
        self.scout = False

        self.clientName = name
        self.drone_subscribe_topics = [self.clientName,]
        self.credentials = cr.getCredentials()
        self.mqttClient = sv.MqttClient(self.credentials[0], self.credentials[1], self.credentials[2], self.credentials[3])
        self.mqttClient.createClient(self.clientName, self.drone_subscribe_topics)
        self.mqttClient.startConnection()

    def createMessage(self, state, nextPosition):
        message =   {
                        "state": self.drone_states[state],
                        "position": nextPosition,
                    }
        return message

    def takeoff(self):
        with SyncCrazyflie(self.URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            self.periferal.activate_high_level_commander(scf)
            self.periferal.reset_estimator(scf)
            self.periferal.activate_mellinger_controller(scf, False)
            commander = scf.cf.high_level_commander
            commander.takeoff(DroneInterface.DEFAULT_HEIGHT, 1.5)
            time.sleep(DroneInterface.flightTime)
            self.drone_state = 'idle'

    def droneMove(self, destination):
        with SyncCrazyflie(self.URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            x = (destination[0] - self.drone_current_position[0]) / self.cells / 2
            y = - (destination[1] - self.drone_current_position[1]) / self.cells / 2
            self.periferal.activate_mellinger_controller(scf, False)
            commander = scf.cf.high_level_commander
            commander.go_to(x, y, 0, 0, DroneInterface.flightTime, relative=True)
            time.sleep(0.2)
            # self.drone_state = 'idle'

    def droneDance(self):
        with SyncCrazyflie(self.URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            with MotionCommander(scf, default_height=DroneInterface.DEFAULT_HEIGHT) as mc:
                self.drone_state = 'dance'
                mc.turn_left(360)
                time.sleep(1)
                mc.turn_right(360)
                time.sleep(1)

    def droneLand(self):
        with SyncCrazyflie(self.URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            self.periferal.activate_mellinger_controller(scf, False)
            commander = scf.cf.high_level_commander
            commander.land(0.0, DroneInterface.flightTime)
            time.sleep(DroneInterface.flightTime)
            commander.stop()


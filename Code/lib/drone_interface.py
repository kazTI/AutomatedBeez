
import sys
import lib.services as sv
import lib.credentials as cr
import math
import os
import logging
import time
import threading as td

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.crazyflie.syncLogger import SyncLogger

class DroneInterface:
    DEFAULT_HEIGHT = 0.3
    flightTime = 2
    def __init__(self, name, URI):
        cflib.crtp.init_drivers()
        self.URI = URI
        self.cells = 10
        self.message = ''
        self.connected = ''
        self.flying_message = False

        self.drone_start_position = []
        self.drone_current_position = []
        self.name = name
        self.ready = False

        
        self.drone_state = 'available'
        self.flying = False

        self.credentials = cr.getCredentials()
        self.orders_mqttClient = sv.MqttClient(self.credentials[0], self.credentials[1], self.credentials[2], self.credentials[3])
        self.orders_mqttClient.createClient('drone_control', ['drone_control',])
        self.orders_mqttClient.startConnection()
        self.connect = URI, name
        self.message_connect = self.createMessageDrone(None, self.connect)
        self.orders_mqttClient.sendPublish('crazyflie_control', self.message_connect, 0)
        while self.connected == '':
            try:
                _, message = self.orders_mqttClient.messages.pop(0)
                self.connected = message['connection']
            except:
                pass
        self.position_mqttClient = sv.MqttClient(self.credentials[0], self.credentials[1], self.credentials[2], self.credentials[3])
        self.position_mqttClient.createClient('drone', ['drone',])
        self.position_mqttClient.startConnection()
        position_thread = td.Thread(target=self.getCurrentPosition, daemon = True)
        position_thread.start()

    def getCurrentPosition(self):
        while True:
            if not len(self.position_mqttClient.messages) <= 0:
                _, self.message = self.position_mqttClient.messages.pop(0)
                self.drone_current_position = self.message[self.name]
                self.position_mqttClient.messages = []
                self.ready = True
                time.sleep(0.2)

    def createMessageDrone(self, orders=None, connect = None):
        message =   {
                        "connection": connect,
                        "orders": orders,
                    }
        return message

    def takeoff(self):
        print('yo')
        orders = self.name, 'takeoff'
        message_drone = self.createMessageDrone(orders, None)
        self.orders_mqttClient.sendPublish('crazyflie_control', message_drone, 0)
        while not self.flying_message:
            try:
                _, message = self.orders_mqttClient.messages.pop(0)
                self.flying_message = message['flying']
            except:
                pass
        self.flying = True

    def droneMove(self, destination):
        x = (destination[0] - self.drone_current_position[0]) / self.cells / 2
        y = - (destination[1] - self.drone_current_position[1]) / self.cells / 2
        orders = self.name, 'move', x, y
        message_drone = self.createMessageDrone(orders, None)
        self.orders_mqttClient.sendPublish('crazyflie_control', message_drone, 0)
        time.sleep(0.2)

    def droneDance(self):
        orders = self.name, 'dance'
        message_drone = self.createMessageDrone(orders, None)
        self.orders_mqttClient.sendPublish('crazyflie_control', message_drone, 0)
        time.sleep(6*DroneInterface.flightTime)

    def droneLand(self):
        orders = self.name, 'land'
        message_drone = self.createMessageDrone(orders, None)
        self.orders_mqttClient.sendPublish('crazyflie_control', message_drone, 0)
        time.sleep(DroneInterface.flightTime)
        self.flying_message = False
        self.flying = False


import sys
sys.path.append('C:\\Users\\Robert\\Desktop\\TINLabs AS\\Code')
import lib.services as sv
import lib.credentials as cr

class DroneInterface:
    def __init__(self, name):
        self.name_dict = {'drone_1':'controller_1',
                          'drone_2':'controller_2',
                          'drone_3':'controller_3'
                         }

        self.clientName = self.name_dict[name]

        self.drone_publish_topic = name + '_instructions'
        self.drone_subscribe_topic = name + '_response'

        self.credentials = cr.getCredentials()
        self.mqttClient = sv.MqttClient(self.credentials[0], self.credentials[1], self.credentials[2], self.credentials[3])
        self.mqttClient.createClient(self.clientName, self.drone_subscribe_topic)
        self.mqttClient.startConnection()
        
        self.drone_states = ['on', 'takeoff', 'moving', 'dancing', 'landing', 'off']

    def createMessage(self, state, nextPosition):
        message =   {
                        "header": {
                            "drone_id": self.clientName
                        },
                        "body": {
                            "state": self.drone_states[state],
                            "position": nextPosition,
                        }
                    }
        return message

    def droneStart(self, nextPosition):
        msg = self.createMessage(0, nextPosition)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 2)

    def droneTakeoff(self, nextPosition):
        msg = self.createMessage(1, nextPosition)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 2)

    def droneMove(self, nextPosition):
        msg = self.createMessage(2, nextPosition)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 2)

    def droneDance(self, nextPosition):
        msg = self.createMessage(3, nextPosition)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 2)

    def droneLand(self, nextPosition):
        msg = self.createMessage(4, nextPosition)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 2)

    def droneShutdown(self, nextPosition):
        msg = self.createMessage(5, nextPosition)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 2)


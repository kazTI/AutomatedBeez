
import lib.services as sv
import lib.credentials as cr

class SimDroneInterface:
    def __init__(self, name):
        # drone name
        self.name_dict = {
                          'drone_0':'controller_0',
                          'drone_1':'controller_1',
                          'drone_2':'controller_2'
                         }
        
        # drone states and commands
        self.drone_states = ['available', 'idle', 'scouting', 'returning' 'dancing', 'gathering']
        self.drone_commands = {
                                0 : 'start',
                                1 : 'stop',
                                2 : 'ascend',
                                3 : 'descend',
                                4 : 'move_forward',
                                5 : 'move_backwards',
                                6 : 'move_left',
                                7 : 'move_right',
                              }
        self.scout = False

        # drone variables
        self.drone_start_position = None
        self.drone_current_position = self.drone_start_position
        self.drone_destination = None
        self.drone_state = self.drone_states[0]

        self.drone_publish_topic = name + '_instructions'
        self.drone_subscribe_topics = [name + '_response', ]
        self.clientName = self.name_dict[name]
        self.credentials = cr.getCredentials()
        self.mqttClient = sv.MqttClient(self.credentials[0], self.credentials[1], self.credentials[2], self.credentials[3])
        self.mqttClient.createClient(self.clientName, self.drone_subscribe_topics)
        self.mqttClient.startConnection()

        # message about drone location are received on subscribed topic
            # topic: name + _response
            # message: topic, body
                # body: message_count, position
                    # position: x, y, z

        # received 'message' example:
            # topic: 'drone_1_response'
            # body: [341, [5, 0, 7]])

    def createMessage(self, command, next_position = None):
        message =   {
                        "command": self.drone_commands[command],
                        "position": next_position,
                    }
        return message

    # prepare the drone for the simulation by placing it at the given position
    def start(self, start_position):
        msg = self.createMessage(0, start_position)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 0)


    # take off the ground
    def takeoff(self):
        msg = self.createMessage(2)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 0)

    # land on the ground
    def land(self):
        msg = self.createMessage(3)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 0)


    # the following commands are indefinite movement to given direction
    def move_forward(self):
        msg = self.createMessage(4)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 0)

    def move_backwards(self):
        msg = self.createMessage(5)

    def move_left(self):
        msg = self.createMessage(6)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 0)

    def move_right(self):
        msg = self.createMessage(7)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 0)


    # stops the indefinite movement of the drone
    def stop(self):
        msg = self.createMessage(1)
        self.mqttClient.sendPublish(self.drone_publish_topic, msg, 0)
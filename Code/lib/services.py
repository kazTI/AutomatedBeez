import json
import paho.mqtt.client as mqtt
import time as tm

class PositionHandler:
    grid_length = 2         # meters
    grid_tile_size = 0.2    # meters
    offset = [0.1, 0.005, 0.1]
    def __init__(self):
        self.grid_cells = int (self.grid_length / self.grid_tile_size)

    def getDestCoordX(self, x):
        if x > self.grid_cells-1 or x < 0:
            raise Exception(f'Please assign grid value from the following range [0 - {self.grid_cells-1}]')
        else:
            dest_coord_x = x * self.grid_length / self.grid_cells + self.offset[0]
            return dest_coord_x

    def getDestCoordZ(self, z):
        if z > self.grid_cells-1 or z < 0:
            raise Exception(f'Please assign grid value from the following range [0 - {self.grid_cells-1}]')
        else:
            dest_coord_z = z * self.grid_length / self.grid_cells + self.offset[0]
            return dest_coord_z

    def getAbsoluteCoordX(self, x):
        return int(self.grid_cells * (x / self.grid_length))

    def getAbsoluteCoordZ(self, z):
        return int(self.grid_cells * (z / self.grid_length))

class MqttClient:
    # initialize MqttClient for communication with Mqtt broker
    def __init__(self, user, password, port, brokerAddress='127.0.0.1', logger=None):
        self.brokerAddress = brokerAddress  # Broker address
        self.port = port                    # Broker port
        self.user = user                    # Connection username
        self.password = password            # Connection password

        self.messages = []

    # create new instace that will communicate the Mqtt broker
    def createClient(self, name, topics = []):
        self.name = name
        self.assigned_topics = topics
        self.client = mqtt.Client(self.name)
        self.client.tls_set(tls_version=mqtt.ssl.PROTOCOL_TLS)
        self.client.username_pw_set(self.user, password=self.password)
        
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        print(f'[{self.name}] Instance of MQTT Client created.')

    # start connection of MqttClient
    def startConnection(self):
        try:
            self.client.connect(self.brokerAddress, port=self.port)

            # create new thread to process network traffic
            self.client.loop_start()
            # tm.sleep(5)
            
            print(f'[{self.name}] Connection established.')
        except:
            print(f'[{self.name}] Connection failed, retrying...')

    # def loopConnection(self):
    #     self.client.loop()

    # method which stops all initialized object to properly close the MqttClient connection
    def stopConnection(self):
        self.client.disconnect()
        self.client.loop_stop()
        print(f'[{self.name}] Connection closed.')
        
    # method which publishes a created message to Mqtt Broker
    def sendPublish(self, topic, message, qos):
        # print('The following message is sent :', '(topic: '+topic+', message: '+str(message)+', qos: '+str(qos)+')')
        self.client.publish(topic, json.dumps(message), qos)

    # callback method which is used on connect to subscribed to perticular channels found within this method
    def on_connect(self, client, userdata, flags, connectionResult):
        errorMessage = f'[{self.name}] Connection refused'
        if connectionResult == 0:
            for topic in self.assigned_topics:
                self.client.subscribe(topic, 0)
        elif connectionResult == 1:
            print(errorMessage, 'incorrect protocol version')
        elif connectionResult == 2:
            print(errorMessage, 'invalid client identifier')
        elif connectionResult == 3:
            print(errorMessage, 'server unavailable')
        elif connectionResult == 4:
            print(errorMessage, 'bad username or password')
        elif connectionResult == 5:
            print(errorMessage, 'not authorised')

    # this method triggers an event when a message is received
    def on_message(self, client, userdata, message):
        # messages are received as MQTTMessage object
        topic = message.topic
        message = str(message.payload.decode('utf-8'))
        message = json.loads(message)
        # print('###### NEW MESSAGE ######')
        # print(f'[{self.name}] Message received on {topic}: ', message)
        self.messages.append((topic, message))

if __name__ == '__main__':
    x = 1
    y = 1 
    z = 3
    offset = [0.05, 0.005, 0.05]
    ph = PositionHandler([0, 0, 0], offset)
    ph.generateAbsoluteGrid(1, 0.1)
    ph.updateAbsolutePosition(x, z)
    ph.updateAbsolutePosition(x, y, z)

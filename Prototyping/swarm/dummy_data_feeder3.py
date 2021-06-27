import sys
sys.path.append('c:\\Users\\Robert\\Desktop\\TINLabs AS\\Code')
import lib.services as sv
import lib.credentials as cr
import time as tm

def createMessage():
    message =   {
                "header": {
                    "drone_id": drone_id
                },
                "body": {
                    "state": state,
                    "position": position,
                }
            }
    return message

drone_id = 'drone_3'
state = 'on'
position = [1, 7]
message = createMessage()

topic = drone_id + '_instructions'
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient('dummy_data_feeder', topic)
mqttClient.startConnection()

# message to let the drone take off
mqttClient.sendPublish(topic, message, 1)
tm.sleep(0.1)

state = 'takeoff'
message = createMessage()
mqttClient.sendPublish(topic, message, 1)
tm.sleep(0.1)

state = 'moving'
message = createMessage()
run = 0
while run < 5:
    position[0] += 1
    mqttClient.sendPublish(topic, message, 1)
    run += 1
    tm.sleep(0.1)

state = 'landing'
message = createMessage()
mqttClient.sendPublish(topic, message, 2)
tm.sleep(0.1)
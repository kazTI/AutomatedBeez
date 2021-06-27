import lib.services as sv
import lib.credentials as cr
import lib.timer as tm

clientName = 'test_client'
assigned_topics = ['drone_1_instructions', 'drone_2_instructions', 'drone_3_instructions']
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient(clientName, assigned_topics)
mqttClient.startConnection()

def createMessage(state, nextPosition):
    message =   {
                    "header": {
                        "drone_id": clientName
                    },
                    "body": {
                        "state": state,
                        "position": nextPosition,
                    }
                }
    return message

timer = tm.Timer()
msg_nr = 1
time_passed = 0
publish_timer = 1

pos_y = 1
for topic in assigned_topics:
    message = createMessage('on', (msg_nr, pos_y))
    mqttClient.sendPublish(topic, message, 0)
    pos_y += 2

while msg_nr < 10:
    pos_y = 1
    for topic in assigned_topics:
        if msg_nr == 1:
            message = createMessage('takeoff', (msg_nr, pos_y))
        else:
            message = createMessage('moving', (msg_nr, pos_y))
        mqttClient.sendPublish(topic, message, 0)
    
    msg_nr += 1
    timer.postpone(1)

pos_y = 1
for topic in assigned_topics:
    message = createMessage('landing', (msg_nr-1, pos_y))
    mqttClient.sendPublish(topic, message, 0)
    pos_y += 1
timer.postpone(1)
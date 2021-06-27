import lib.services as sv
import lib.credentials as cr
import time as tm

clientName = 'receive_client'
assigned_topics = ['drone_1_response', 'drone_2_response', 'drone_3_response'] #'drone_1_instructions', 'drone_2_instructions', 'drone_3_instructions',]
credentials = cr.getCredentials()
mqttClient = sv.MqttClient(credentials[0], credentials[1], credentials[2], credentials[3])
mqttClient.createClient(clientName, assigned_topics)
mqttClient.startConnection()

time = tm.time()
started = False

sent_message_count = 0
time_passed = 0
response_time = 0.3
z = 0
x = 0
while True:
    if not len(mqttClient.messages) <= 0:
        message = mqttClient.messages.pop(0)
        topic, [message_count, [x, y, z]] = message
        # print(message)
        # print(f'[{clientName}] Received following message: {message[1]}')

    

    old_time = time
    time = tm.time()
    time_passed += time - old_time
    if time_passed > response_time:
        if not started:
            mqttClient.sendPublish('drone_1_instructions', ('start', (0.15, 0.15)), 0)
            started = True
            ascended = False
        # elif not ascended:
        #     mqttClient.sendPublish('drone_1_instructions', ('ascend', (1, 1)), 0)
        #     ascended = True if y == 1 else False
        elif started:
            print(message[1])
            if round(x, 2) == 0.15:
                mqttClient.sendPublish('drone_1_instructions', ('move_right', None), 0)
            elif (round(x) == 0.5 or round(x, 2) == 0.51 or round(x, 2) == 0.52) and z == 0.15:
                mqttClient.sendPublish('drone_1_instructions', ('move_backwards', None), 0)
            elif (z == 0.5 or round(z, 2) == 0.51 or round(x, 2) == 0.52) and x > 0.2:
                mqttClient.sendPublish('drone_1_instructions', ('move_left', None), 0)
            elif x <= 0.2 and z >= 0.5:
                mqttClient.sendPublish('drone_1_instructions', ('move_forward', None), 0)
            # else:
            #     mqttClient.sendPublish('drone_1_instructions', 'stop', 0)
        # print('the following message is sent:', sent_message_count)

        sent_message_count += 1
        time_passed = 0
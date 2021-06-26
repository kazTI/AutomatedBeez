import sys
sys.path.append('C:\\Users\\Robert\\Desktop\\TINLabs AS\\Code')
import lib.services as sv
import lib.credentials as cr
import drone_interface as di
import timer as tm

controller1 = di.DroneInterface('drone_1')
controller2 = di.DroneInterface('drone_2')
controller3 = di.DroneInterface('drone_3')
controllers = [controller1, controller2, controller3]


timer = tm.Timer()
y = 1
running = True
for controller in controllers:
    controller.droneStart((1, y))
    y += 2
    print(y)
    timer.postpone(2)
# timer.postpone(2)

y = 1
for controller in controllers:
    controller.droneTakeoff((1, y))
    timer.postpone(2)

for controller in controllers:
    x = 1
    for i in range(2):
        x += 1
        controller.droneMove((x, y))
        timer.postpone(1)


    controller.droneLand((x, y))
    y += 2
    timer.postpone(1)



# print(sys.path)
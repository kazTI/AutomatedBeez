"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor

# create the Robot instance.
robot = Supervisor()
supervisorNode = robot.getSelf()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

trans = supervisorNode.getField("translation")

pos = [-0.05, 0.0005, 0.25]
ascending = True
descending = False
while robot.step(timestep) != -1:
    if pos[1] <= 0.3 and ascending:
        pos[1] += 0.005
        if pos[1] >= 0.3:
            ascending = False
    elif pos[0] <= 0.35:
        pos[0] += 0.005
    else:
        if pos[1] >= 0.0005:
            pos[1] -= 0.005
    
    # if descending and not ascending:
        # pos[1] -= 0.005
        
    trans.setSFVec3f(pos)
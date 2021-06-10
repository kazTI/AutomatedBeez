
"""
This script shows the basic use of the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

The MotionCommander uses velocity setpoints.

Change the URI variable to your Crazyflie configuration.

# TODO: schrijf een functie voor aanpassen van coordinaten / nieuwe objecten
# TODO: schrijf een functie voor weghalen van obkjecten 
# TODO: schrijf een functie voor het wegscchrijven van gebruikte paden
# TODO: schrijf een functie voor het dynamish wegahalen van paden nadat ze zijn gevlogen
# TODO: schrijf een functie voor het verplaatsen van end goal
# TODO: schrijf een functie voor verweideren van een end goal

#potentie?


"""
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper


import pathfinding
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')



# creates a grid of given dimensions and borders it up 
def create_grid(row, column):

  map = [[0 for x in range(row)] for y in range(column)]

  for i in range(column):
    for j in range(row):
      if(i == 0 or i ==row):
        map[i][j] = 0
      elif(j == 0 or j == column):
        map[i][j] = 0
      else:
        map[i][j] = 1

  return map

def create_obstacle(x, y):
  grid[x][y] = 0

def remove_obstacle(x, y):
  grid[x][y] = 1

def mark_path():
  for coords in path[0]:
    create_obstacle(coords[0], coords[1])

def current_pos(x,y):
  return grid.node(x,y)

def end_pos(x,y):
  return grid.node(x,y)

def get_diffrence(first, next):
  x_dif = first[0] - next[0]
  y_dif = first[1] - next[1]
  return (x_dif, y_dif)

def follow_path(mc):
  print(grid.grid_str(path=path, start=current_pos(1,1), end=end_pos(8,11)))
  mark_path()


  print("moving")
  for index, coords in enumerate(path[0]):
    if(index != 0):
      move = get_diffrence(coords, last)
      if(move == (0, 1)):        
        mc.forward(0.1)
      elif(move == (0,-1)):        
        mc.back(0.1)
      elif(move == (1, 0)):        
        mc.right(0.1)
      elif(move == (-1,0)):        
        mc.left(0.1)
    
    last = coords
    remove_obstacle(last[0], last[1])
    print(grid)



# create the grid for global use to share variables
grid = Grid(matrix=create_grid(18,18))
finder = AStarFinder(diagonal_movement=DiagonalMovement.never)

path = finder.find_path(current_pos(11,17), end_pos(4,4), grid)



# print('operations:', runs, 'path length:', len(path))
# print(grid.grid_str(path=path, start=current_pos(1,1), end=end_pos(8,11)))
print(path[0], len(path[0]))

# if __name__ == '__main__':
#     grid = Grid(matrix=map)
#     # Initialize the low-level drivers
#     cflib.crtp.init_drivers()

#     with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
#         # We take off when the commander is created
#         with MotionCommander(scf) as mc:
#           # mc.take_off(height=20)
#           follow_path(mc)
          


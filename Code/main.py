# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
# from cflib.utils import uri_helper

# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# # create the grid for global use to share variables
# grid = Grid(matrix=create_grid(30,30))
# finder = AStarFinder(diagonal_movement=DiagonalMovement.never)



# if __name__ == '__main__':
#     grid = Grid(matrix=map)
#     # Initialize the low-level drivers
#     cflib.crtp.init_drivers()

#     with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
#         # We take off when the commander is created
#         with MotionCommander(scf) as mc:
#           # mc.take_off(height=20)
#           follow_path(mc)
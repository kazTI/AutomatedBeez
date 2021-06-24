class DroneController:
    def __init__(self, timestep, flight_height = 0.3, drone_speed = 0.1):
        # variables used for drone control
        self.flight_height = flight_height                                          # m
        self.drone_speed = drone_speed                                              # m/s
        self.simulated_drone_speed = timestep * self.drone_speed / 1000             # m/0.032s

    def process_movement(self, process_message, message, positionHandler, position, target_position):
        current_position = position
        # print(current_position)

        if message['body']['state'] == 'on':
            current_position = [target_position[0], positionHandler.start_position[1], target_position[1]]
            process_message = False

        elif message['body']['state'] == 'takeoff':
            if current_position[1] <= self.flight_height:
                current_position[1] += self.simulated_drone_speed
            else:
                process_message = False

        elif message['body']['state'] == 'moving':
            if current_position[0] <= target_position[0]:
                current_position[0] += self.simulated_drone_speed
            elif current_position[2] <= target_position[1]:
                current_position[2] += self.simulated_drone_speed
            else:
                process_message = False
                
        elif message['body']['state'] == 'landing':
            if current_position[1] >= positionHandler.start_position[1]:
                current_position[1] -= self.simulated_drone_speed
            else:
                process_message = False

        return process_message, current_position
class MovementController:
    def __init__(self, timestep, flight_height=0.3, drone_speed=0.1):
        self.started = False
        self.flight_height = flight_height                                          # m
        self.drone_speed = drone_speed                                              # m/s
        self.simulated_drone_speed = timestep * self.drone_speed / 1000             # m/0.0032s
        self.current_position = []
        self.hover = False

    def processCommands(self, command):
        if command == 'stop':
            self.current_position = self.stop()
        elif command == 'ascend':
            self.hover = self.ascend()
        elif command == 'descend':
            self.hover = self.descend()
        elif command == 'move_forward':
            self.current_position = self.move_forward()
        elif command == 'move_backwards':
            self.current_position = self.move_backwards()
        elif command == 'move_left':
            self.current_position = self.move_left()
        elif command == 'move_right':
            self.current_position = self.move_right()

    def start(self, start_position):
        self.started = True
        self.current_position = start_position

    def ascend(self):
        if self.current_position[1] < self.flight_height:
            self.current_position[1] += self.simulated_drone_speed
        else:
            self.hover = True
        return self.hover

    def descend(self):
        if self.current_position[1] > 0.005:
            self.current_position[1] -= self.simulated_drone_speed
        else:
            self.hover = False
            self.started = False
        return self.hover

    
    def move_forward(self):
        self.current_position[2] -= self.simulated_drone_speed
        return self.current_position

    def move_backwards(self):
        self.current_position[2] += self.simulated_drone_speed
        return self.current_position

    def move_left(self):
        self.current_position[0] -= self.simulated_drone_speed
        return self.current_position

    def move_right(self):
        self.current_position[0] += self.simulated_drone_speed
        return self.current_position


    def stop(self):
        self.current_position = self.current_position
        return self.current_position
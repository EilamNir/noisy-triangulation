
from utils.matrix_helpers import transpose_matrix2d
import numpy as np
import math

class generate_path:
    def __init__(self, initial_phi, initial_speed_xy, initial_speed_z, initial_position, time_res):
        # generate_path - generate a path made out of intervals of different types
        #
        # Syntax: obj = generate_path(initial_position, initial_theta, initial_phi, initial_speed, TimeRes)
        #
        # Generate a path of different intervals.
        # The path will start from initial_position (defaults to [0,0,0]),
        # and will grow towards [theta, phi] (defaults to [theta=0, phi=0]).
        # The initial speed of the target is indicated at initial_speed (defaults to 50).
        # TimeRes defaults to 0.5.
        self.phi = initial_phi
        self.position = initial_position
        self.speed_xy = initial_speed_xy
        self.speed_z = initial_speed_z
        self.acceleration = 0
        self.time_res = time_res
        self.path = initial_position
        self.time = [0]


    def add_straight_interval(self, interval_duration):
    #add_straight_interval - add a straight interval to the path
    #
    # Syntax: add_straight_interval(interval_duration)
    #
    # Add a straight interval to the path.
    # interval_duration is the total length of the path to add.
    # If any of theta or phi are given, the heading will change before the interval.
        self.advance_path(interval_duration, 0)


    def add_xy_turn_interval(self, interval_duration, rotation_speed):
    #add_xy_turn_interval - add a turn in the x-y plane, with a constant theta
    #
    # Syntax: add_xy_turn_interval(obj, rotation_speed)
    #
    # Add a turn interval, which has a constant radius, and can climb or descend in the z axis.
        self.advance_path(interval_duration, rotation_speed)


    def add_3d_turn_interval(self, interval_duration, rotation_speed, angle):
    #add_3d_turn_interval - add a turn in 3 dimensions
    #
    # Syntax: add_3d_turn_interval(obj, interval_duration, rotation_speed, angle)
    #
    # Add a turn interval, which has a constant radius.
    # The angle controls the direction of the turn.
    # angle of 0 means pitching up, and angle of 90 means turning fully right.
        self.advance_path(interval_duration, rotation_speed * cos(angle), rotation_speed * sin(angle))


    def advance_path(self, interval_duration, phi_rotation_speed):
    #advance_path - internal function to advance the path
    #
    # Syntax: advance_path(interval_duration)
    #
    # advance the path using the current position and speed
        t = np.arange(self.time_res, interval_duration, self.time_res)
        for i in t:
            self.position[0:2] = self.position[0:2] + (self.time_res * self.speed_xy * transpose_matrix2d(self.phi)).T
            self.position[2] = self.position[2] + (self.time_res * self.speed_z)
            self.path = np.vstack((self.path, self.position))
            self.time = np.vstack((self.time, self.time[-1] + self.time_res))

            self.phi = (self.phi - self.time_res * phi_rotation_speed) % 2*math.pi





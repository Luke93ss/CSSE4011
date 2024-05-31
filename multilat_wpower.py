# Multi-lateration by least squares using RSSI powers
# Author: Robert M

import numpy as np
import math as m
from scipy.optimize import least_squares
from dataclasses import dataclass
import random as r

N = 2

beacon_positions = np.array([
        [0, 0],
        [2, 0],
        [4, 0],
        [4, 2],
        [4, 4],
        [2, 4],
        [0, 4],
        [0, 2]
        ])


@dataclass
class Position:
    x:float
    y:float


def position_from_rssi(rssi_powers, initial_guess, beacon_positions):

    def objective_function(p):
        return np.array([power(p, pos) for pos in beacon_positions]) - rssi_powers

    def power(p, beacon_pos):
        cartDist = np.sqrt(np.sum((p - beacon_pos)**2))
        
        if cartDist <= 0:
            cartDist = 0.01
        

        #-53
        rssi = -53 -m.log10(cartDist) * 10  * N 
        return rssi

    # Solve the least squares problem to find the receiver's position
    result = least_squares(objective_function, initial_guess)

    # Extract the position of the receiver
    receiver_position = result.x
    x_new, y_new = float(receiver_position[0]), float(receiver_position[1])
 
    if x_new < 0.0:
        x_new = 0
    if x_new > 4.0:
        x_new = 4.0
    if y_new < 0.0:
        y_new = 0.0
    if y_new > 4.0:
        y_new = 4.0


    return (x_new, y_new)

def generate_distances(p, beacon_positions):
    # we generate distances with a little bit of wobble, cause the real world is messy
    distances = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

    rssi_powers = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

    # ((r.random()*2) - 1) / 10

    for d in range(np.size(distances)):
        distances[d] = m.sqrt((p.x - beacon_positions[d][0] + ((r.random()*2) - 1) / 5) ** 2 
                            + (p.y - beacon_positions[d][1] + ((r.random()*2) - 1) / 5) ** 2)
        
        if (distances[d] == 0):
            distances[d] = 0.1

        
        rssi_powers[d] = -m.log10(distances[d]) * 10 * N
        print("Generated RSSI to beacon", d,"@",beacon_positions[d], "is:", rssi_powers[d])

    return rssi_powers






if __name__ == '__main__':


    p = Position(r.random() * 4, r.random() * 4)

    print("Test point created at [{:1.4},{:1.4}]".format(p.x,p.y))

    #powers = generate_distances(p, beacon_positions)

    powers = np.array([-63, -68, -68, -60, -61, -58, -55, -55])

    #print("      Test position at: [{:1.4},{:1.4}]".format(p.x,p.y))

    tp = position_from_rssi(powers, [2,2], beacon_positions)
    print("Calculated position at:", "[{:1.4},{:1.4}]".format(tp[0],tp[1]))
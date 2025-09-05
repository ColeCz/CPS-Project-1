# Simulator Skeleton File - Project 1
# CS-7639-O01/ECE-8823-OCY Fall 2025 
# This file provides the bare - bones requirements for interacting with the Robotarium.
# Note that this code won't actually run.  You'll have to insert your own algorithm!
# If you want to see some working code, check out the 'examples' folder.

# NOTE:
# The Bot is controlled using unicycle dynamics, where theta is the angle of the body frame of the robot with respect to the frame defined on the testbed
# The units of the velocity inputs and are m/s and rad/s, respectively
# An overhead sensor is used to return pose data for the robot, The position coordinates (x, y) are given in meters, and 𝜃 is in radians with range [-π, π]
# theta is increasing (with positive vals)
# iterations seem to run for 0.033 seconds
# negative theta gives clockwise rotation 
# positive theta gives counterclockwise rotation

# NOTE: Report info:
# Report Q2: t1_iterations = target1_distance * meters_per_second second_per_iteration * 2 # NOTE: "2" is currently a magic number until I find a variable name for a value the denominator of the meters per second, should be "target1_distance * iterations_per_second / mps"

import time
import math

import numpy as np
from scipy.io import savemat

import rps.robotarium as robotarium
from rps.utilities.barrier_certificates import create_unicycle_barrier_certificate_with_boundary

# Get Robotarium object used to communicate with the robots / simulator
N = 1
r = robotarium.Robotarium(number_of_robots=N, show_figure=True)
data = []

# Select the number of iterations for the experiment.
iterations = 3000 # Do not change

# Create a boundary barrier
uni_barrier_certificate = create_unicycle_barrier_certificate_with_boundary()

# Other important variables
target1 = np.array([[-0.5],[0],[0]]) # x, y, theta
target2 = np.array([[.5],[0],[0]])


k = 1

# ######################### Place Static Variables Here ##################
# ############### Do not modify anything outside this area ###############
# var = 0;
pi_by_two = 1.57079632679 # seems to be a small stride ( 1.57079632679 * 0.033 = 0.05183628 => 121 iterations per revolution, so strides of 3 degrees)
pi_by_4 = 0.78539816339
pi_by_8 = 0.39269908169
pi_by_16 = pi_by_8 / 2

iterations_per_second = 30.30303030

t1_reached_initial = False
t1_reached_final = False
t1_angle_correct = False
t2_reached = False
t2_angle_correct = False



# ############### Do not modify anything outside this area ###############
# ########################################################################


# ######################## Place Helper Functions Here ##################
# ############## Do not modify anything outside this area ###############

def get_approach_angle(target_x, target_y, current_x, current_y) -> float:
    distance_y = target_y - current_y
    distance_x = target_x - current_x
    return math.atan2(distance_y, distance_x)

def is_distance_valid(target_x, target_y, current_x, current_y) -> bool:
    distance = math.sqrt( (target_x - current_x)**2 + (target_y - current_y)**2 )
    return distance < 0.01

def is_angle_valid(target_theta, current_theta) -> bool:
    return abs(target_theta - current_theta) <= 0.1

# ############## Do not modify anything outside this area ###############
# #######################################################################

target1_marker, = r.figure.gca().plot(-0.5, 0, marker='x', markersize=10, color='red')
target2_marker, = r.figure.gca().plot(target2[0], target2[1], marker='x', markersize=10, color='purple')

# Iterate for the previously specified number of iterations
for t in range(iterations):

    # Retrieve the most recent poses from the Robotarium
    p = r.get_poses()
    position_tracker, = r.figure.gca().plot(p[0], p[1], marker='o', markersize=3, color='blue')
    


    # ######################## Place Algorithm Here #########################
    # ############## Do not modify anything outside this area ###############
    # u = ???;

    # NOTE:
    #   calculate target1_start_theta each iteration
    #   remove "target1_distance" variable


    # if (t == 0):
    #     target1_distance_x = float(target1[0] - p[0])
    #     target1_distance_y = float(target1[1] - p[1])
    #     target1_start_theta = math.atan2(target1_distance_y, target1_distance_x)

    #     target1_distance = math.sqrt((target1_distance_x * target1_distance_x) + (target1_distance_y * target1_distance_y))

    
    # if (t1_reached_initial == False and (abs(p[2] - target1_start_theta) > 0.03)):
    #     u = np.array([[0], [pi_by_4]])
    # elif (t1_reached_initial == False and ( (abs(p[0] - target1[0] > .1)) or (abs(p[1] - target1[1] > .1)))):
    #     u = np.array([[0.08], [0]])
    # else:
    #     t1_reached_initial = True
    #     print(f"t1 reached: \n target is {target1} \n position is {p}")
    #     time.sleep(30)
    #     u = np.array([[0], [0]])


    # target 1 block (initial): Closed loop
    if not t1_reached_initial:
        current_angle = get_approach_angle(target_x=target1[0], target_y=target1[1], current_x=p[0], current_y=p[1])
        if abs(p[2] - current_angle) > 0.05:
            if p[2] > current_angle:
                u = np.array([[0], [-1 * pi_by_4]])
            else:
                u = np.array([[0], [pi_by_4]])
        elif ( abs(p[0] - target1[0]) > 0.1 or abs(p[1] - target1[1]) > 0.1):
            u = np.array([[0.08], [0]])
        else:
            t1_reached_initial = True
            print(f"t1 reached: \n target is {target1} \n position is {p}")
            u = np.array([[0], [0]])

    elif t1_reached_initial and not t1_angle_correct:
        current_angle = get_approach_angle(target_x=target1[0], target_y=target1[1], current_x=p[0], current_y=p[1])
        if abs(p[2]) > 0.1:
            print(f"absolute val of angle is {abs(p[2])}")
            u = np.array([[0], [pi_by_8]])
        else:
            print(f"absolute val of angle is {abs(p[2])}")
            t1_angle_correct = True
            time.sleep(3)


    # target 2 block: Closed loop
    elif t1_reached_initial and not t2_reached:
        current_angle = get_approach_angle(target_x=target2[0], target_y=target2[1], current_x=p[0], current_y=p[1])
        if abs(p[2] - current_angle) > 0.05:
            if p[2] > current_angle:
                u = np.array([[0], [-1 * pi_by_4]])
            else:
                u = np.array([[0], [pi_by_4]])
        elif ( abs(p[0] - target2[0]) > 0.1 or abs(p[1] - target2[1]) > 0.1):
            u = np.array([[0.08], [0]])
        else:
            t2_reached = True
            print(f"t2 reached: \n target is {target1} \n position is {p}")
            u = np.array([[0], [0]])
    elif t2_reached and not t2_angle_correct:
        current_angle = get_approach_angle(target_x=target2[0], target_y=target2[1], current_x=p[0], current_y=p[1])
        if abs(p[2]) > 0.1:
            print(f"absolute val of angle is {abs(p[2])}")
            u = np.array([[0], [pi_by_8]])
        else:
            print(f"absolute val of angle is {abs(p[2])}")
            t2_angle_correct = True
            time.sleep(3)

    # target 1 block (final): Closed loop
    elif t1_reached_initial and t2_reached and not t1_reached_final:
        pass
    

    # You should think about implementing a finite-state machine. How many
    # states are there? What are the transitions? What signals the transition
    # from one state to another?

    # ############## Do not modify anything outside this area ###############
    # #######################################################################

    # Send velocities to agents

    # Apply the barrier to the velocities
    u = uni_barrier_certificate(u, p)

    # Set velocities of agents 1,...,N
    r.set_velocities(np.arange(N), u) # u is the input, a 2x1 vector for 1 robot

    data.append(np.vstack((p, u)))
    # Send the previously set velocities to the agents.  This function must be called!
    r.step()

savemat('data.mat', {'data':np.hstack(tuple(data))})
# Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()

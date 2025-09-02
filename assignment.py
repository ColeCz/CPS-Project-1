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

# NOTE: Next Steps:
# Test the example u values
# Calculate u by hand
# Calculate u programmatically w/ helper func
# goal is to get pos p to equal target using u

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

iterations_per_second = 30.30303030

t1_reached = False
# target1_x = False
# target1_y = False
# target1_theta = False

# target2_passed = False
# target2_x = False
# target2_y = False
# target2_theta = False


# ############### Do not modify anything outside this area ###############
# ########################################################################


# ######################## Place Helper Functions Here ##################
# ############## Do not modify anything outside this area ###############
# def foo(b)

# ############## Do not modify anything outside this area ###############
# #######################################################################

# Iterate for the previously specified number of iterations
for t in range(iterations):
    # Retrieve the most recent poses from the Robotarium
    p = r.get_poses()
    target_marker, = r.figure.gca().plot(-0.5, 0, marker='x', markersize=10, color='red')
    target_marker2, = r.figure.gca().plot(p[0], p[1], marker='o', markersize=3, color='blue')
    


    # ######################## Place Algorithm Here #########################
    # ############## Do not modify anything outside this area ###############
    # u = ???;

    # NOTE:
    #   cosine_value = math.cos(angle_radians)

    if (t == 0):
        print(f"target is: {target1}\n")
        print(f"position is: {p}\n")
        target1_distance_x = float(target1[0] - p[0])
        print(f"x distance is: {target1_distance_x}\n")
        target1_distance_y = float(target1[1] - p[1])
        print(f"y distance is: {target1_distance_y}\n")

        target1_distance = math.sqrt((target1_distance_x * target1_distance_x) + (target1_distance_y * target1_distance_y))
        print(f"hypotenuse is: {target1_distance}\n")

        if target1_distance == 0:
            target1_distance += 0.01

        target1_start_theta = math.acos(target1_distance_x / target1_distance) * -1
        print(f"theta is: {target1_start_theta}\n")
        t1_iterations = target1_distance * iterations_per_second * 2 # NOTE: "2" is currently a magic number until I find a variable name for a value the denominator of the meters per second, should be "target1_distance * iterations_per_second / mps"
        t1_iterator = 0

    
    if (t1_reached == False and (abs(p[2] - target1_start_theta) > 0.03)):
        u = np.array([[0], [pi_by_4]])
    elif (t1_reached == False and ( (abs(p[0] - target1[0] > .1)) or (abs(p[1] - target1[1] > .1)))):
        print(f"second if entered, iteration {t1_iterator} of {t1_iterations}")
        u = np.array([[0.8], [0]])
        t1_iterator += 1
    else:
        t1_reached = True
        print(f"t1 reached: \n target is {target1} \n position is {p}")
        time.sleep(30)
        u = np.array([[0], [0]])



    
            

    


    
    
    #u = np.array([[1], [0]])

    # You  can try with u = np.array([[0.1], [0]]) and np.array([[0], [1]]) first.
    # Observe what happens to get a sense of how it works.

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

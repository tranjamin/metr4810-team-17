#!/usr/bin/env python3
'''
altitude_fuser.py - Sonar / Barometer fusion example using TinyEKF.

We model a single state variable, altitude above sea level (ASL) in
centimeters.  This is obtained by fusing the barometer pressure in Pascals and
sonar above-ground level (ASL) in centimters.

Copyright (C) 2016 Simon D. Levy

MIT License
'''

from math import pi
import numpy as np
import matplotlib.pyplot as plt
from TinyEKF.python.tinyekf import EKF

# for plotting
BARO_RANGE = 20
SONAR_RANGE = 200
BARO_BASELINE = 97420


# ground-truth AGL to sonar measurement, empirically determined:
# see http://diydrones.com/profiles/blogs/altitude-hold-with-mb1242-sonar
def sonarfun(agl):

    return 0.933 * agl - 2.894


# Convert ASL cm to Pascals: see
# http://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html
def asl2baro(asl):

    return 101325 * pow((1 - 2.25577e-7 * asl), 5.25588)


# Convert Pascals to cm ASL
def baro2asl(pa):

    return (1.0 - pow(pa / 101325.0, 0.190295)) * 4433000.0

def rotate_2d_matrix(theta: float):
    return np.array([[np.cos(theta),-np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])

if __name__ == '__main__':
    np.random.seed(4810)
    LOOPSIZE = 100
    TOTAL_TIME = 30 # seconds
    TIME_STEP_SENSOR = 0.01 # seconds
    TIME_STEP_PLOT = TIME_STEP_SENSOR # time step used for plotting true path

    # simulate a path of (t, sin(t))
    # this path has velocity (1, cos(t)), acceleration (0, -sin(t))
    # 
    # for fun, define theta as constantly spinning,
    # theta = t
    # this will cook the accelerations sensed by the robot

    ## CREATE TRUE DATA

    time = np.arange(0, TOTAL_TIME, TIME_STEP_PLOT)
    w = 1
    x = time
    xdot = np.ones(len(time))
    y = np.sin(time)
    ydot = np.cos(time)
    theta = time * 2*pi / TOTAL_TIME
    theta_dot = np.ones(len(time)) * 2*pi / TOTAL_TIME
    true_states = np.array([x, xdot, y, ydot, theta, theta_dot])

    count = 0

    N = 100

    # One state (ASL), two measurements (baro, sonar), with
    # larger-than-usual measurement covariance noise to help with sonar
    # blips.
    IMU_NOISE = 1.8*9.81/1000 # value from datasheet
    CAMERA_NOISE = 0.01  # 5e-4
    P = np.eye(6) * 5e-1
    Q = np.eye(2) * IMU_NOISE #1e-1 # applies directly to sensor measurements, gets distributed on the fly
    R = np.eye(4) * 0.01 #5e-5

    ekf = EKF(P)

    baro = np.zeros(N)
    sonar = np.zeros(N)
    fused = np.zeros(N)
    timesteps = len(time)
    fused_x = np.zeros(timesteps)
    fused_y = np.zeros(timesteps)
    fused_states = np.zeros([6, timesteps])

    initial_state = np.array([2, 0, 1, -1, 1, 0]) #true_states[:,0] # this is actually wrong, let's see if it corrects

    # initial dummy 'first' prediction
    ekf.predict(initial_state, np.zeros([6,6]), P) 

    #prev_state = initial_state
    for k, t in enumerate(time):

        # MEASURE, then UPDATE
        # We measure at the current time to update our current belief
        # Model Aruco Data (timestep k)

        measured_position = true_states[[0,2,4,5],k] + np.atleast_2d(np.random.normal(0, CAMERA_NOISE, 4))
        measured_position = np.ravel(measured_position)
        z = (measured_position[0], measured_position[1], measured_position[2], measured_position[3]) # this is a tuple for some reason

        # predicted measurement
        hx = ekf.get()[[0,2,4,5]] # We will measure these

        H = np.array([[1,0,0,0,0,0],
                     [0,0,1,0,0,0],
                     [0,0,0,0,1,0],
                     [0,0,0,0,0,1]])

        ekf.update(z, hx, H, R)

        # log these corrected measurements
        fused_x[k] = ekf.get()[0]
        fused_y[k] = ekf.get()[2]
        fused_states[:,k] = ekf.get()

        ## PREDICT
        # use current acceleration data to predict the next timestep
        # Model IMU Data (timestep k)

        true_acceleration_x_world = 0
        true_acceleration_y_world = -np.sin(t)
        a_world = np.array([[true_acceleration_x_world],[true_acceleration_y_world]])

        a_robot = np.linalg.inv(rotate_2d_matrix(true_states[4,k])) @ a_world
        # the acceleration measure depends on the TRUE angle and TRUE acceleration
        # add some noise

        a_measured = a_robot + np.atleast_2d(np.random.normal(0, IMU_NOISE, 2)).T

        ## ESTIMATE THE STATE

        # State-transition function f is actually already linear (in the state)

        F_state = np.array([
            [1, TIME_STEP_SENSOR, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, TIME_STEP_SENSOR, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, TIME_STEP_SENSOR],
            [0, 0, 0, 0, 0, 1]
            ]
        )


        B_not_rotated = np.array([[0.5*TIME_STEP_SENSOR**2, 0],[TIME_STEP_SENSOR, 0], [0, 0.5*TIME_STEP_SENSOR**2], [0, TIME_STEP_SENSOR], [0, 0], [0, 0]])
        theta_est = ekf.get()[4]
        B_rotated = B_not_rotated @ rotate_2d_matrix(theta_est)
        fx = np.ravel(F_state @ np.atleast_2d(ekf.get()).T + B_rotated @ a_measured)

        F_control = B_not_rotated @ rotate_2d_matrix(theta_est + pi/2) @ a_measured

        # Actual F matrix depends on control input too due to rotation
        F = F_state + F_control
        ## Predict location based off measured acceleration

        # noise changes distribution depending on direction
        Q_curr = B_rotated.dot(Q).dot(B_rotated.T)
        ekf.predict(fx, F, Q_curr)

        ## Now we have increased the noise, go back round the loop to update based on measurement (at next timestep)
        

    # Make some plot to show results
        
    plt.rcParams['text.usetex'] = True
    labels = [r"$x$ position", r"$x$ velocity", r"$y$ position", r"$y$ velocity", r"$\theta$ position", r"$\theta$ velocity"]
    for i in range(0,6):
        plt.subplot(3,2,i+1)
        plt.plot(time, fused_states[i,:], label = r"fused state")
        plt.plot(time, true_states[i,:], label = r"true state")
        plt.xlabel(r"Time")
        plt.ylabel(labels[i])
        plt.legend()

    plt.show()

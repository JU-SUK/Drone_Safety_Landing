import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot import inv

def ekfilter(x, y):
    dt = 1.0
    updateNumber = 0

    # Initialize State
    # First Update
    if updateNumber == 0:
        updateNumber += 1
        # compute position values from measurements
        temp_x = x
        temp_y = y

        # state vector
        # initialize position value
        ekfilter.x = np.array([[temp_x],
                               [temp_y],
                               [0],
                               [0]])
        # state covariance matrix
        # initialized to zero for first update
        ekfilter.P = np.array([[0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0]])
        # state transition matrix
        # linear extrapolation assuming constant velocity
        ekfilter.A = np.array([[1, 0, dt, 0],
                               [0, 1, 0, dt],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])
        # measurement covariance matrix
        # 다른 곳에선 이거 그냥 [[50, 0], [0, 50]] 이렇게 두던데 왜지??
        ekfilter.R = [[50, 0], [0, 50]]

        # system error matrix
        # initialize to zero matrix for first update
        ekfilter.Q = np.array([[0, 0, 0, 0],
                               [0 ,0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0]])
        # residual and kalman gain
        # not computed for first update but initialized so it could be output
        residual = np.array([[0, 0],
                             [0, 0]])
        K = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0]])
    # Reinitialize State
    # Second Update
    if updateNumber == 1:
        updateNumber += 1
        prev_x = ekfilter.x[0][0]
        prev_y = ekfilter.x[1][0]

        temp_x = x
        temp_y = y
        temp_xv = (temp_x - prev_x) / dt
        temp_yv = (temp_y - prev_y) / dt

        # state vector, reinitialized with new position and computed velocity
        ekfilter.x = np.array([[temp_x],
                               [temp_y],
                               [temp_xv],
                               [temp_yv]])
        # state covariance matrix
        # initialized to large values and more accurate position values can be used on measurement
        ekfilter.P = np.array([[100, 0, 0, 0],
                               [0, 100, 0, 0],
                               [0, 0, 250, 0],
                               [0, 0, 0, 250]])
        # state transition matrix
        ekfilter.A = np.array([[1, 0, dt, 0],
                               [0, 1, 0, dt],
                               [0 ,0, 1, 0],
                               [0, 0, 0, 1]])
        # measurement covariance matrix
        ekfilter.R = [[50, 0], [0, 50]]

        # system error matrix
        ekfilter.Q = np.array([[0.1, 0, 0, 0],
                               [0, 0.1, 0, 0],
                               [0, 0, 0.02, 0],
                               [0, 0, 0, 0.02]])
        residual = np.array([[0,0],
                             [0, 0]])
        K = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0]])
    if updateNumber > 1:
        # predict state forward
        x_prime = ekfilter.A.dot(ekfilter.x)

        # predict covariance forward
        P_prime = ekfilter.A.dot(ekfilter.P).dot(ekfilter.A.T) + ekfilter.Q

        # state to measurement transition matrix
        ekfilter.H = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0]])
        ekfilter.HT = np.array([[1, 0],
                                [0, 1],
                                [0, 0],
                                [0, 0]])
        # measurement covariance matrix
        ekfilter.R = [[50, 0], [0, 50]]

        # Compute Kalman Gain
        S = ekfilter.H.dot(P_prime).dot(ekfilter.HT) + ekfilter.R
        K = P_prime.dot(ekfilter.HT).dot(np.linalg.inv(S))

        # Estimate State
        temp_z = np.array([[x],
                           [y]])
        h_small = ekfilter.H.dot(x_prime)
        residual = temp_z - h_small

        # compute new estimate for state vector using Kalman Gain
        ekfilter.x = x_prime + K.dot(residual)
        # compute new estimate for state covariance
        ekfilter.P = P_prime - K.dot(ekfilter.H).dot(P_prime)
    return [ekfilter.x[0], ekfilter.x[1], ekfilter.x[2], ekfilter.x[3], ekfilter.P, K, residual, updateNumber]

import numpy as np

class KalmanFilter(object):
    def __init__(self, dt, u_x, u_y, u_z, std_acc, x_std_meas, y_std_meas, z_std_meas):
        """
        dt : sampling time(time for 1 cycle)
        u_x : acceleration in x-direction
        u_y : acceleration in y-direction
        u_z : acceleration in z-direction
        std_acc : process noise magnitude
        x_std_meas : standard deviation of the measurement in x-direction
        y_std_meas : standard deviation of the measurement in y-direction
        z_std_meas : standard deviation of the measurement in z-direction
        """
        # Define sampling time
        self.dt = dt

        # Define the control input variable
        self.u = np.matrix([[u_x], [u_y], [u_z]])

        # Initialize the state(x,y,z,x',y',z')
        self.x = np.matrix([[0], [0], [0], [0], [0], [0]])

        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, 0, self.dt, 0 ,0],
                            [0, 1, 0, 0, self.dt, 0],
                            [0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        # Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2)/2, 0, 0],
                            [0, (self.dt**2)/2, 0],
                            [0, 0, (self.dt**2)/2],
                            [self.dt, 0, 0],
                            [0, self.dt, 0],
                            [0, 0, self.dt]])
        # Define the Measurement Mapping Matrix
        # Assume that we are only measuring the position but not the velocity
        self.H = np.matrix([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])
        # Initialize the Process Noise Covariance
        self.Q = np.matrix([[(self.dt**4)/4, 0, 0, (self.dt**3)/2, 0, 0],
                            [0, (self.dt**4)/4, 0, 0, (self.dt**3)/2, 0],
                            [0, 0, (self.dt**4)/4, 0, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, 0, self.dt**2, 0, 0],
                            [0, (self.dt**3)/2, 0, 0, self.dt**2, 0],
                            [0, 0, (self.dt**3)/2, 0, 0, self.dt**2]]) * (std_acc**2)
        # Initialize the Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas, 0, 0],
                            [0, y_std_meas, 0],
                            [0, 0, z_std_meas]])
        # Initialize the Covariance Matrix
        # identity matrix whose shape is the same as the shape of the matrix A
        self.P = np.eye(self.A.shape[1])

    def predict(self):
        # update time state
        # x_k = A*x_(k-1) + B*u_(k-1)
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # calculate the error covariance
        # P = A*P*A^T + Q
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[0:3] # return x,y,z
    
    def update(self, z):
        # S = H*P*H^T + R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # calculate the Kalman Gain
        # K = P * H^T * inv(H*P*H^T + R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))), 3)
        I = np.eye(self.H.shape[1])

        # update error covariance matrix
        self.P = (I - (K*self.H)) * self.P
        return self.x[0:3]

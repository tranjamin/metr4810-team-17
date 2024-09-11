import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class KalmanEstimator():
    def __init__(self, Q, R, H, f_function, bound: float=None) -> None:
        self.Q = Q
        self.R = R
        self.H = H
        self.f_function = f_function
        self.bound = bound
        self.state_dimension = np.shape(Q)[0]
        assert np.shape(Q)[0] == np.shape(Q)[1]
        self.x_hat = np.zeros((self.state_dimension, 1))
        self.P = 0.5 * np.ones((self.state_dimension, self.state_dimension))
        # assume non-informative prior distribution

        self._received_first_measurement = False
        # predictions are only valid once the first measurement has been received
    
    def wrap_between_bounds(self, val):
        return (val + self.bound) % (2 * self.bound) - self.bound

    def wrapped_difference(self, b: float, a: float):
        """Calculates difference corrected after angle wrapping.
        b - a

        Args:
            b (float): b
            a (float): a
        """
        return 
    
    def update(self, z):
        if not self._received_first_measurement:
            self.x_hat = np.linalg.pinv(self.H) * z
            self._received_first_measurement = True
            return self.x_hat
        y = z - self.H @ self.x_hat
        if self.bound:
            y = self.wrap_between_bounds(y)

        S = self.H @ self.P @ np.atleast_2d(self.H).T + self.R

        K = self.P @ np.atleast_2d(self.H).T / S

        new_x_hat = self.x_hat + K @ np.atleast_2d(y)
        if self.bound:
            new_x_hat[0,0] = self.wrap_between_bounds(new_x_hat[0, 0])
        self.x_hat = new_x_hat 
        self.P = (np.identity(2) - K @ np.atleast_2d(self.H)) @ self.P
        return self.x_hat

    def predict(self, dt):
        if not self._received_first_measurement:
            return None

        if dt > 0:
            F: np.matrix = self.f_function(dt) 
            new_x_hat = F @ self.x_hat
            if self.bound:
                new_x_hat[0,0] = self.wrap_between_bounds(new_x_hat[0, 0])
            self.x_hat = new_x_hat
            self.P = F @ self.P @ F.T + self.Q

        return self.x_hat


class RigidBodyTracker():
    def __init__(self, Q_dist, R_dist, Q_angle, R_angle):
        # use a constant velocity model
        f_function = lambda dt: np.array([[1, dt],[0, 1]])

        H = np.array([[1, 0]]) # only positions are measured
        self.position_estimators = [
            KalmanEstimator(Q_dist, R_dist, H, f_function),
            KalmanEstimator(Q_dist, R_dist, H, f_function),
            KalmanEstimator(Q_dist, R_dist, H, f_function)
        ]
        self.angle_estimators = [
            KalmanEstimator(Q_angle, R_angle, H, f_function, math.pi),
            KalmanEstimator(Q_angle, R_angle, H, f_function),
            KalmanEstimator(Q_angle, R_angle, H, f_function, math.pi)
        ]
        pass

    def predict_estimate(self, dt = 0.0):
        # update current position given time from last call to this function
        positions = []
        angles = []
        for estimator in self.angle_estimators:
            angles.append(estimator.predict(dt))
        
        for estimator in self.position_estimators:
            positions.append(estimator.predict(dt))

        return positions, angles

    def update_estimate(self, rotation: R, tvec: np.ndarray, euler_order: str):

        for index, angle in enumerate(rotation.as_euler(euler_order)):
            self.angle_estimators[index].update(angle)
        
        for index, position in enumerate(tvec.ravel().tolist()):
            self.position_estimators[index].update(position)
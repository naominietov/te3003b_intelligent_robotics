import numpy as np

class DeadReckoning:
    def __init__(self):
        # Robot's initial position
        self.u0 = np.array([[0], [0], [0]])
        # Initial covariance matrix
        self.E0 = np.array([[0, 0, 0], 
                            [0, 0, 0], 
                            [0, 0, 0]])
        # Motion model covariance
        self.Qk = np.array([[0.5, 0.01, 0.01], 
                            [0.01, 0.5, 0.01], 
                            [0.01, 0.01, 0.2]]) 
        self.vel = 1 # 1 m/s mobile robot linear velocity
        self.omega = 1 # 1 rad/s mobile robot angular velocity
        self.dt = 0.1 # Sampling time
        self.H = 0

    def estimated_position(self):
        x_prev, y_prev, theta_prev = self.u0.ravel()

        delta_x = self.vel * self.dt * np.cos(theta_prev)
        delta_y = self.vel * self.dt * np.sin(theta_prev)

        delta_theta = self.omega * self.dt

        Sx_0 = x_prev + delta_x
        Sy_0 = y_prev + delta_y
        theta_hat = theta_prev + delta_theta

        uk = np.array([[Sx_0], [Sy_0], [theta_hat]])
        self.u0 = uk

        return uk
    
    def linearisation(self):
        Hk = np.array([[1, 0, -self.dt * self.vel * np.sin(self.u0[2][0])],
                      [0, 1, self.dt * self.vel * np.cos(self.u0[2][0])],
                      [0, 0, 1]])
        
        self.H = Hk

        return Hk
    
    def propagation_uncertainity(self, H):
        # Ek = np.matmul(self.H, np.matmul(self.E0, self.H.T)) + self.Qk
        Ek = self.H.dot(self.E0.dot(self.H.T)) + self.Qk
        self.E0 = Ek

        return Ek

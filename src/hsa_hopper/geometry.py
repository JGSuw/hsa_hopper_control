import numpy as np

class RigidBody:
    def __init__(self, I: np.matrix, com: np.array):
        raise NotImplementedError
    
class SE2Frame:
    def __init__(self, p: np.array, theta: float):
        self.p = p
        self.theta = theta
        c = np.cos(theta)
        s = np.sin(theta)
        self.mat = np.array([[c, -s, self.p[0]], [s, c, self.p[1]], [0, 0, 1]])

class SE2Generator:
    def __init__(self, v: np.array, omega: float):
        self.v = v
        self.omega = omega
        self.mat = np.array([[0, -omega, v[0]], [omega, 0, v[1]], [0,0,0]])
        self.vec = np.zeros(3)
        self.vec[:2] = self.v[:]
        self.vec[2] = omega

class SE2TangentVector:
    def __init__(self, g: SE2Frame, xi: SE2Generator):
        self.g = g
        self.xi = xi
        self.mat = self.g.mat @ self.xi.mat

    def body_velocity(self):
        return (self.xi[:2], self.xi[2])

    def spatial_velocity(self):
        mat = self.mat @ np.linalg.inv(self.g.mat)
        v = mat[:2,2]
        omega = (mat[0,1]-mat[1,0])/2
        return (v, omega) 


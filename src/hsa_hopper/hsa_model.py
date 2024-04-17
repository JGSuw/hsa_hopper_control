import numpy as np
import yaml
import os

class HSAPotential:
    def __init__(self, 
                 v: np.ndarray, # kernel weights i.e. estimates of potential at configuration
                 y: np.ndarray, # sample configurations corresponding to weights in v
                 s: float):     # length-scale parameter used by kernels
        self.N = y.shape[0]
        self.v = v
        self.y = y
        self.s = s
        self.S = S = np.cov(y.T)
        self.Sinv = Sinv = np.linalg.inv(S)

        # distance function, and its gradient, and hessian
        self.rho = lambda z, i: np.dot(z-y[i,:], Sinv@(z-y[i,:]))/s**2
        self.drho = lambda z, i: 2*Sinv@(z-y[i,:])/s**2
        self.d2rho = lambda z, i: 2*Sinv/s**2

        # square exponential kernel, its gradient, and hessian
        self.k = lambda z, i: np.exp(-self.rho(z,i))
        self.dk = lambda z, i: (self.k(z,i))*(-self.drho(z,i))
        self.d2k = lambda z, i: self.k(z,i)*(self.drho(z,i)@self.drho(z,i).T - self.d2rho(z,i))

    def V(self, z: np.array):
        return sum(self.v[i]*self.k(z,i) for i in range(self.N))
    
    def dV(self, z: np.array):
        return sum(self.v[i]*self.dk(z,i) for i in range(self.N))

    def d2V(self, z: np.array):
        return sum(self.v[i]*self.d2k(z,i) for i in range(self.N))
    
    def attribute_dict(self):
        attributes = {'v': self.v.tolist(), 'y': self.y.tolist(), 's': self.s}
        return attributes

def load_potential(path:os.path):
    with open(path, 'rb') as f:
        attributes = yaml.load(f, yaml.Loader)
    return HSAPotential(
        np.array(attributes['v']),
        np.array(attributes['y']),
        attributes['s']
    )

def save_potential(potential: HSAPotential, path: os.path):
    attributes = potential.attribute_dict()
    with open(path, 'wb') as f:
        yaml.dump(attributes, f, yaml.Dumper)
import numpy as np
import pickle
import os


class HSAPotential:
    def __init__(self, 
                 v: np.array, # kernel weights i.e. estimates of potential at configuration
                 y: np.array, # sample configurations corresponding to weights in v
                 s: float):   # length-scale parameter used by kernels
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

def load_potential(path:os.path):
    with open(path, 'rb') as f:
        data = pickle.load(f)
    return HSAPotential(
        data['v'],
        data['y'],
        data['s']
    )

def save_potential(potential: HSAPotential, path: os.path):
    data = {
        'v': potential.v,
        'y': potential.y,
        's': potential.s
    }
    with open(path, 'wb') as f:
        pickle.dump(data, f)
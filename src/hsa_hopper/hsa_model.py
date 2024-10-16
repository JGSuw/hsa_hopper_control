import numpy as np
import yaml
import os

class HSAPotential:
    def __init__(self, 
                 w_c: np.ndarray,   # conservative kernel weights i.e. estimates of potential at configuration
                 w_d: np.ndarray,   # dissipative kernel weights i.e. estimates of potential at configuration
                 y: np.ndarray,     # sample configurations corresponding to weights in v
                 l: float,          # smoothness parameter for dissipation potential
                 s: float):         # length-scale parameter used by kernels
        self.N = y.shape[0]
        self.w_c = w_c
        self.w_d = w_d
        self.y = y
        self.l = l
        self.s = s
        self.S = S = np.cov(y.T)
        self.Sinv = Sinv = np.linalg.inv(S)

        # distance function, and its gradient, and hessian
        self.rho = lambda z, i: np.dot(z-y[i,:], Sinv@(z-y[i,:]))/s**2
        self.drho = lambda z, i: 2*Sinv@(z-y[i,:])/s**2
        self.d2rho = lambda z, i: 2*Sinv/s**2

        # conservative kernel, its gradient, and hessian
        self.kc = lambda z, i: np.exp(-self.rho(z,i))
        self.dkc = lambda z, i: (self.kc(z,i))*(-self.drho(z,i))
        # self.d2k = lambda z, i: self.k(z,i)*(self.drho(z,i)@self.drho(z,i).T - self.d2rho(z,i))

        # activation function and derivative, used in disspation kernel
        self.f = lambda x: np.log((1+np.exp(self.l*x))*(1+np.exp(-self.l*x))/4)
        self.df = lambda x: (4*self.l)*(np.exp(self.l*x)-1)/(np.exp(self.l*x)+1)

        # disspation kernel and its gradient
        self.kd = lambda z, zdot, i: self.f(self.drho(z,i)@zdot)*np.exp(-self.rho(z,i))
        self.dkd = lambda z, zdot, i: self.df(self.drho(z,i)@zdot)*np.exp(-self.rho(z,i))*self.drho(z,i)


    def V(self, z: np.ndarray):
        return sum(self.w_c[i]*self.kc(z,i) for i in range(self.N))

    def R(self, z: np.ndarray, zdot: np.ndarray):
        return sum(self.w_d[i]*self.kd(z,zdot,i) for i in range(self.N))
    
    def dV(self, z: np.ndarray):
        return sum(self.w_c[i]*self.dkc(z,i) for i in range(self.N))

    def dR(self, z: np.ndarray, zdot: np.ndarray):
        return sum(self.w_d[i]*self.dkd(z,zdot,i) for i in range(self.N))

    # def d2V(self, z: np.ndarray):
    #     return sum(self.v[i]*self.d2k(z,i) for i in range(self.N))
    
    def attribute_dict(self):
        attributes = {'w_c': self.w_c.tolist(), 'w_d': self.w_d.tolist(), 'y': self.y.tolist(), 'l': self.l, 's': self.s}
        return attributes
    
    def make_from_dict(attributes):
        return HSAPotential(
            np.array(attributes['w_c']), 
            np.array(attributes['w_d']), 
            np.array(attributes['y']), 
            attributes['l'],
            attributes['s'])

def load_potential(path:os.path):
    with open(path, 'rb') as f:
        attributes = yaml.load(f, yaml.Loader)
    return HSAPotential.make_from_dict(attributes)

def save_potential(potential: HSAPotential, path: os.path):
    attributes = potential.attribute_dict()
    with open(path, 'wb') as f:
        yaml.dump(attributes, f, yaml.Dumper)
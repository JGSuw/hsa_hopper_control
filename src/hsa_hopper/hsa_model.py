import numpy as np
import yaml
import os

def logistic_function(x, a):
    return 1/(1+np.exp(-a*x))

class HSAPotential:
    def __init__(self, 
                 v: np.ndarray, # kernel weights i.e. estimates of potential at configuration
                 y: np.ndarray, # sample configurations corresponding to weights in v
                 a: float,      # exponential rate in logistic function
                 s: float):     # length-scale parameter used by kernels
        self.N = y.shape[1]
        self.v = v
        self.y = y
        self.a = a
        self.s = s
        self.S = S = np.cov(y)
        self.Sinv = Sinv = np.linalg.inv(S)

        # distance function, and its gradient, and hessian
        # self.rho = lambda z, i: np.dot(z-y[i,:], Sinv@(z-y[i,:]))/s**2
        # self.drho = lambda z, i: 2*Sinv@(z-y[i,:])/s**2
        self.rho = lambda res: np.sum(res*(Sinv@res),axis=0,keepdims=True)/s**2
        self.drho = lambda res: (2*Sinv@res/s**2)[[0],:]

        # square exponential kernel, its gradient, and hessian
        self.k = lambda z: np.exp(-self.rho(z-self.y))
        self.dk = lambda z: self.k(z)*(-self.drho(z-self.y))

    def V(self, l: float, psi: float, ldot: float):
        z = np.vstack((l,psi))
        _k = self.k(z)
        _logi = logistic_function(ldot, self.a)
        sum_prod = np.sum(self.v*_k,axis=1)
        potential = _logi*sum_prod[0]+(1-_logi)*sum_prod[1]
        return potential
    
    def dV(self, l: float, psi: float, ldot: float):
        z = np.vstack((l,psi))
        _dk = self.dk(z)
        _logi = logistic_function(ldot, self.a)
        sum_prod = np.sum(self.v*_dk,axis=1)
        force = _logi*sum_prod[0]+(1-_logi)*sum_prod[1]
        return force
    
    def attribute_dict(self):
        attributes = {'v': self.v.tolist(), 'y': self.y.tolist(), 's': self.s, 'a': self.a}
        return attributes
    
    def make_from_dict(attributes):
        return HSAPotential(
            np.array(attributes['v']), 
            np.array(attributes['y']), 
            attributes['a'],
            attributes['s'])

def load_potential(path:os.path):
    with open(path, 'rb') as f:
        attributes = yaml.load(f, yaml.Loader)
    return HSAPotential.make_from_dict(attributes)

def save_potential(potential: HSAPotential, path: os.path):
    attributes = potential.attribute_dict()
    with open(path, 'wb') as f:
        yaml.dump(attributes, f, yaml.Dumper)
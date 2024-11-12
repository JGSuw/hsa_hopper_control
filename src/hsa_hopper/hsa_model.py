import numpy as np
import yaml
import os

def quadratic_f(x,a):
    return .5*(a*x)**2

def quadratic_df(x,a):
    return x*a**2

class HSAPotential:
    LINEAR = 0
    GENERALIZED = 1
    def __init__(self, 
                 K: float,          # linear spring rate
                 F0: float,         # spring preload
                 b: float,          # linear dissipation term
                 w_c: np.ndarray,   # conservative kernel weights i.e. estimates of potential at configuration
                 y_c: np.ndarray,   # basis function locations for conservative kernels
                 w_d: np.ndarray,   # dissipative kernel weights i.e. estimates of potential at configuration
                 y_d: np.ndarray,   # basis function locations for dissipative kernels
                 a: float,          # smoothness parameter for dissipation potential
                 s: float,
                 kind=LINEAR):         
        self.K = K
        self.F0 = F0
        self.b = b
        self.kind = kind
        if kind != HSAPotential.LINEAR:
            assert(w_c.shape[0] == y_c.shape[1])
            self.w_c = w_c
            self.y_c = y_c
            assert(w_d.shape[0] == y_d.shape[1])
            self.w_d = w_d
            self.y_d = y_d
            self.a = a
            self.s = s
            self.Sc = Sc = np.cov(y_c)
            self.sc = np.linalg.det(Sc)
            self.Scinv = Scinv = np.linalg.inv(Sc)
            self.Sd = Sd = np.cov(y_d)
            self.sd = np.linalg.det(Sd)
            self.Sdinv = Sdinv = np.linalg.inv(Sd)

            # distance function, and its gradient, and hessian
            self.rhoc = lambda z, i: np.dot(z-self.y_c[:,i], Scinv@(z-self.y_c[:,i]))/self.s**2
            self.drhoc = lambda z, i: (2*Scinv@(z-self.y_c[:,i]))/self.s**2
            self.rhod = lambda z, i: np.dot(z-self.y_d[:,i], Sdinv@(z-self.y_d[:,i]))/self.s**2
            self.drhod = lambda z, i: (2*Sdinv@(z-self.y_d[:,i]))/self.s**2


            # conservative kernel, its gradient, and hessian
            self.kc = lambda z, i: np.exp(-self.rhoc(z,i))
            self.dkc = lambda z, i: self.kc(z, i)*(-self.drhoc(z,i))

            # activation function and derivative, used in disspation kernel
            if kind == HSAPotential.GENERALIZED:
                self.f = lambda x: quadratic_f(x, self.a)
                self.df = lambda x: quadratic_df(x, self.a)
            # disspation kernel and its gradient
            self.kd = lambda z, zdot, i: self.f(np.dot(self.drhod(z,i),zdot))*np.exp(-self.rhod(z,i))
            self.dkd = lambda z, zdot, i: self.df(np.dot(self.drhod(z,i),zdot))*np.exp(-self.rhod(z,i))*self.drhod(z,i)

    def V(self, z: np.ndarray):
        return sum(self.w_c[i]*self.kc(z,i) for i in range(self.w_c.shape[0])) + .5*self.K*z[0]**2

    def R(self, z: np.ndarray, zdot: np.ndarray):
        return sum(self.w_d[i]*self.kd(z,zdot,i) for i in range(self.w_d.shape[0])) + .5*self.b*zdot[0]**2
    
    def dV(self, z: np.ndarray):
        return sum(self.w_c[i]*self.dkc(z,i)[0] for i in range(self.w_c.shape[0])) + self.K*z[0] + self.F0

    def dR(self, z: np.ndarray, zdot: np.ndarray):
        return sum(self.w_d[i]*self.dkd(z,zdot,i)[0] for i in range(self.w_d.shape[0])) + self.b*zdot[0]

    def attribute_dict(self):
        if self.kind != HSAPotential.LINEAR:
            attributes = {
                'K' : self.K,
                'F0' : self.F0,
                'b': self.b, 
                'w_c': self.w_c.tolist(), 
                'y_c': self.y_c.tolist(), 
                'w_d': self.w_d.tolist(), 
                'y_d': self.y_d.tolist(), 
                'a': self.a,
                's': self.s,
                'kind': self.kind}
        else: 
            attributes = {
                'K' : self.K,
                'F0' : self.F0,
                'b': self.b, 
                'w_c': None, 
                'y_c': None, 
                'w_d': None, 
                'y_d': None, 
                'a': None, 
                's': None,
                'kind': self.kind}

        return attributes
    
    def num_params(self):
        if self.kind == HSAPotential.LINEAR:
            return 3
        else:
            return 3 + self.w_c.shape[0]+self.w_d.shape[0]
    
    def make_from_dict(attributes):
        return HSAPotential(
            attributes['K'],
            attributes['F0'],
            attributes['b'], 
            attributes['w_c'], 
            attributes['y_c'], 
            attributes['w_d'], 
            attributes['y_d'], 
            attributes['a'], 
            attributes['s'], 
            attributes['kind'])

def load_potential(path:os.path):
    with open(path, 'rb') as f:
        attributes = yaml.load(f, yaml.Loader)
    return HSAPotential.make_from_dict(attributes)

def save_potential(potential: HSAPotential, path: os.path):
    attributes = potential.attribute_dict()
    with open(path, 'wb') as f:
        yaml.dump(attributes, f, yaml.Dumper)
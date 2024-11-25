import numpy as np
import yaml
import os

def quadratic_f(x,a):
    return .5*(a*x)**2

def quadratic_df(x,a):
    return x*a**2

def quasi_quadratic_f(x,a):
    return np.log((1+np.exp(a*x))*(1+np.exp(-a*x))/(4*a))

def quasi_quadratic_df(x,a):
    return (np.exp(a*x)-1)/(1+np.exp(a*x))

class LinearModel:
    def __init__(self, k: float, f: float, b: float):
        self.k = k
        self.f = f
        self.b = b

    def dV(self, z: np.ndarray):
        return self.f + self.k*z[0]
    
    def dR(self, z: np.ndarray, zdot: np.ndarray):
        return self.b*zdot[0]

    def num_params(self):
        return 3

    def attribute_dict(self):
        return {'k' : self.k, 'f': self.f, 'b': self.b}

class HSAModel:
    QUADRATIC = 0
    QUASI_QUADRATIC = 1
    def __init__(self, 
                 w_c: np.ndarray,   # conservative kernel weights i.e. estimates of potential at configuration
                 y_c: np.ndarray,   # basis function locations for conservative kernels
                 w_d: np.ndarray,   # dissipative kernel weights i.e. estimates of potential at configuration
                 y_d: np.ndarray,   # basis function locations for dissipative kernels
                 a: float,          # smoothness parameter for dissipation potential
                 s: float,
                 linear=None,       # optional linear model
                 kind=QUADRATIC):
        self.linear = linear
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
        self.rhoc = lambda z, i: np.dot(z-self.y_c[:,i], Scinv@(z-self.y_c[:,i]))
        self.drhoc = lambda z, i: (2*Scinv@(z-self.y_c[:,i]))
        self.rhod = lambda z, i: np.dot(z-self.y_d[:,i], Sdinv@(z-self.y_d[:,i]))
        self.drhod = lambda z, i: (2*Sdinv@(z-self.y_d[:,i]))


        # conservative kernel, its gradient, and hessian
        self.kc = lambda z, i: np.exp(-self.s*self.rhoc(z,i))
        self.dkc = lambda z, i: self.kc(z, i)*(-self.drhoc(z,i)*self.s)

        # disspation kernel and its gradient
        self.kind = kind
        if kind == HSAModel.QUADRATIC:
            self.f = quadratic_f
            self.df = quadratic_df
        elif kind == HSAModel.QUASI_QUADRATIC:
            self.f = quasi_quadratic_f
            self.df = quasi_quadratic_df

        self.kd = lambda z, zdot, i: self.f(np.dot(zdot,self.Sdinv@zdot),self.a)*np.exp(-self.rhod(z,i)*self.s)
        self.dkd = lambda z, zdot, i: self.df(np.dot(zdot,self.Sdinv@zdot),self.a)*np.exp(-self.rhod(z,i)*self.s)*(2*self.Sdinv@zdot)
        # self.kd = lambda z, zdot, i: self.f(np.dot(self.drhod(z,i),zdot),self.sd)*np.exp(-self.rhod(z,i)*self.s)
        # self.dkd = lambda z, zdot, i: self.df(np.dot(self.drhod(z,i),zdot),self.sd)*np.exp(-self.rhod(z,i)*self.s)*(self.drhod(z,i))

    def V(self, z: np.ndarray):
        return sum(self.w_c[i]*self.kc(z,i) for i in range(self.w_c.shape[0]))

    def R(self, z: np.ndarray, zdot: np.ndarray):
        return sum(self.w_d[i]*self.kd(z,zdot,i) for i in range(self.w_d.shape[0]))
    
    def dV(self, z: np.ndarray):
        if self.linear is None:
            return sum(self.w_c[i]*self.dkc(z,i)[0] for i in range(self.w_c.shape[0]))
        else:
            return sum(self.w_c[i]*self.dkc(z,i)[0] for i in range(self.w_c.shape[0])) + self.linear.dV(z)


    def dR(self, z: np.ndarray, zdot: np.ndarray):
        if self.linear is None:
            return sum(self.w_d[i]*self.dkd(z,zdot,i)[0] for i in range(self.w_d.shape[0]))
        else:
            return sum(self.w_d[i]*self.dkd(z,zdot,i)[0] for i in range(self.w_d.shape[0])) + self.linear.dR(z,zdot)

    def attribute_dict(self):
        if self.linear is None:
            linear_attrs = None
        else:
            linear_attrs = self.linear.attribute_dict()
        attributes = {
            'linear' : linear_attrs,
            'w_c': self.w_c, 
            'y_c': self.y_c, 
            'w_d': self.w_d, 
            'y_d': self.y_d, 
            'a': float(self.a),
            's': float(self.s),
        }
        return attributes
    
    def num_params(self):
        if self.linear is None:
            return self.w_c.shape[0]+self.w_d.shape[0]
        else:
            return self.linear.num_params() + self.w_c.shape[0] + self.w_d.shape[0]
    
    def make_from_dict(attributes):
        if attributes['linear'] is None:
            linear = None
        else:
            linear = LinearModel(**attributes['linear'])
        return HSAModel(
            attributes['w_c'], 
            attributes['y_c'], 
            attributes['w_d'], 
            attributes['y_d'], 
            attributes['a'], 
            attributes['s'], 
            linear=linear
        )

def load_model(path: str):
    with open(path, 'rb') as f:
        attributes = yaml.load(f, yaml.Loader)
    return HSAModel.make_from_dict(attributes)

def save_model(potential: HSAModel, path: str):
    attributes = potential.attribute_dict()
    with open(path, 'w') as f:
        yaml.dump(attributes, f, yaml.Dumper)
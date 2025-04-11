import numpy as np
import yaml
import os

def quadratic_f(x,a):
    return .5*(a*x)**2

def quadratic_df(x,a):
    return x*a**2

class HSAModel:
    LINEAR = 0
    GENERALIZED = 1
    def __init__(self, 
                 K: float,          # linear spring rate
                 F0: float,         # spring preload
                 b: float,          # linear dissipation term
                 y: np.ndarray,     # location of basis functions in configuration space
                 w_c: np.ndarray,   # conservative kernel weights i.e. estimates of potential at configuration
                 w_d: np.ndarray,   # dissipative kernel weights i.e. estimates of potential at configuration
                 a: float,          # smoothness parameter for dissipation potential
                 s: float,
                 kind=LINEAR):         
        self.K = K
        self.F0 = F0
        self.b = b
        self.kind = kind
        if kind != HSAModel.LINEAR:
            assert(w_c.shape == (y.shape[0],))
            self.w_c = w_c
            self.y = y
            assert(w_d.shape == (y.shape[0],))
            self.w_d = w_d
            self.a = a
            self.s = s
            # covariance of basis centers
            self.S = S = np.cov(y,rowvar=False)
            self.Sdet = np.linalg.det(S)
            self.Sinv = Sinv = np.linalg.inv(S)

            # distance function, and its gradient, and hessian
            # self.rhoc = lambda z, i: np.dot(z-self.y_c[:,i], Scinv@(z-self.y_c[:,i]))/self.s**2
            # self.drhoc = lambda z, i: (2*Scinv@(z-self.y_c[:,i]))/self.s**2
            # self.rhod = lambda z, i: np.dot(z-self.y_d[:,i], Sdinv@(z-self.y_d[:,i]))/self.s**2
            # self.drhod = lambda z, i: (2*Sdinv@(z-self.y_d[:,i]))/self.s**2


            # conservative kernel, its gradient, and hessian
            # self.kc = lambda z, i: np.exp(-self.rhoc(z,i))
            # self.dkc = lambda z, i: self.kc(z, i)*(-self.drhoc(z,i))

            # activation function and derivative, used in disspation kernel
            # if kind == HSAModel.GENERALIZED:
            self.f = lambda x: quadratic_f(x, self.a)
            self.df = lambda x: quadratic_df(x, self.a)

            # disspation kernel and its gradient
            # self.kd = lambda z, zdot, i: self.f(np.dot(self.drhod(z,i),zdot))*np.exp(-self.rhod(z,i))
            # self.dkd = lambda z, zdot, i: self.df(np.dot(self.drhod(z,i),zdot))*np.exp(-self.rhod(z,i))*self.drhod(z,i)

    def basis_norm(self, l: np.ndarray, psi: float, grad=False):
        assert(type(l) == np.ndarray)
        z = np.zeros((*l.shape,2))
        z[...,0] = l
        z[...,1] = psi
        delta = np.zeros((self.y.shape[0],*z.shape))
        for i in range(delta.shape[0]):
            delta[i,...] = z - self.y[i,:]
        SinvDelta = np.einsum('ik,...k->...i', self.Sinv, delta)
        norm = np.einsum('...i,...i->...',SinvDelta,delta)/(2*self.s**2)
        if grad:
            return norm, SinvDelta[...,0]/self.s**2
        else:
            return norm

    def spring_kernel(self, l: np.ndarray, psi: float):
        norm = self.basis_norm(l, psi) # (i,j) indexed
        return np.exp(-norm)
        
    def spring_potential(self, l: np.ndarray, psi: float):
        if self.kind == HSAModel.LINEAR:
            return .5*l*(self.K*l+self.F0)
        else:
            kv = self.spring_kernel(l,psi)
            return np.einsum('i...,i->...', kv, self.w_c) + .5*l*(self.K*l+self.F0)
        
    def spring_force(self, l: np.ndarray, psi: float):
        if self.kind == HSAModel.LINEAR:
            return l*self.K+self.F0
        else:
            norm, norm_grad = self.basis_norm(l, psi, grad=True)
            dkdl = -np.exp(-norm)*norm_grad
            return np.einsum('i...,i->...',dkdl,self.w_c)+l*self.K+self.F0
        
    def dissipation_kernel(self, l: np.ndarray, ldot: np.ndarray, psi:float):
        norm = self.basis_norm(l,psi)
        # norm_dot = np.einsum('ij,i->ij',norm_grad,ldot)
        f = self.f(ldot)
        return f*np.exp(-norm)
    
    def dissipation_potential(self, l: np.ndarray, ldot:np.ndarray, psi: float):
        if self.kind == HSAModel.LINEAR:
            return .5*self.b*ldot**2
        else:
            kd = self.dissipation_kernel(l, ldot, psi)
            return np.einsum('i...,i->...',kd,self.w_d)+.5*self.b*ldot**2
        
    def dissipation_force(self, l:np.ndarray, ldot: np.ndarray, psi: float):
        if self.kind == HSAModel.LINEAR:
            return self.b*ldot
        else:
            norm, norm_grad = self.basis_norm(l,psi,grad=True)
            df = self.df(ldot)
            dkdldot = df*np.exp(-norm)
            return np.einsum('i...,i->...',dkdldot,self.w_d)+self.b*ldot
        
    def covectors(self, l: np.ndarray, ldot:np.ndarray, psi: float, grad=False):
        if grad:
            norm, norm_grad = self.basis_norm(l,psi,grad=True)
            k = np.exp(-norm)
            f = self.f(ldot)
            df = self.df(ldot)
            dk = -k*norm_grad
            return k, f*k, dk, df*k
        else:
            norm = self.basis_norm(l,psi)
            k = np.exp(-norm)
            f = self.f(ldot)
            return k, f*k

    def attribute_dict(self):
        if self.kind != HSAModel.LINEAR:
            attributes = {
                'K' : float(self.K),
                'F0' : float(self.F0),
                'b': float(self.b), 
                'y': self.y.tolist(), 
                'w_c': self.w_c.tolist(), 
                'w_d': self.w_d.tolist(), 
                'a': float(self.a),
                's': float(self.s),
                'kind': self.kind}
        else: 
            attributes = {
                'K' : float(self.K),
                'F0' : float(self.F0),
                'b': float(self.b), 
                'y': None, 
                'w_c': None, 
                'w_d': None, 
                'a': None, 
                's': None,
                'kind': self.kind}

        return attributes
    
    def num_params(self):
        if self.kind == HSAModel.LINEAR:
            return 3
        else:
            return 3 + self.w_c.shape[0]+self.w_d.shape[0]
    
    def make_from_dict(attributes):
        return HSAModel(
            attributes['K'],
            attributes['F0'],
            attributes['b'], 
            np.array(attributes['y']), 
            np.array(attributes['w_c']), 
            np.array(attributes['w_d']), 
            attributes['a'], 
            attributes['s'], 
            kind=attributes['kind'])

def load_potential(path:os.path):
    with open(path, 'rb') as f:
        attributes = yaml.load(f, yaml.Loader)
    return HSAModel.make_from_dict(attributes)

def save_potential(potential: HSAModel, path: os.path):
    attributes = potential.attribute_dict()
    with open(path, 'wb') as f:
        yaml.dump(attributes, f, yaml.Dumper)
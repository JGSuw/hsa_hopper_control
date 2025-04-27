import numpy as np
import yaml
import os

def quadratic_f(x,a):
    return .5*(a*x)**2

def quadratic_df(x,a):
    return x*a**2

class HSAModel:
    def __init__(self, 
                 K: float,          # linear spring rate
                 F0: float,         # spring preload
                 b: float,          # linear dissipation term
                 y: np.ndarray,     # location of basis functions in configuration space
                 S: np.ndarray,     # PSD matrix used in distance calculations (like a covariance matrix)
                 w_c: np.ndarray,   # conservative kernel weights i.e. estimates of potential at configuration
                 w_d: np.ndarray,   # dissipative kernel weights i.e. estimates of potential at configuration
                 a: float,          # smoothness parameter for dissipation potential
                 s: float):         
        self.K = K
        self.F0 = F0
        self.b = b
        self.y = y
        if (w_c is not None):
            assert(w_c.size == y.shape[0])
        self.w_c = w_c
        if (w_d is not None):
            assert(w_d.size== y.shape[0])
        self.w_d = w_d
        self.a = a
        self.s = s
        # covariance of basis centers
        if y is not None:
            # self.S = S = np.cov(y,rowvar=False)
            # self.Sdet = np.linalg.det(S)
            # self.Sinv = Sinv = np.linalg.inv(S)
            self.f = lambda x: quadratic_f(x, self.a)
            self.df = lambda x: quadratic_df(x, self.a)
        self.S = S

    def basis_norm(self, l: np.ndarray, psi: float, grad=False):
        assert(type(l) == np.ndarray)
        z = np.zeros((*l.shape,2))
        z[...,0] = l
        z[...,1] = psi
        delta = np.zeros((self.y.shape[0],*z.shape))
        for i in range(delta.shape[0]):
            delta[i,...] = z - self.y[i,:]
        # SinvDelta = np.einsum('ik,...k->...i', self.Sinv, delta)
        SDelta = np.einsum('ik,...k->...i', self.S, delta)
        norm = np.einsum('...i,...i->...', SDelta, delta)/(self.s**2)
        if grad:
            return norm, 2*SDelta[...,0]/self.s**2
        else:
            return norm

    def spring_kernel(self, l: np.ndarray, psi: float):
        norm = self.basis_norm(l, psi) # (i,j) indexed
        return np.exp(-norm)
        
    def spring_potential(self, l: np.ndarray, psi: float):
        if self.w_c is None:
            return .5*l*(self.K*l+self.F0)
        else:
            kv = self.spring_kernel(l,psi)
            return np.einsum('i...,i->...', kv, self.w_c) + .5*l*(self.K*l+self.F0)
        
    def spring_force(self, l: np.ndarray, psi: float):
        if self.w_c is None:
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
        if self.w_d is None:
            return .5*self.b*ldot**2
        else:
            kd = self.dissipation_kernel(l, ldot, psi)
            return np.einsum('i...,i->...',kd,self.w_d)+.5*self.b*ldot**2
        
    def dissipation_force(self, l:np.ndarray, ldot: np.ndarray, psi: float):
        if self.w_d is None:
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
            # f = self.f(ldot/np.sqrt(self.S[0,0]))
            # df = self.df(ldot/np.sqrt(self.S[0,0]))
            f = self.f(ldot)
            df = self.df(ldot)
            # f = self.f(norm_grad*ldot)
            # df = self.df(norm_grad*ldot)
            dk = -k*norm_grad
            if self.w_c is None:
                kc, dkc = None, None
            else:
                kc, dkc = k, dk
            if self.w_d is None:
                kd, dkd = None, None
            else:
                kd, dkd = f*k, df*k
            return kc, kd, dkc, dkd
        else:
            norm, norm_grad = self.basis_norm(l,psi,grad=True)
            k = np.exp(-norm)
            f = self.f(ldot*norm_grad)
            if self.w_c is None:
                kc = None
            else:
                kc = k
            if self.w_d is None:
                kd = None
            else:
                kd = f*k
            return kc, kd

    def attribute_dict(self):
        if self.y is not None:
            y = self.y.tolist()
        else:
            y = None
        if self.S is not None:
            S = self.S.tolist()
        else:
            S = None
        if self.w_c is not None:
            w_c = self.w_c.tolist()
        else:
            w_c = None
        if self.w_d is not None:
            w_d = self.w_c.tolist()
        else:
            w_d = None
        attributes = {
            'K' : float(self.K),
            'F0' : float(self.F0),
            'b': float(self.b), 
            'y': y, 
            'S': S,
            'w_c': w_c, 
            'w_d': w_d, 
            'a': self.a, 
            's': self.s}

        return attributes
    
    def num_params(self):
        n = 3
        if self.w_c is not None:
            n += self.w_c.size
        if self.w_d is not None:
            n += self.w_d.size
        return n
    
    def make_from_dict(attributes):
        if attributes['y'] is None:
            y = None 
        else:
            y = np.array(attributes['y'])
        if attributes['S'] is None:
            S = None
        else:
            S = np.array(attributes['S'])
        if attributes['w_c'] is None:
            w_c = None
        else:
            w_c = np.array(attributes['w_c'])
        if attributes['w_d'] is None:
            w_d = None
        else:
            w_d = np.array(attributes['w_d'])
        return HSAModel(
            attributes['K'],
            attributes['F0'],
            attributes['b'],
            y,
            S,
            w_c,
            w_d,
            attributes['a'], 
            attributes['s'])

def load_potential(path:os.path):
    with open(path, 'rb') as f:
        attributes = yaml.load(f, yaml.Loader)
    return HSAModel.make_from_dict(attributes)

def save_potential(potential: HSAModel, path: os.path):
    attributes = potential.attribute_dict()
    with open(path, 'wb') as f:
        yaml.dump(attributes, f, yaml.Dumper)
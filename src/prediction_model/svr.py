# -*- coding: utf-8 -*-

import numpy as np

class SVR:
    def __init__(self, filename):
        model = np.load(filename)
        self.mu = np.array(model['mu'])
        self.sigma = np.array(model['sigma'])
        self.sv = np.array(model['sv'])
        self.kernel_scale = np.array(model['kernel_scale'])
        self.alpha = np.array(model['alpha'])
        self.bias = np.array(model['bias'])
        self.in_size = len(self.mu)

    def predict_all(self, x):
        n = x.shape[0]
        y = np.zeros((n, 1))
        for i in range(n):
            y[i] = self.predict(x[i:i+1,:])
        return y

    def predict(self, x):
        if x.shape[0] > 1:
            raise ValueError(
                "Input shape of {:d} is not supported by SVR.predict()".format(x.shape[0]) +
                "\n\tUse SVR.predict_all() for multiple input samples"
            )

        if x.shape[1] == 6:
            x = np.delete(x, 2, 1)
        
        if x.shape[1] != self.in_size:
            raise ValueError(
                "Input shape of {:d} is different from expected dimension of {:d}".format(
                    x.shape[1], self.in_size
                )
            )

        x_normalised = (x - self.mu) / self.sigma
        _temp = (self.sv - x_normalised) / self.kernel_scale
        # K = np.exp(- np.linalg.norm(_temp, ord=2, axis=1)**2)
        K = np.exp(- np.sum(_temp**2, axis=1))

        y = np.sum(self.alpha * K) + self.bias
        return [y]

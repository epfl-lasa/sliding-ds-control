# -*- coding: utf-8 -*-

import numpy as np
import os

from . import SVR
from . import NN

class BumperModel:
    def __init__(self, folder=None):
        if folder is None:
            folder = os.path.dirname(__file__)

        # self.models = [
        #     SVR(os.path.join(folder, 'trainedModels_Fx.npz')),
        #     SVR(os.path.join(folder, 'trainedModels_Fy.npz')),
        #     SVR(os.path.join(folder, 'trainedModels_Tz.npz'))
        # ]

        self.models = [
            NN(os.path.join(folder, 'trainedModels_nn.tflite'))
        ]
    
    def predict(self, x):
        return [
            out
            for model in self.models
            for out in model.predict(x)
        ]

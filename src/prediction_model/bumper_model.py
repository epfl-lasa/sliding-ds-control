# -*- coding: utf-8 -*-

import os
from typing import List

from . import SVR
from . import NN

class BumperModel:
    """BumperModel class"""

    def __init__(self, folder: str = None, model_type: str = "NN") -> None:
        if folder is None:
            folder = os.path.dirname(__file__)

        if model_type.upper() == "SVR":
            self.models = [
                SVR(os.path.join(folder, 'trainedModels_Fx.npz')),
                SVR(os.path.join(folder, 'trainedModels_Fy.npz')),
                SVR(os.path.join(folder, 'trainedModels_Tz.npz'))
            ]
        else:
            # Default to RNN model
            self.models = [
                NN(os.path.join(folder, 'trainedModels_nn.tflite'))
            ]
    
    def predict(self, x: List[float]) -> List[float]:
        return [
            out
            for model in self.models    
            for out in model.predict(x)
        ]

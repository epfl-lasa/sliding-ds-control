#!/usr/bin/env python3

import numpy as np
import tflite_runtime.interpreter as tflite

class NN:
    def __init__(self, filename):
        self.interpreter = tflite.Interpreter(model_path=filename)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Prev Values
        self.prev = np.zeros((5,6))

    def predict(self, x):
        data = np.float32(np.vstack((self.prev, x)))

        self.interpreter.set_tensor(self.input_details[0]['index'], data.reshape((1, -1)))
        self.interpreter.invoke()

        output_data = [
            self.interpreter.get_tensor(self.output_details[i]['index'])[0, 0]
            for i in range(len(self.output_details))
        ]
        self.prev = data[1:,:]
        return output_data
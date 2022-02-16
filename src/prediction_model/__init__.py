"""Bumper Prediction Module.

This module implements prediction model for bumper damping.
"""

__all__ = ['SVR', 'NN', 'BumperModel']
__version__ = '1.0'
__author__ = 'Vaibhav Gupta'

from .svr import SVR
from .nn import NN
from .bumper_model import BumperModel


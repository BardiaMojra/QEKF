import sys
import pandas as pd
import numpy as np


from pdb import set_trace as st
from nbug import *




class dlm:
  ''' Data Logger Module
  '''
  def __init__(self, enabled=True):
    self.enabled = enabled
    if self.enabled == True:
      self.idx = None
      self.z_hist = None # state measurement history
      self.x_hist = None # state estimate history
      self.y_hist = None # residual history
      self.P_hist = None # state estimation covar posterior history
      self.K_hist = None # Kalman gain matrix history
    else:
      eprint('Warning: data logger is DISABLED...\n\n')
    return

  def log_z_state(self, z, idx):
    if self.enabled == True:
      if (self.z_hist is None) or (self.idx is None):
        self.z_hist = np.copy(z.T)
        self.idx = np.zeros((1), dtype=int)
      else:
        self.z_hist = np.concatenate((self.z_hist, z.T), axis=0)
        self.idx = np.concatenate((self.idx, np.asarray([idx])), axis=0)
    return

  def log_update(self, y, x, P, K):
    if self.enabled == True:
      if (self.y_hist is None) or \
         (self.x_hist is None) or \
         (self.P_hist is None) or \
         (self.K_hist is None):
        self.y_hist = np.copy(y.T)
        self.x_hist = np.copy(x.T)
        self.P_hist = np.copy(P.reshape(1,-1))
        self.K_hist = np.copy(K.reshape(1,-1))
        self.K_hist = np.copy(K.reshape(1,-1))
      else:
        self.y_hist = np.concatenate((self.y_hist, y.T), axis=0)
        self.x_hist = np.concatenate((self.x_hist, x.T), axis=0)
        self.P_hist = np.concatenate((self.P_hist, P.reshape(1,-1)), axis=0)
        self.K_hist = np.concatenate((self.K_hist, K.reshape(1,-1)), axis=0)
    return



# end of file

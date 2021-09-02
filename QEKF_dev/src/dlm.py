import sys
import pandas as pd
import math
from math import isnan
from pdb import set_trace as st
import numpy as np

linehead = '\n\n  \--->> '
longtail = '\n\n'

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

class dlm:
  ''' Data Logger Module
  '''
  def __init__(self, enabled=True):
    self.enabled = enabled
    if self.enabled == True:
      # log measurement (marginal) state
      self.idx = None
      self.z_hist = None # state measurement history
      # log est (prior) state
      self.x_prior_hist = None # state estimate history
      self.P_prior_hist = None # state covar estimate history
      # log update (posterior) state
      self.v_hist = None # residual history
      self.x_post_hist = None # state estimation posterior history
      self.P_post_hist = None # state estimation covar posterior history
      self.K_hist = None # Kalman gain matrix history
      self.x_q_hist = np.ndarray(shape=(0,4))
      self.x_prior_q_hist = np.ndarray(shape=(0,4))
    else:
      eprint(linehead+'Warning: data logger is DISABLED...\n\n')
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

  def log_predicted_quat(self, x_q, x_pr_q):
    ''' Not used
    '''
    if self.enabled == True:
      st()
      #if np.all(np.isnan(self.x_q_hist)) or np.all(np.isnan(self.x_prior_q_hist)):
      #  self.x_q_hist = np.ndarray([x_q.w, x_q.x, x_q.y, x_q.z],shape=(1,4))
      #  self.x_prior_q_hist = x_pr_q.T
      #  print('self.x_q_hist:'); print(self.x_q_hist)
      #  print('x_prior_q_hist:'); print(self.x_prior_q_hist)
      #else:
      q = np.ndarray([float(x_q.w),
                      float(x_q.x),
                      float(x_q.y),
                      float(x_q.z)])
      q = np.reshape(q, (4,1))
      q = q.T # 1x4
      self.x_q_hist = np.concatenate((self.x_q_hist, q), axis=0)

      self.x_prior_q_hist = np.concatenate((self.x_prior_q_hist,\
        x_pr_q.T), axis=0)
    st()
    return

  def log_prediction(self, x_prior, P_prior):
    if self.enabled == True:
      if (self.x_prior_hist is None) or (self.P_prior_hist is None):
        self.x_prior_hist = np.copy(x_prior.T)
        self.P_prior_hist = np.copy(P_prior.flatten())
      else:
        self.x_prior_hist = np.concatenate((self.x_prior_hist, x_prior.T), axis=0)
        self.P_prior_hist = np.concatenate((self.P_prior_hist, P_prior.flatten()), axis=0)
    return

  def log_update(self, v, x_post, P_post, K):
    if self.enabled == True:
      if (self.v_hist is None) or \
         (self.x_post_hist is None) or \
         (self.P_post_hist is None) or \
         (self.K_hist is None):
        self.v_hist = np.copy(v.T)
        self.x_post_hist = np.copy(x_post.T)
        self.P_post_hist = np.copy(P_post.reshape(1,-1))
        self.K_hist = np.copy(K.reshape(1,-1))
      else:
        self.v_hist = np.concatenate((self.v_hist, v.T), axis=0)
        self.x_post_hist = np.concatenate((self.x_post_hist, x_post.T), axis=0)
        self.P_post_hist = np.concatenate((self.P_post_hist, P_post.reshape(1,-1)), axis=0)
        self.K_hist = np.concatenate((self.K_hist, K.reshape(1,-1)), axis=0)
    return



# end of file

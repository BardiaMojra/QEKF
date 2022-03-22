import pandas as pd
import numpy as np

from pdb import set_trace as st
from nbug import *


''' config '''
_prec       = 6 # nprt precision
np.set_printoptions(precision=_prec)
pd.set_option('display.float_format', lambda x: f'%.{_prec}f' % x)


class dlm:
  ''' data logger module '''
  def __init__(self, enabled=True):
    self.enabled = enabled
    if self.enabled == True:
      print(lhead+'data logger is ENABLED...'+stail)
      self.i_hist   = None
      self.q_hist   = None
      self.qs_hist   = None
      self.qr_hist  = None
      self.t_hist   = None
      self.tr_hist  = None
      self.Qerr_hist  = None
      self.Terr_hist  = None
      self.alg_hist   = None
    else:
      eprint('Warning: data logger is DISABLED...\n\n')
    return

  def log_quat(self, i:int, col:str, q:np.quaternion, alg:str):
    assert alg, '\n\nno "alg" passed to log_quat()...\n\n'
    self.log_dat(i, col+'_w', np.float128(q.w), alg)
    return

  def log_state(self,i,q,qs,qr,t,tr,Qerr,Terr,alg):
    assert self.enabled, '\n\ndata logger DISABLES\n\n'
    #todo: format the data to be logged.
    #i,q,qs,qr,t,tr,Qerr,Terr,alg
    st()


    if(self.i_hist is None) or\
      (self.q_hist is None) or\
      (self.qs_hist is None) or\
      (self.qr_hist is None) or\
      (self.t_hist is None) or\
      (self.tr_hist is None) or\
      (self.Qerr_hist is None) or\
      (self.Terr_hist is None) or\
      (self.alg_hist is None):
        self.i_hist = np.copy(i)
        self.q_hist = np.copy(q.flatten())
        self.qs_hist = np.copy(q.flatten())
        self.qr_hist = np.copy(qr.flatten())
        self.t_hist = np.copy(t.flatten())
        self.tr_hist = np.copy(tr.flatten())
        self.Qerr_hist = np.copy(Qerr.flatten())
        self.Terr_hist = np.copy(Terr.flatten())
        self.alg_hist = np.copy(alg)

        st()
    else:
      self.i_hist = np.concatenate((self.i_hist, i), axis=0)
      self.q_hist = np.concatenate((self.q_hist, q.flatten()), axis=0)
      self.qs_hist = np.concatenate((self.qs_hist, qs.flatten()), axis=0)
      self.qr_hist = np.concatenate((self.qr_hist, qr.flatten()), axis=0)
      self.t_hist = np.concatenate((self.t_hist, t.flatten()), axis=0)
      self.tr_hist = np.concatenate((self.tr_hist, tr.flatten()), axis=0)
      self.Qerr_hist = np.concatenate((self.Qerr_hist, Qerr.flatten()), axis=0)
      self.Terr_hist = np.concatenate((self.Terr_hist, Terr.flatten()), axis=0)
      self.alg_hist = np.concatenate((self.alg_hist, alg.flatten()), axis=0)
    st()

    self.prt()
    return





  def prt(self):
    assert self.enabled, '\n\ndata logger is DISABLED!\n\n'
    nprint('self.i_hist', self.i_hist)
    nprint('self.q_hist', self.q_hist)
    nprint('self.qs_hist', self.qs_hist)
    nprint('self.qr_hist', self.qr_hist)
    nprint('self.t_hist', self.t_hist)
    nprint('self.tr_hist', self.tr_hist)
    nprint('self.Qerr_hist', self.Qerr_hist)
    nprint('self.Terr_hist', self.Terr_hist)
    nprint('self.alg_hist', self.alg_hist)
    st()
    return





# EOF

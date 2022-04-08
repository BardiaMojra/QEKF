import pandas as pd
import numpy as np

''' private modules '''
from utils import *
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



  def log_state(self,i:int,q:np.quaternion,qs:np.ndarray,qr:np.quaternion,
                t:np.ndarray,tr:np.ndarray,Qerr:np.float128,Terr:np.float128,
                alg:str,_dtype=np.float128):
    assert self.enabled, '\n\ndata logger DISABLES\n\n'

    i = np.asarray([[i]], dtype=int).reshape(1,1)
    q = quat2np(q)
    qs = quats2np(qs)
    qr = quat2np(qr)
    t = np.asarray(t, dtype=_dtype).reshape(1,-1)
    tr = np.asarray(tr, dtype=_dtype).reshape(1,-1)
    Qerr = np.asarray([[Qerr]], dtype=_dtype).reshape(1,1)
    Terr = np.asarray([[Terr]], dtype=_dtype).reshape(1,1)
    alg = np.asarray([[alg]], dtype=str).reshape(1,1)

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
        self.q_hist = np.copy(q)
        self.qs_hist = np.copy(qs)
        self.qr_hist = np.copy(qr)
        self.t_hist = np.copy(t)
        self.tr_hist = np.copy(tr)
        self.Qerr_hist = np.copy(Qerr)
        self.Terr_hist = np.copy(Terr)
        self.alg_hist = np.copy(alg)
    else:
      self.i_hist = np.concatenate((self.i_hist,i), axis=0)
      self.q_hist = np.concatenate((self.q_hist,q), axis=0)
      self.qs_hist = concat_w_padding(self.qs_hist,qs,_axis=0)
      self.qr_hist = np.concatenate((self.qr_hist,qr), axis=0)
      self.t_hist = np.concatenate((self.t_hist, t), axis=0)
      self.tr_hist = np.concatenate((self.tr_hist, tr), axis=0)
      self.Qerr_hist = np.concatenate((self.Qerr_hist, Qerr), axis=0)
      self.Terr_hist = np.concatenate((self.Terr_hist, Terr), axis=0)
      self.alg_hist = np.concatenate((self.alg_hist, alg), axis=0)
    return

  def prt_stats(self):
    npprint('self.q_hist', self.q_hist)
    npprint('self.qr_hist', self.qr_hist)
    npprint('self.Qerr_hist', self.Qerr_hist)
    npprint('self.Terr_hist', self.Terr_hist)
    print('\n\n add NVM stat logger....\n\n')
    print('\n >> results and statistics << ')
    tbprint('rotation error mean',   np.mean(self.Qerr_hist))
    tbprint('rotation error std',    np.std(self.Qerr_hist))
    tbprint('rotation error median', np.median(self.Qerr_hist))
    tbprint('rotation 25% quantile', np.quantile(self.Qerr_hist, 0.25))
    tbprint('rotation 75% quantile', np.quantile(self.Qerr_hist, 0.75))
    tbprint('translation error mean',   np.mean(self.Terr_hist))
    tbprint('translation error std',    np.std(self.Terr_hist))
    tbprint('translation error median', np.median(self.Terr_hist))
    tbprint('translation 25% quantile', np.quantile(self.Terr_hist, 0.25))
    tbprint('translation 75% quantile', np.quantile(self.Terr_hist, 0.75))

  def prt_log(self):
    assert self.enabled, '\n\ndata logger is DISABLED!\n\n'
    npprint('self.i_hist', self.i_hist)
    npprint('self.q_hist', self.q_hist)
    # npprint('self.qs_hist', self.qs_hist)
    npprint('self.qr_hist', self.qr_hist)
    npprint('self.t_hist', self.t_hist)
    npprint('self.tr_hist', self.tr_hist)
    npprint('self.Qerr_hist', self.Qerr_hist)
    npprint('self.Terr_hist', self.Terr_hist)
    # npprint('self.alg_hist', self.alg_hist)
    return

def concat_w_padding(qs_h:np.ndarray,qs:np.ndarray,_axis=0):
  w = max(qs_h.shape[1],qs.shape[1])
  if qs_h.shape[1] == qs.shape[1]:
    pass
  elif qs_h.shape[1] > qs.shape[1]:
    pad = np.zeros((1,w-qs.shape[1]),dtype=type(qs_h.dtype))
    qs = np.concatenate((qs,pad),axis=1)
  elif qs_h.shape[1] < qs.shape[1]:
    pad = np.zeros((qs_h.shape[0],w-qs_h.shape[1]),dtype=type(qs_h.dtype))
    qs_h = np.concatenate((qs_h,pad),axis=1)
  else:
    assert True, lhead+'IMPOSSIBLE!'+stail
  # npprint('qs_h', qs_h)
  # npprint('qs', qs)
  qs_n = np.concatenate((qs_h,qs),axis=_axis)
  # npprint('qs_n', qs_n)
  return qs_n

def tbprint(string, args=None):
  print('\-->> '+string+': '+str(args))

# EOF

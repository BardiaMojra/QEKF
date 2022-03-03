import pandas as pd
import numpy as np

from pdb import set_trace as st
from nbug import *

class dlm:
  ''' data logger module '''
  def __init__(self, enabled=True):
    self.enabled = enabled
    if self.enabled == True:
      print(lhead+'data logger is ENABLED...'+stail)
      self.df = pd.DataFrame()
    else:
      eprint('Warning: data logger is DISABLED...\n\n')
    return

  def log_data(self, idx:int, alg:str, label:str, val):
    if self.enabled == True:
      if label in self.df:
        self.df[label] = val.copy()
      else:
        self.df.insert(len(self.df.columns), label, np.zeros((self.df.shape[0])))
        self.df[label] = val.copy()
    return






# EOF

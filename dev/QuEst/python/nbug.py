
import sys
# import pdb
from pprint import pprint as pp
import numpy as np
import pandas as pd

''' NBUG config '''
_prec       = 6 # nprt precision
nprt_en     = True #False
nprt_en2    = True

# print aliases
llhead      = '\n\n  \--->> '
lhead       = '\n\-->> '
shead       = '\-->> '
lltail      = '\n\n\n'
ltail       = '\n\n'
stail       = ''
attn        = 'here ----------- <<<<<\n\n'

np.set_printoptions(precision=_prec)
pd.set_option('display.float_format', lambda x: f'%.{_prec}f' % x)

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def nprint(string, args=None):
  if nprt_en is True:
    if args is not None:
      print(shead+string+': ')
      pp(args); print(stail)
    else:
      pp(shead+(string+stail))

def nsprint(string, ndarr:np.ndarray):
  if nprt_en is True:
    shape = str(ndarr.shape)
    print(shead+(string+': '+shape));
    # pp(ndarr); print(stail)

def npprint(string:str, ndarr:np.ndarray, N=None, labels=None):
  if nprt_en is True:
    shape = str(ndarr.shape)
    print(shead+string+': '+shape)
    df = pd.DataFrame(ndarr, columns=labels)
    if isinstance(N,int): pp(df.head(N)); print(stail)
    else: pp(df); print(stail)

def dfspp(string, df:pd.DataFrame):
  if nprt_en is True:
    shape = str(df.shape)
    print(shead+(string+': '+shape))

def nlprint(string, list_like:list):
  if nprt_en is True:
    le = str(len(list_like))
    print(shead+(string+': '+le));
    # pp(list_like); print(shorttail)

def plh():
  print(lhead);

def pstail():
  print(stail)

def pattn():
  print(attn)

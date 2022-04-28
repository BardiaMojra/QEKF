
import sys
# import pdb
import os
from pprint import pprint as pp
import numpy as np
import pandas as pd

''' NBUG config '''
_prec       = 2 # nprt precision
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

# np.set_printoptions(precision=_prec)
pd.set_option("display.precision", _prec)
# pd.option_context('display.float_format', '{:0.10f}'.format)

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
    # pd.set_option("display.precision", _prec)
    # pd.option_context('display.float_format', '{:0.10f}'.format)
    df = pd.DataFrame(ndarr.astype(np.float128), columns=labels)
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

def write_np2txt(ndarr:np.ndarray,fname:str,
                 _dir:str='../pout/',_sep:str=' ',_prt=False):
  ''' example: write_np2txt(A0,fname='QuEst_A0_py.txt') '''
  df = pd.DataFrame(data=ndarr)
  df.to_csv(_dir+fname,sep=_sep,header=False,index=False)
  return

def load_txt2np(fname:str,_dir:str='../mout/',_sep:str=' ',_prt=False):
  assert os.path.exists(_dir+fname)
  ndarr = np.loadtxt(_dir+fname, delimiter=_sep)
  if _prt: npprint(fname,ndarr)
  return ndarr

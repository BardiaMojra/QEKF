
import sys
# import pdb
from pprint import pprint as pp
import numpy as np
import pandas as pd

# enable nbug print
nprt_en = True #False
nprt_en2 = True

llhead = '\n\n  \--->> '
lhead = '\n\-->> '
shead = '\-->> '
ltail = '\n\n'
stail = '\n'
attn = 'here ----------- <<<<<\n\n'

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def nprint(string, *args):
  if nprt_en is True:
    if args is not None:
      print(shead+(string+': '))
      pp(*args); print(stail)
    else:
      print(shead+(string+stail))

def nsprint(string, ndarr:np.ndarray):
  if nprt_en is True:
    shape = str(ndarr.shape)
    print(shead+(string+': '+shape));
    pp(ndarr); print(stail)

def nppshape(string, ndarr:np.ndarray):
  if nprt_en is True:
    shape = str(ndarr.shape)
    print(shead+(string+': '+shape));

def dfspp(string, df:pd.DataFrame):
  if nprt_en is True:
    shape = str(df.shape)
    print(shead+(string+': '+shape));

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

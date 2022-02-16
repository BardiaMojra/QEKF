
import sys
# import pdb
from pprint import pprint as pp
import numpy as np
import pandas as pd

# enable nbug print
nprt_en = True #False
nprt_en2 = True


longhead = '\n\-->> '
shorthead = '\-->> '
longtail = '\n\n'
shorttail = '\n'
attn = 'here ----------- <<<<<\n\n'

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def nprint(string, *args):
  if nprt_en is True:
    if args is not None:
      print(shorthead+(string+': '+str(args.__class__)));
      pp(*args); print(shorttail)

def nsprint(string, ndarr:np.ndarray):
  if nprt_en is True:
    shape = str(ndarr.shape)
    print(shorthead+(string+': '+shape));
    pp(ndarr); print(shorttail)

def nppshape(string, ndarr:np.ndarray):
  if nprt_en is True:
    shape = str(ndarr.shape)
    print(shorthead+(string+': '+shape));

def dfspp(string, df:pd.DataFrame):
  if nprt_en is True:
    shape = str(df.shape)
    print(shorthead+(string+': '+shape));

def nlprint(string, list_like:list):
  if nprt_en is True:
    le = str(len(list_like))
    print(shorthead+(string+': '+le));
    # pp(list_like); print(shorttail)

def plh():
  print(longhead);

def pstail():
  print(shorttail)

def plt():
  print(longtail)

def pattn():
  print(attn)

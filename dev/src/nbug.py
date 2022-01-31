
import sys
# import pdb
from pprint import pprint as pp
import numpy as np

# enable nbug print
nprt_en = True #False
nprt_en2 = True



longhead = '\n\-->> '
shorthead = '\-->> '
longtail = '\n\n'
shorttail = '\n'
attn = 'here ----------- <<<<<\n\n'  #print(longhead+attn)

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def nprint(string, *args):
  if nprt_en is True:
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
    # pp(ndarr); print(shorttail)

def nprint_2(string, *args):
  if nprt_en2 is True:
    print(shorthead+(string+': '));
    pp(*args);

def plh():
  print(longhead);

def pstail():
  print(shorttail)

def pltail():
  print(longtail)

def pattn():
  print(attn)

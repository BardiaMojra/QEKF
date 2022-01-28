
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
    print(shorthead+(string+': ')); print(*args);

def nsprint(string, ndarr:np.ndarray):
  if nprt_en is True:
    shape = str(ndarr.shape)
    print(shorthead+(string+': '+shape));
    pp(ndarr); print(shorttail)


def nprint_2(string, *args):
  if nprt_en2 is True:
    print(shorthead+(string+': ')); print(*args);

def lh():
  print(longhead);

def stail():
  print(shorttail)

def ltail():
  print(longtail)

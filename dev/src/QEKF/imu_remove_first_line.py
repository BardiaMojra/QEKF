
import sys
import os

from nbug import *
from pdb import set_trace as st

''' pass in imu dir path
'''

if __name__ == "__main__":
  DIR = sys.argv[1]
  print('remove first line of every file in: ', DIR)
  for i, file in enumerate(os.listdir(DIR)):
    with open(DIR+file, 'r') as fin:

      data = fin.read().splitlines(True)

      fin.close()
    nlprint(file, data)
    if data[0][0] == '#':
      with open(DIR+file, 'w') as fout:
        fout.writelines(data[1:])
        fout.close()
        print(i, file)

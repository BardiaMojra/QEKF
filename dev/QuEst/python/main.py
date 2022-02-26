# import numpy as np
# import matplotlib
import pandas as pd

''' private libraries '''
from dmm import *
from util import *

''' NBUG libraries '''
from pdb import set_trace as st
from nbug import *

''' general config '''
NBUG          = True
_show         = True
_save         = True
_prt          = True
# _START        = 0
# _END          = 150
_ALGORITHMS    = ['QuEst']
_BENCHTYPE    = 'KITTI'
_BENCHNUM     = 3

skipFrame     = 0 # num of frames that are skiped between two key frames
ranThresh     = 1e-6 # RANSAC outlier threshold
surfThresh    = 200 # SURF feature point detection threshold

# Number of feature points
maxPts        = 30 # max num of feature points to use for pose est (lower value increases speed)
minPts        = 6 # min num of feature points required (6 to est a unique pose from RANSAC)

def main():
  global fignum; fignum = int(0)

  # init dataset object
  dset = dmm(_BENCHTYPE,
             _BENCHNUM
            #  start=_START,
            #  end=_END,
            )

  numImag = len(dset.fnames) # total number of images
  i = 1+skipFrame
  keyFrames = list()
  while i < numImag:
    keyFrames.append(i)
    i = i+1+skipFrame

  nprint('keyFrames', keyFrames)
  st()

  numKeyFrames = len(keyFrames) # num of key frames
  numMethods   = len(_ALGORITHMS) # num of algorithms used in the comparison
  # rotErr      = NaN(numKeyFrames, numMethods) # rotation error for each method
  # tranErr     = NaN(numKeyFrames, numMethods) # translation error for each method
  # Q           = cell(numKeyFrames, numMethods) # recovered quaternions
  # T           = cell(numKeyFrames, numMethods) # recovered translations


  ''' recover Pose using RANSAC and compare with ground truth '''
  # Initialize with the first image feature points
  ppoints, Ip = GetFeaturePoints(0, dset, surfThresh)

  nprint('ppoints', ppoints)
  st()

  # init QEKF object
  # quest = QuEst()

  # x_Txyz = dset.t1  # init state vectors
  # x_Qxyz = dset.q1  # init state vectors
  # for i in range(dset.start, dset.end):
  #   # print('    \\--->>> new state ------->>>>>:  ', i)
  #   u_Wrpy = dset.u_Wrpy_np[i].reshape(-1,1)
  #   z_TVQxyz = dset.z_TVQxyzw_np[i,:-1].reshape(-1,1)
  #   z_TVQxyzw = dset.z_TVQxyzw_np[i].reshape(-1,1) # only for data logging
  #   # nsprint('x_TVQxyz', x_TVQxyz)
  #   # nsprint('u_Wrpy', u_Wrpy)
  #   # nsprint('z_TVQxyz', z_TVQxyz)
  #   # st()
  #   x_TVQxyz = qekf.predict(x_TVQxyz, u_Wrpy)
  #   # nsprint('x_TVQxyz', x_TVQxyz)
  #   x_TVQxyz = qekf.update(x_TVQxyz, z_TVQxyz)
  #   # st()
  #   ''' log z state vector '''
  #   qekf.log.log_z_state(z_TVQxyzw, i)
  print('end of qekf data iterator ----->>')


  ''' post processing '''

  return # end of main


if __name__ == "__main__":

  main()
  print('---------   End   ---------')
  exit()

# EOF



import numpy as np
import quaternion
import scipy.linalg as la
from scipy.spatial.transform import Rotation as sc_R



''' NBUG '''
from pdb import set_trace as st
from nbug import *

def get_Txyz_v0100(m,n,q):
  ''' recover translation and depths given the rotation and feature point
  coordinates
  Inputs:
  m, n:  Homogeneous coordinates of matched feature points in the first
         and second coordinate frames. Each column of m or n has the
         format [x, y, 1]^T, where x and y are coordinates of the
         feature point on the image plane.
  Q:     Recovered solutions for rotation, either in quaternion form or
         rotation matrix form. For quaternions, Q is a 4*k matrix, where
         k is the number of solutions. For rotations, Q is a 3*3*k
         structure.
  Outputs:
  T   :  Associated translation vectors.
  Z1  :  Depths in the first camera frame.
  Z2  :  Depths in the second camera frame.
  R   :  Rotation matrix representation.
  NOTE: T, Z1, and Z2 are recovered up to a common scale factor.
  Copyright (C) 2017, by Kaveh Fathian.'''
  # if input is quaternion transform it into the rotation matrix
  nprint('q', q)
  if isinstance(q, np.quaternion):
    R = sc_R.from_quat([q.w,q.x,q.y,q.z]).as_matrix()
  npprint('R', R)
  npprint('m', m)
  npprint('n', n)


  st()



  # numPts  = size(m,2);        % Number of feature points
  # # numInp  = size(R,3);        % Number of rotation matrices in input
  # # T       = zeros(3,numInp);       % Translation vector
  # # Z1      = zeros(numPts,numInp);  % Depth of points in the 1st camera frame
  # # Z2      = zeros(numPts,numInp);  % Depth of points in the 2nd camera frame
  # Res     = zeros(1,numInp);       % Residue from SVD
  numKP = m.shape[1]

  # for k in range(numKP):
  # stack rigid motion constraints into matrix-vector form C * Y = 0
  C = np.zeros((3*numKP, 2*numKP+3))

  for i in range(numKP):

    C[i*3:(i+1)*3, 0:3] = np.eye(3)
    C[i*3:(i+1)*3, i*2+4:i*2+6] = np.asarray([R @ m[:,i], -n[:,i]])
    npprint('i', i)
    npprint('C', C)
    st()

    # obtain singular vectors via svd (solve standard form equation)
    U,S,V = la.svd(C)

    npprint('U', U)
    npprint('S', S)
    npprint('V', V)
    st()

    # singular values
    Sd = np.diag(S)
    npprint('Sd', Sd)
    st()

    # the right singular vector corresponding to the zero singular value of C.
    Y = V[:,-1]

    t = Y[0:3]  # translation vector
    z = Y[3:-1] # depths in both camera frames

    # adjust the sign s.t. the recovered depths are positive
    numPos = sum(z > 0)
    numNeg = sum(z < 0)
    if numPos < numNeg:
        t = -t
        z = -z

    z1 =  z[0 : 2 : -2] # depths in camera frame 1
    z2 =  z[1 : 2 : -1] # depths in camera frame 2

    # store the results
    T  [:,k] = t
    Z1 [:,k] = z1
    Z2 [:,k] = z2
    Res[:,k] = Sd[-1]
  return T, Z1, Z2, R, Res

  # EOF

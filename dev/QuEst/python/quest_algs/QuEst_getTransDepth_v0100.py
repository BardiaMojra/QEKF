

import numpy as np
import quaternion
import scipy.linalg as la
from scipy.spatial.transform import Rotation as sc_R



''' NBUG '''
from pdb import set_trace as st
from nbug import *

def get_Txyz_v0100(kps1,kps2,q,_dtype=np.float128):
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
  if isinstance(q, np.quaternion):
    q_arr = np.asarray([q.w,q.x,q.y,q.z], dtype=_dtype)
    R = sc_R.from_quat(q_arr).as_matrix()
  # npprint('R', R)

  numKP = kps1.shape[1]

  C = np.zeros((3*numKP,2*numKP+3), dtype=_dtype)
  for i in range(numKP):
    C[i*3:(i+1)*3, 0:3] = np.eye(3, dtype=_dtype)
    a1,a2,b1,b2 = i*3,(i+1)*3,0,3
    # npprint(f'1 C[{a1}:{a2},{b1}:{b2}]', C[a1:a2,b1:b2])
    # npprint('C', C)
    # npprint('kps1', kps1)
    # npprint('kps2', kps2)
    A = R @ kps1[:,i].reshape(-1,1)
    B = -kps2[:,i].reshape(-1,1)
    # npprint('A', A)
    # npprint('B', B)
    a1,a2,b1,b2 = i*3,(i+1)*3,i*2+3,i*2+5
    # npprint(f'2 C[{a1}:{a2},{b1}:{b2}]', C[a1:a2,b1:b2])
    C[i*3:(i+1)*3, i*2+3:i*2+5] = np.concatenate((A,B), axis=1)
    # st()
    # npprint('C', C)
    # st()
    # end of for i in range(numKP):
  # npprint('C', C)

  # obtain singular vectors via svd (solve standard form equation)
  U,S,V = la.svd(C)
  S = S.reshape(-1,1)

  # the right singular vector corresponding to the zero singular value of C.
  Y = V[:,-1]
  t = Y[0:3]  # translation vector
  z = Y[3:] # depths in both camera frames

  # adjust the sign s.t. the recovered depths are positive
  numPos = [n for n, c in enumerate(z) if c > 0]
  numNeg = [n for n, c in enumerate(z) if c < 0]
  if len(numPos) < len(numNeg):
    t=-t; z=-z;

  z1 = z[0:-2:2] # depths in camera frame 1
  z2 = z[1::2] # depths in camera frame 2
  # npprint('t', t)
  # npprint('z1', z1)
  # npprint('z2', z2)
  # npprint('S[-1]', S[-1])
  # st()
  return t,z1,z2,S[-1]

  # EOF

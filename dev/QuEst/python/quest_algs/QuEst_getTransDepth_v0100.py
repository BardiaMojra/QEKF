

import numpy as np
import scipy.linalg as la
from scipy.spatial.transform import Rotation as R



''' NBUG '''
from pdb import set_trace as st
from nbug import *

def get_Txyz_v0100(m,n, Q):
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

  npprint('Q', Q)
  st()
  Q = np.random.rand((3,3))
  npprint('Q', Q)
  st()
  # if input is quaternion transform it into the rotation matrix
  if Q.shape == (3,3):
    R = Q
  else:
    R = R.from_quat(Q)



  # I       = eye(3);           % Identity matrix
  # numPts  = size(m,2);        % Number of feature points
  # # numInp  = size(R,3);        % Number of rotation matrices in input
  # # T       = zeros(3,numInp);       % Translation vector
  # # Z1      = zeros(numPts,numInp);  % Depth of points in the 1st camera frame
  # # Z2      = zeros(numPts,numInp);  % Depth of points in the 2nd camera frame
  # Res     = zeros(1,numInp);       % Residue from SVD

  numKP = m.shape[1]
  for k in range(numKP):
    # stack rigid motion constraints into matrix-vector form C * Y = 0
    C = np.zeros(3*numKP, 2*numKP+3)

    for i = 1 : numPts
      C((i-1)*3+1:i*3, 1:3) = I;
      C((i-1)*3+1:i*3, (i-1)*2+4:(i-1)*2+5) = [R(:,:,k)*m(:,i), -n(:,i)];

    # use SVD to find singular vectors
    [~,S,N] = svd(C,0);

    # singular values
    Sd = diag(S);

    # the right singular vector corresponding to the zero singular value of C.
    Y = N(:,end);

    t = Y(1:3)   # translation vector
    z = Y(4:end) # depths in both camera frames

    # adjust the sign s.t. the recovered depths are positive
    numPos = sum(z > 0);
    numNeg = sum(z < 0);
    if numPos < numNeg
        t = -t;
        z = -z;

    z1 =  z(1 : 2 : end-1) # depths in camera frame 1
    z2 =  z(2 : 2 : end) # depths in camera frame 2

    # store the results
    T(:,k) = t;
    Z1(:,k) = z1;
    Z2(:,k) = z2;
    Res(:,k) = Sd(end);
  return T, Z1, Z2, R, Res

  # EOF

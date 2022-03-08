
import numpy as np
import scipy.linalg as linalg

from quest_algs.coefs import CoefsVer3_1_1 as COEFS_V0311



''' NBUG '''
from pdb import set_trace as st
from nbug import *

def QuEst_5Pt_Ver7_8(m,n,_dtype=np.float64):
  ''' QuEst (Quaternion Estimation) algorithm for 5 feature points
  NOTE: Include the folder "Helpers" in Matlab path before execution.
  Inputs: #todo update the comments -- now input is Nx3 (numpy style instead matlab)
  m, n:  Homogeneous coordinates of N feature points in the first
         and second coordinate frames. Each column of m or n has the
         format [x, y, 1]^T, where x and y are coordinates of the
         feature point on the image plane. Thus, m and n are 3*N matrices,
         with one entries in the 3rd row.
  Outputs:
  q   :  The recovered rotation in quaternion.
  NOTE: t, z1, and z2 are recovered up to a common scale factor. Copyright (C)
  2013-2017, by Kaveh Fathian. This program is a free software: you can
  redistribute it and/or modify it under the terms of the GNU lesser General
  Public License as published by the Free Software Foundation, either version 3
  of the License, or any later version. This program is distributed in the hope
  that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details
  <http://www.gnu.org/licenses/>.
  ------------------------------------------------------------------------
  Ver 7_8:
    -  Uses all matrices B1,B2,B3 to find solutions
    -  Slightly slower, but has better accuracy  '''
  # preallocate variables
  Idx = [1, 2, 5, 11, 21, 3, 6, 12, 22, 8, 14, 24, 17, 27, 31, 4, 7, 13, 23, 9,
         15, 25, 18, 28, 32, 10, 16, 26, 19, 29, 33, 20, 30, 34, 35, 2, 5, 11,
         21, 36, 6, 12, 22, 37, 14, 24, 39, 27, 42, 46, 7, 13, 23, 38, 15, 25,
         40, 28, 43, 47, 16, 26, 41, 29, 44, 48, 30, 45, 49, 50, 3, 6, 12, 22,
         37, 8, 14, 24, 39, 17, 27, 42, 31, 46, 51, 9, 15, 25, 40, 18, 28, 43,
         32, 47, 52, 19, 29, 44, 33, 48, 53, 34, 49, 54, 55, 4, 7, 13, 23, 38,
         9, 15, 25, 40, 18, 28, 43, 32, 47, 52, 10, 16, 26, 41, 19, 29, 44, 33,
         48, 53, 20, 30, 45, 34, 49, 54, 35, 50, 55, 56]
  # Construct coefficient matrix
  # coefficient matrix in the linearized system of multinomials (Cf * V = 0)
  Cf = COEFS_V0311(m,n)
  numEq = Cf.shape[0]
  nsprint('Cf', Cf)
  st()
  # A is the coefficient matrix such that A * X = 0
  A = np.zeros((4*numEq,56), dtype=_dtype)
  for i in range(1,5):
      idx = Idx[i-1,:]
      A[(i-1)*numEq+1 : i*numEq, idx] = Cf
      nprint('idx', idx)
      nprint('A', A)
      st()

  # find bases for the null space of A
  _,__,V = linalg.svd(A,0)
  N = V[:,37:56].copy()

  idx = Idx[0,:];   A0 = N[idx-1,:]
  idx = Idx[1,:];   A1 = N[idx-1,:]
  idx = Idx[2,:];   A2 = N[idx-1,:]
  idx = Idx[3,:];   A3 = N[idx-1,:]

  B = A0 / [A1, A2, A3];

  # split B to 3 square matrices
  # B1 = B[ 0:20,:]
  # B2 = B[20:40,:]
  # B3 = B[40:60,:]
  # compute eigenvectors - initial guess
  # [V1, ~] = eig(B1)
  # [V2, ~] = eig(B2)
  # [V3, ~] = eig(B3)
  # Ve = np.concatenate([V1, V2, V3], axis=1)

  # Ve = eig()
  Ve = linalg.eigh(B) # generalized hamiltonian eigen values

  # % #todo for now remove all the imaginary solutions
  # Remove duplicate complex eigenvectors
  Vy = Ve[Ve.imag == 0]
  Vy = Ve[Ve.imag == 0]
  imagIdx = sum(abs(Vy)) > 10*np.finfo(float).eps
  Viall = Ve[imagIdx,:]
  # [~,srtIdx] = sort(real(Viall(1,:)),'ascend');
  srtIdx = sorted(Viall, key=lambda v : v.imag == 0)
  # Vi = Viall(:,srtIdx(1:2:end)) # keep only one complex eigenvector
  Vi = Viall[srtIdx[range(0, len(srtIdx), 2)]] # keep only one complex eigenvector
  Vr = Ve[~imagIdx,:]
  V0 = np.concatenate([Vi, Vr], axis=0).real # Use only the real parts

  ''' extract quaternion elements '''
  # degree 5 monomial solution vectors
  X5 = N @ V0

  # correct the sign of each column s.t. the first element (i.e., w^5) is always positive
  X5 = np.sign(X5.w) * X5

  # recover quaternion elements
  # w = nthroot(X5(1,:),5);
  w = X5.w ** (1/5)
  w4 = w ** 4
  x = X5.x / w4
  y = X5.y / w4
  z = X5.z / w4

  nppshape('w', w)
  nppshape('x', x)
  nppshape('y', y)
  nppshape('z', z)
  nprint('w', w)
  nprint('x', x)
  nprint('y', y)
  nprint('z', z)

  Q = np.quaternion(w, x, y, z)
  nppshape('Q', Q)
  nprint('Q', Q)

  Q = Q.norm()

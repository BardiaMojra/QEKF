''' QuEst v07.08 '''

''' public libraries '''
import numpy as np
import quaternion
import scipy.linalg as la

''' private libraries '''
from quest_algs.coefs import CoefsVer3_1_1 as get_COEFS
from utils import *

''' NBUG '''
from pdb import set_trace as st
from nbug import *


def QuEst_5Pt_Ver7_8(dat,_dtype=np.float128):
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
    -  Slightly slower, but has better accuracy '''

  ''' config '''
  ZERO_THRESHOLD = _dtype(2.220446049250313080847263336181640625e-15)
  NO_IMAGINARY_ROOTS = False
  # constants
  Idx = np.asarray([1, 2, 5, 11, 21, 3, 6, 12, 22, 8, 14, 24, 17, 27, 31, 4, 7,
                    13, 23, 9, 15, 25, 18, 28, 32, 10, 16, 26, 19, 29, 33, 20,
                    30, 34, 35, 2, 5, 11, 21, 36, 6, 12, 22, 37, 14, 24, 39, 27,
                    42, 46, 7, 13, 23, 38, 15, 25, 40, 28, 43, 47, 16, 26, 41,
                    29, 44, 48, 30, 45, 49, 50, 3, 6, 12, 22, 37, 8, 14, 24, 39,
                    17, 27, 42, 31, 46, 51, 9, 15, 25, 40, 18, 28, 43, 32, 47,
                    52, 19, 29, 44, 33, 48, 53, 34, 49, 54, 55, 4, 7, 13, 23,
                    38, 9, 15, 25, 40, 18, 28, 43, 32, 47, 52, 10, 16, 26, 41,
                    19, 29, 44, 33, 48, 53, 20, 30, 45, 34, 49, 54, 35, 50, 55,
                    56], dtype=int).reshape(4,-1) - 1

  # npprint('dat', dat)
  m = dat[:,:3].T
  n = dat[:,3:].T
  # npprint('m',m)
  # npprint('n',n)
  # st()
  ''' Data integrity: matches line 64 of QuEst_RANSAC_Ver1_2.m '''

  # coefficient matrix in the linearized system of multi-nomials (Cf * V = 0)
  Cf = get_COEFS(m,n)
  # write_np2txt(Cf,fname='QuEst_Cf_py.txt')
  # write_np2txt(Idx+1,fname='QuEst_Idx_py.txt',_dtype=int)
  numEq = Cf.shape[0]
  # A is the coefficient matrix such that A * X = 0
  A = np.zeros((4*numEq,56), dtype=_dtype)

  #todo: bug here
  for i in range(1,5):
    idx = Idx[i-1,:]
    A[(i-1)*numEq : i*numEq, idx] = Cf

  #A = A.T # correction

  # write_np2txt(A,fname='QuEst_A_py.txt') # matched matlab
  # npprint('A',A)
  # find bases for the null space of A
  # U,s,VT = la.svd(A, lapack_driver='gesvd')
  U,s,VT = la.svd(A,full_matrices=False, lapack_driver='gesdd')
  S = np.diag(s)
  V = VT.T

  # nsprint('U',U)
  # nsprint('S',S)
  # nsprint('V.T',V.T)
  # nsprint('V',V)
  # A_rcon = U @ S @ V.T
  # A_rcon = np.where(A_rcon>ZERO_THRESHOLD,A_rcon,0)
  #todo working here ....................
  # write_n0p2txt(V[:10,:],fname='QuEst_V_fi10_gesdd_py.txt')
  # write_np2txt(V[-10:,:],fname='QuEst_V_la10_gesdd_py.txt')
  # write_np2txt(V,fname='QuEst_V_gesdd_py.txt')
  # write_np2txt(S,fname='QuEst_S_gesdd_py.txt')
  # write_np2txt(A_rcon,fname='QuEst_A_rcon_py.txt')

  N = V[:,-20:]
  # write_np2txt(N,fname='QuEst_N_py.txt')

  # nsprint('N',N)
  # show_eigenVal_energies(S)
  # st()


  idx = Idx[0,:];   A0 = N[idx,:]
  idx = Idx[1,:];   A1 = N[idx,:]
  idx = Idx[2,:];   A2 = N[idx,:]
  idx = Idx[3,:];   A3 = N[idx,:]
  # st()
  A_ = np.concatenate((A1, A2, A3), axis=1)

  B = la.pinv(A0) @ A_# perform a left matrix division
  # nsprint('B', B)
  # npprint('B', B)

  B1 = B[:, 0:20].copy()
  B2 = B[:,20:40].copy()
  B3 = B[:,40:60].copy()
  # nsprint('B1', B1)
  # nsprint('B2', B2)
  # nsprint('B3', B3)
  # st()
  # write_np2txt(A0,fname='QuEst_A0_py.txt')
  # write_np2txt(A1,fname='QuEst_A1_py.txt')
  # write_np2txt(A2,fname='QuEst_A2_py.txt')
  # write_np2txt(A3,fname='QuEst_A3_py.txt')
  # write_np2txt( B,fname='QuEst_B_py.txt')

  # st()


  # compute eigenvectors - initial guess
  evals, V1 = la.eig(B1)
  evals, V2 = la.eig(B2)
  evals, V3 = la.eig(B3)

  # write_np2txt(V1,fname='QuEst_V1_py.txt')
  # write_np2txt(V2,fname='QuEst_V2_py.txt')
  # write_np2txt(V3,fname='QuEst_V3_py.txt')

  # st()

  Ve = np.concatenate((V1,V2,V3), axis=1)
  Ve_img = Ve.imag
  # get idx of img roots w magnitude greater than ZERO_ROOT_THRESHOLD
  Vi_idx = list()
  Vr_idx = list()
  for i in range(Ve_img.shape[1]):
    # npprint('Ve_img[:,i]', Ve_img[:,i])
    Ve_img_sum = (sum(abs(Ve_img[:,i])))

    if Ve_img_sum > ZERO_THRESHOLD: Vi_idx.append(i)
    else: Vr_idx.append(i)
  # indices should be a complement
  Vi_idx = np.asarray(Vi_idx, dtype=np.int64)
  Vr_idx = np.asarray(Vr_idx, dtype=np.int64)
  # npprint('Vi_idx', Vi_idx)
  # npprint('Vr_idx', Vr_idx)
  # st()
  Vi_all = Ve[:,Vi_idx] # extract all imaginary roots
  Vi_idx = sorted(range(Vi_all.shape[1]), key=lambda x:Vi_all[0,:].real[x])
  # st()
  Vi_idx_keep = [Vi_idx[x] for x in range(0, len(Vi_idx), 2)]
  Vi = Vi_all[:, Vi_idx_keep]
  Vr = Ve[:, Vr_idx]
  if NO_IMAGINARY_ROOTS:
    V0 = np.concatenate((Vi,Vr), axis=1).real
    nprint('using imaginary roots', Vi_idx_keep)
  else:
    V0 = Vr


  ''' extract quaternion elements in order to obtained relative rotation '''
  X5 = N @ V0 # degree 5 monomial solution vectors
  # correct quat sign i.e. w is always positive
  for i, q in enumerate(X5[0,:]):
    if X5[0,i] < 0: X5[:,i] = -X5[:,i]
  # recover quaternion elements
  w = (X5[0,:] ** (1/5)).reshape(1,-1)
  w4 = w ** 4
  x = X5[1,:] / w4
  y = X5[2,:] / w4
  z = X5[3,:] / w4
  qs = np.concatenate((w,x,y,z), axis=0, dtype=np.complex128)
  qs = np.float128(qs)
  Qnorm = np.sqrt(np.sum(qs**2, axis=0)).reshape(1,-1)
  qs = qs/Qnorm # match matlab format

  # write_np2txt(V0,fname='QuEst_V0_py.txt')
  # write_np2txt(X5,fname='QuEst_X5_py.txt')
  # write_np2txt(qs,fname='QuEst_Qs_py.txt')


  return quaternion.as_quat_array(qs.T).reshape(-1,1) #

def get_qs_residues(dat, qSols):
  ''' calculate the residue value |C x - c| for estimated quaternion solutions
  '''
  # npprint('dat', dat)
  m1 = dat[:,:3].T
  m2 = dat[:,3:].T
  npprint('m1',m1)
  npprint('m2',m2)
  # coefficinet matrix in the linearized system of multinomials (C@x=c)
  C0 = get_COEFS(m1,m2)

  qs_np = quats2np(qSols,axis=1).T
  nsprint('qs_np',qs_np)

  q1 = qs_np[0,:]
  q2 = qs_np[1,:]
  q3 = qs_np[2,:]
  q4 = qs_np[3,:]

  # nsprint('q1',q1)
  # nsprint('q2',q2)
  # nsprint('q3',q3)
  # nsprint('q4',q4)
  # vector of moninals
  xVec = np.asarray([q1**4,
                     q1**3. * q2,
                     q1**2. * q2**2,
                     q1 * q2**3,
                     q2**4,
                     q1**3. *q3,
                     q1**2. *q2 *q3,
                     q1* q2**2. *q3,
                     q2**3. *q3,
                     q1**2. * q3**2,
                     q1*q2 * q3**2,
                     q2**2. * q3**2,
                     q1* q3**3,
                     q2* q3**3,
                     q3**4,
                     q1**3. *q4,
                     q1**2. *q2*q4,
                     q1*q2**2. *q4,
                     q2**3. *q4,
                     q1**2. *q3*q4,
                     q1*q2 * q3*q4,
                     q2**2. * q3*q4,
                     q1* q3**2. *q4,
                     q2* q3**2. *q4,
                     q3**3. *q4,
                     q1**2. *q4**2,
                     q1*q2 * q4**2,
                     q2**2. * q4**2,
                     q1*q3 * q4**2,
                     q2*q3 * q4**2,
                     q3**2. * q4**2,
                     q1 * q4**3,
                     q2 * q4**3,
                     q3 * q4**3,
                     q4**4], dtype=np.float128)
  # nsprint('xVec', xVec)
  # nsprint('C0',C0)
  # residues
  residuMat = C0 @ xVec
  # npprint('residuMat',residuMat)

  residu = np.sum(np.abs(residuMat),axis=0)
  # npprint('residu',residu)

  return residu
# EOF

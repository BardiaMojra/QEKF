import numpy as np
import scipy as sc

''' NBUG libraries '''
from pdb import set_trace as st
from nbug import *

''' general config '''

def CoefsVer3_1_1(m1, m2, _dtype=np.float64):
  ''' Five Point Pose Estimation using Quaternion
    Fast way of generating the coefficient matrix C from the image points.
    Inputs: #todo update the comments -- now input is Nx3 (numpy style instead matlab)
      m1:   Matrix containing the homogeneous coordinates of feature points in
            the 1st camera frame.
      m2:   Matrix containing the homogeneous coordinates of feature points in
            the 2nd camera frame.
    Output:
      C:    The 11*35 coefficient matrix
    Copyright (C) 2013-2017, by Kaveh Fathian.
    ------------------------------------------------------------------------
    Input:
      - 3xN array of pure quaternion, N equations
    ------------------------------------------------------------------------
    Ver 3_1_1:
      - Based on Ver3_1, without the unit norm coefficient vector appended at
      the end. '''
  numPts = m1.shape[1]
  idx1 = np.zeros((int(sc.special.binom(numPts,2))-1, 2), dtype=int)
  cntr = 0
  for i in range(numPts-2):
    for j in range(i+1, numPts):
      idx1[cntr] = [i,j]
      cntr += 1
  idx1 = idx1.T

  mx1 = m1[0,idx1[0,:]].reshape(-1,1); my1 = m1[1,idx1[0,:]].reshape(-1,1);
  nx1 = m2[0,idx1[0,:]].reshape(-1,1); ny1 = m2[1,idx1[0,:]].reshape(-1,1);
  mx2 = m1[0,idx1[1,:]].reshape(-1,1); my2 = m1[1,idx1[1,:]].reshape(-1,1);
  nx2 = m2[0,idx1[1,:]].reshape(-1,1); ny2 = m2[1,idx1[1,:]].reshape(-1,1);
  s1  = m1[2,idx1[0,:]].reshape(-1,1); r1  = m2[2,idx1[0,:]].reshape(-1,1);
  s2  = m1[2,idx1[1,:]].reshape(-1,1); r2  = m2[2,idx1[1,:]].reshape(-1,1);

  # get coefficients for numerator (cN) and denominator (cD)
  cN = coefsNumVer2_0(mx1,mx2,my1,my2,nx2,ny2,r2,s1,s2,_dtype=_dtype)
  cD = coefsDenVer2_0(mx2,my2,nx1,nx2,ny1,ny2,r1,r2,s2,_dtype=_dtype)

  # npprint('coefsN', coefsN)
  # npprint('coefsD', coefsD)

  # total number of equations
  numEq = int(sc.special.binom(numPts,3))
  # idxBin2 = np.zeros((numEq, 2), dtype=int)
  idx2 = np.asarray([1,1,1,2,2,3,5,5,6,8,\
                        2,3,4,3,4,4,6,7,7,9], dtype=int).reshape(2,-1)

  idx2 = idx2 - 1
  # npprint('idxBin2', idx2)
  # npprint('coefsN', coefsN)
  # npprint('coefsD', cD)
  # npprint('coefsD[idxBin2[0,:], 0]', cD[idx2[0,:], 0])

  a1  = np.concatenate((cN[idx2[0,:],0],cD[idx2[0,:],0]),axis=0).reshape(-1,1)
  a2  = np.concatenate((cN[idx2[0,:],1],cD[idx2[0,:],1]),axis=0).reshape(-1,1)
  a3  = np.concatenate((cN[idx2[0,:],2],cD[idx2[0,:],2]),axis=0).reshape(-1,1)
  a4  = np.concatenate((cN[idx2[0,:],3],cD[idx2[0,:],3]),axis=0).reshape(-1,1)
  a5  = np.concatenate((cN[idx2[0,:],4],cD[idx2[0,:],4]),axis=0).reshape(-1,1)
  a6  = np.concatenate((cN[idx2[0,:],5],cD[idx2[0,:],5]),axis=0).reshape(-1,1)
  a7  = np.concatenate((cN[idx2[0,:],6],cD[idx2[0,:],6]),axis=0).reshape(-1,1)
  a8  = np.concatenate((cN[idx2[0,:],7],cD[idx2[0,:],7]),axis=0).reshape(-1,1)
  a9  = np.concatenate((cN[idx2[0,:],8],cD[idx2[0,:],8]),axis=0).reshape(-1,1)
  a10 = np.concatenate((cN[idx2[0,:],9],cD[idx2[0,:],9]),axis=0).reshape(-1,1)

  b1  = np.concatenate((cD[idx2[1,:],0],cN[idx2[1,:],0]),axis=0).reshape(-1,1)
  b2  = np.concatenate((cD[idx2[1,:],1],cN[idx2[1,:],1]),axis=0).reshape(-1,1)
  b3  = np.concatenate((cD[idx2[1,:],2],cN[idx2[1,:],2]),axis=0).reshape(-1,1)
  b4  = np.concatenate((cD[idx2[1,:],3],cN[idx2[1,:],3]),axis=0).reshape(-1,1)
  b5  = np.concatenate((cD[idx2[1,:],4],cN[idx2[1,:],4]),axis=0).reshape(-1,1)
  b6  = np.concatenate((cD[idx2[1,:],5],cN[idx2[1,:],5]),axis=0).reshape(-1,1)
  b7  = np.concatenate((cD[idx2[1,:],6],cN[idx2[1,:],6]),axis=0).reshape(-1,1)
  b8  = np.concatenate((cD[idx2[1,:],7],cN[idx2[1,:],7]),axis=0).reshape(-1,1)
  b9  = np.concatenate((cD[idx2[1,:],8],cN[idx2[1,:],8]),axis=0).reshape(-1,1)
  b10 = np.concatenate((cD[idx2[1,:],9],cN[idx2[1,:],9]),axis=0).reshape(-1,1)

  # npprint('b2', b2)
  # npprint('b1', b1)

  # st()

  # coefsND = [num1 * den2;
  #            den1 * num2];
  coefsND = coefsNumDen(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,\
                        b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,\
                        _dtype=_dtype);

  # nprint('numEq', numEq)
  # npprint('coefsND[:numEq,:]', coefsND[:numEq,:])
  # st()
  # matrix of all coefficients
  # coefs = (num1 * den2)  -  (den1 * num2)
  C = coefsND[:numEq,:] - coefsND[numEq:2*numEq,:]
  # npprint('C', C)
  # st()
  return C

def coefsNumVer2_0(mx1,mx2,my1,my2,nx2,ny2,r2,s1,s2, _dtype=np.float64):
  t2  = mx1 * my2 * r2
  t3  = mx2 * ny2 * s1
  t4  = my1 * nx2 * s2
  t5  = mx1 * nx2 * s2  * 2.0
  t6  = my1 * ny2 * s2  * 2.0
  t7  = mx1 * my2 * nx2 * 2.0
  t8  = my2 * r2  * s1  * 2.0
  t9  = mx2 * my1 * r2
  t10 = mx1 * ny2 * s2
  t11 = mx2 * my1 * ny2 * 2.0
  t12 = mx2 * r2  * s1  * 2.0
  t13 = my2 * nx2 * s1
  # compute coefficient columns for the numerator
  cN0 = t2+t3+t4-mx2*my1*r2-mx1*ny2*s2-my2*nx2*s1
  cN1 = t11+t12-mx1*my2*ny2*2.0-mx1*r2*s2*2.0
  cN2 = t7+t8-mx2*my1*nx2*2.0-my1*r2*s2*2.0
  cN3 = t5+t6-mx2*nx2*s1*2.0-my2*ny2*s1*2.0
  cN4 = -t2-t3+t4+t9+t10-my2*nx2*s1
  cN5 = -t5+t6+mx2*nx2*s1*2.0-my2*ny2*s1*2.0
  cN6 = t7-t8-mx2*my1*nx2*2.0+my1*r2*s2*2.0
  cN7 = -t2+t3-t4+t9-t10+t13
  cN8 = -t11+t12+mx1*my2*ny2*2.0-mx1*r2*s2*2.0
  cN9 = t2-t3-t4-t9+t10+t13
  coefsN = np.concatenate((cN0,cN1,cN2,cN3,cN4,cN5,cN6,cN7,cN8,cN9), axis=1,\
    dtype=_dtype)
  return coefsN

def coefsDenVer2_0(mx2,my2,nx1,nx2,ny1,ny2,r1,r2,s2,_dtype=np.float64):
  t2  = mx2 * ny1 * r2
  t3  = my2 * nx2 * r1
  t4  = nx1 * ny2 * s2
  t5  = mx2 * nx2 * r1  * 2.0
  t6  = my2 * ny2 * r1  * 2.0
  t7  = mx2 * nx2 * ny1 * 2.0
  t8  = ny1 * r2  * s2  * 2.0
  t9  = my2 * nx1 * r2
  t10 = nx2 * ny1 * s2
  t11 = my2 * nx1 * ny2 * 2.0
  t12 = nx1 * r2  * s2  * 2.0
  t13 = mx2 * ny2 * r1
  # compute coefficient columns for the denominator
  cD0 = t2+t3+t4-mx2*ny2*r1-my2*nx1*r2-nx2*ny1*s2
  cD1 = t11+t12-my2*nx2*ny1*2.0-nx2*r1*s2*2.0
  cD2 = t7+t8-mx2*nx1*ny2*2.0-ny2*r1*s2*2.0
  cD3 = t5+t6-mx2*nx1*r2*2.0-my2*ny1*r2*2.0
  cD4 = t2-t3-t4+t9+t10-mx2*ny2*r1
  cD5 = t5-t6-mx2*nx1*r2*2.0+my2*ny1*r2*2.0
  cD6 = -t7+t8+mx2*nx1*ny2*2.0-ny2*r1*s2*2.0
  cD7 = -t2+t3-t4-t9+t10+t13
  cD8 = t11-t12-my2*nx2*ny1*2.0+nx2*r1*s2*2.0
  cD9 = -t2-t3+t4+t9-t10+t13
  coefsD = np.concatenate((cD0,cD1,cD2,cD3,cD4,cD5,cD6,cD7,cD8,cD9),axis=1,\
    dtype=_dtype)
  return coefsD

def coefsNumDen(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,\
                _dtype=np.float64):
  A00 = a1*b1
  A01 = a1*b2+a2*b1
  A02 = a2*b2+a1*b5+a5*b1
  A03 = a2*b5+a5*b2
  A04 = a5*b5
  A05 = a1*b3+a3*b1
  A06 = a2*b3+a3*b2+a1*b6+a6*b1
  A07 = a2*b6+a3*b5+a5*b3+a6*b2
  A08 = a5*b6+a6*b5
  A09 = a3*b3+a1*b8+a8*b1
  A10 = a3*b6+a6*b3+a2*b8+a8*b2
  A11 = a6*b6+a5*b8+a8*b5
  A12 = a3*b8+a8*b3
  A13 = a6*b8+a8*b6
  A14 = a8*b8
  A15 = a1*b4+a4*b1
  A16 = a2*b4+a4*b2+a1*b7+a7*b1
  A17 = a2*b7+a4*b5+a5*b4+a7*b2
  A18 = a5*b7+a7*b5
  A19 = a3*b4+a4*b3+a1*b9+a9*b1
  A20 = a3*b7+a4*b6+a6*b4+a7*b3+a2*b9+a9*b2
  A21 = a6*b7+a7*b6+a5*b9+a9*b5
  A22 = a3*b9+a4*b8+a8*b4+a9*b3
  A23 = a6*b9+a7*b8+a8*b7+a9*b6
  A24 = a8*b9+a9*b8
  A25 = a4*b4+a1*b10+a10*b1
  A26 = a4*b7+a7*b4+a2*b10+a10*b2
  A27 = a7*b7+a5*b10+a10*b5
  A28 = a3*b10+a4*b9+a9*b4+a10*b3
  A29 = a6*b10+a7*b9+a9*b7+a10*b6
  A30 = a8*b10+a9*b9+a10*b8
  A31 = a4*b10+a10*b4
  A32 = a7*b10+a10*b7
  A33 = a9*b10+a10*b9
  A34 = a10*b10
  coefsND = np.concatenate((A00,A01,A02,A03,A04,A05,A06,A07,A08,A09,\
                            A10,A11,A12,A13,A14,A15,A16,A17,A18,A19,\
                            A20,A21,A22,A23,A24,A25,A26,A27,A28,A29,\
                            A30,A31,A32,A33,A34), axis=1, dtype=_dtype)
  # npprint('coefsND', coefsND)
  # st()
  return coefsND
  # EOF

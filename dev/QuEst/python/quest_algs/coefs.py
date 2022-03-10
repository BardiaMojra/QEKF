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
  # number of feature points

  #todo working here
  nprint('m1', m1)
  nprint('m2', m2)
  nsprint('m1', m1)
  numPts = m1.shape[1]
  nprint('sc.special.binom(numPts,2)', sc.special.binom(numPts,2))
  # st()
  idxBin1 = np.zeros((2, int(sc.special.binom(numPts,2))-1), dtype=_dtype)
  cntr = 0
  for i in range(numPts-2):
    for j in range(i+1, numPts):
      idxBin1[cntr] = [i,j]
      cntr += 1

  nprint('idxBin1', idxBin1)
  st()

  # image size 375, 1242

  mx1 = m1[idxBin1[:,0],0].T;    my1 = m1[idxBin1[:,0],1].T;
  nx1 = m2[idxBin1[:,0],0].T;    ny1 = m2[idxBin1[:,0],1].T;
  mx2 = m1[idxBin1[:,1],0].T;    my2 = m1[idxBin1[:,1],1].T;
  nx2 = m2[idxBin1[:,1],0].T;    ny2 = m2[idxBin1[:,1],1].T;
  s1  = m1[idxBin1[:,0],2].T;    r1  = m2[idxBin1[:,0],2].T;
  s2  = m1[idxBin1[:,1],2].T;    r2  = m2[idxBin1[:,1],2].T;

  coefsN = coefsNumVer2_0(mx1,mx2,my1,my2,nx2,ny2,r2,s1,s2,dtype=_dtype)
  coefsD = coefsDenVer2_0(mx2,my2,nx1,nx2,ny1,ny2,r1,r2,s2,dtype=_dtype)


  # Total number of equations
  numEq = sc.special.binom(numPts,3)
  idxBin2 = np.zeros(2,numEq)
  cntr = 0
  counter2 = 0
  for i in range(numPts-1,2,-1):
    for j in range(1+counter2, i-1+counter2):
      for k in range(j+1, i+counter2):
        cntr = cntr + 1;
        idxBin2[cntr,:] = [j,k]
    counter2 += i

  # ai = [num1;
  #       den1];
  a1  = [coefsN[idxBin2[:,1], 1], coefsD[idxBin2[:,1], 1]]
  a2  = [coefsN[idxBin2[:,1], 2], coefsD[idxBin2[:,1], 2]]
  a3  = [coefsN[idxBin2[:,1], 3], coefsD[idxBin2[:,1], 3]]
  a4  = [coefsN[idxBin2[:,1], 4], coefsD[idxBin2[:,1], 4]]
  a5  = [coefsN[idxBin2[:,1], 5], coefsD[idxBin2[:,1], 5]]
  a6  = [coefsN[idxBin2[:,1], 6], coefsD[idxBin2[:,1], 6]]
  a7  = [coefsN[idxBin2[:,1], 7], coefsD[idxBin2[:,1], 7]]
  a8  = [coefsN[idxBin2[:,1], 8], coefsD[idxBin2[:,1], 8]]
  a9  = [coefsN[idxBin2[:,1], 9], coefsD[idxBin2[:,1], 9]]
  a10 = [coefsN[idxBin2[:,1],10], coefsD[idxBin2[:,1],10]]

  # bi = [num2;
  #       den2];
  b1  = [coefsD[idxBin2[:,2], 1], coefsN[idxBin2[:,2], 1]]
  b2  = [coefsD[idxBin2[:,2], 2], coefsN[idxBin2[:,2], 2]]
  b3  = [coefsD[idxBin2[:,2], 3], coefsN[idxBin2[:,2], 3]]
  b4  = [coefsD[idxBin2[:,2], 4], coefsN[idxBin2[:,2], 4]]
  b5  = [coefsD[idxBin2[:,2], 5], coefsN[idxBin2[:,2], 5]]
  b6  = [coefsD[idxBin2[:,2], 6], coefsN[idxBin2[:,2], 6]]
  b7  = [coefsD[idxBin2[:,2], 7], coefsN[idxBin2[:,2], 7]]
  b8  = [coefsD[idxBin2[:,2], 8], coefsN[idxBin2[:,2], 8]]
  b9  = [coefsD[idxBin2[:,2], 9], coefsN[idxBin2[:,2], 9]]
  b10 = [coefsD[idxBin2[:,2],10], coefsN[idxBin2[:,2],10]]

  # coefsND = [num1 * den2;
  #            den1 * num2];
  coefsND = coefsNumDen(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,\
                        b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,dtype=_dtype);

  # matrix of all coefficients
  # coefs = (num1 * den2)  -  (den1 * num2)
  C = coefsND[:numEq-1,:] - coefsND[numEq:2*numEq-1,:]
  return C

def coefsNumVer2_0(mx1,mx2,my1,my2,nx2,ny2,r2,s1,s2, _dtype=np.float64):
  t2  = mx1 * my2 * r2
  t3  = mx2 * ny2 * s1
  t4  = my1 * nx2 * s2
  t5  = mx1 * nx2 * s2 * 2.0
  t6  = my1 * ny2 * s2 * 2.0
  t7  = mx1 * my2 * nx2 * 2.0
  t8  = my2 * r2  * s1 * 2.0
  t9  = mx2 * my1 * r2
  t10 = mx1 * ny2 * s2
  t11 = mx2 * my1 * ny2 * 2.0
  t12 = mx2 * r2  * s1 * 2.0
  t13 = my2 * nx2 * s1
  coefsN = np.array([t2+t3+t4-mx2*my1*r2-mx1*ny2*s2-my2*nx2*s1,
                     t11+t12-mx1*my2*ny2*2.0-mx1*r2*s2*2.0,
                     t7+t8-mx2*my1*nx2*2.0-my1*r2*s2*2.0,
                     t5+t6-mx2*nx2*s1*2.0-my2*ny2*s1*2.0,
                     -t2-t3+t4+t9+t10-my2*nx2*s1,
                     -t5+t6+mx2*nx2*s1*2.0-my2*ny2*s1*2.0,
                     t7-t8-mx2*my1*nx2*2.0+my1*r2*s2*2.0,
                     -t2+t3-t4+t9-t10+t13,
                     -t11+t12+mx1*my2*ny2*2.0-mx1*r2*s2*2.0,
                     t2-t3-t4-t9+t10+t13], dtype=_dtype)

  return coefsN


def coefsDenVer2_0(mx2,my2,nx1,nx2,ny1,ny2,r1,r2,s2,_dtype=np.float64):
  t2  = mx2 * ny1 * r2
  t3  = my2 * nx2 * r1
  t4  = nx1 * ny2 * s2
  t5  = mx2 * nx2 * r1 * 2.0
  t6  = my2 * ny2 * r1 * 2.0
  t7  = mx2 * nx2 * ny1 * 2.0
  t8  = ny1 * r2  * s2 * 2.0
  t9  = my2 * nx1 * r2
  t10 = nx2 * ny1 * s2
  t11 = my2 * nx1 * ny2 * 2.0
  t12 = nx1 * r2  * s2  * 2.0
  t13 = mx2 * ny2 * r1
  coefsD = np.array([t2+t3+t4-mx2*ny2*r1-my2*nx1*r2-nx2*ny1*s2,
                     t11+t12-my2*nx2*ny1*2.0-nx2*r1*s2*2.0,
                     t7+t8-mx2*nx1*ny2*2.0-ny2*r1*s2*2.0,
                     t5+t6-mx2*nx1*r2*2.0-my2*ny1*r2*2.0,
                     t2-t3-t4+t9+t10-mx2*ny2*r1,
                     t5-t6-mx2*nx1*r2*2.0+my2*ny1*r2*2.0,
                     -t7+t8+mx2*nx1*ny2*2.0-ny2*r1*s2*2.0,
                     -t2+t3-t4-t9+t10+t13,
                     t11-t12-my2*nx2*ny1*2.0+nx2*r1*s2*2.0,
                     -t2-t3+t4+t9-t10+t13], dtype=_dtype)

  return coefsD


def coefsNumDen(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,
                _dtype=np.float64):
  coefsND = np.array([a1*b1, a1*b2+a2*b1, a2*b2+a1*b5+a5*b1, a2*b5+a5*b2, a5*b5,
             a1*b3+a3*b1, a2*b3+a3*b2+a1*b6+a6*b1, a2*b6+a3*b5+a5*b3+a6*b2,
             a5*b6+a6*b5, a3*b3+a1*b8+a8*b1, a3*b6+a6*b3+a2*b8+a8*b2,
             a6*b6+a5*b8+a8*b5, a3*b8+a8*b3, a6*b8+a8*b6, a8*b8, a1*b4+a4*b1,
             a2*b4+a4*b2+a1*b7+a7*b1, a2*b7+a4*b5+a5*b4+a7*b2, a5*b7+a7*b5,
             a3*b4+a4*b3+a1*b9+a9*b1, a3*b7+a4*b6+a6*b4+a7*b3+a2*b9+a9*b2,
             a6*b7+a7*b6+a5*b9+a9*b5, a3*b9+a4*b8+a8*b4+a9*b3,
             a6*b9+a7*b8+a8*b7+a9*b6, a8*b9+a9*b8, a4*b4+a1*b10+a10*b1,
             a4*b7+a7*b4+a2*b10+a10*b2, a7*b7+a5*b10+a10*b5,
             a3*b10+a4*b9+a9*b4+a10*b3, a6*b10+a7*b9+a9*b7+a10*b6,
             a8*b10+a9*b9+a10*b8, a4*b10+a10*b4, a7*b10+a10*b7,
             a9*b10+a10*b9, a10*b10], dtype=_dtype)
  return coefsND

import numpy as np
import scipy as sc #
import scipy.linalg as lalg
from scipy.spatial.transform import Rotation as sc_R



''' NBUG libraries '''
from nbug import *
from pdb import set_trace as st

''' private libraries '''
from quest import *
from utils import *




class rmodel(object):
  def __init__(self,q,t,m1,m2,F):
    self.q  = q
    self.t  = t
    self.m1 = m1
    self.m2 = m2
    self.F  = F
  # end of class rmodel(object): ------------------->> //

class QUEST_RANSAC:
  """QuEst_RANSAC estimates the pose between two camera views using RANSAC and
  QuEst algorithm. Fit model parameters to data using the RANSAC algorithm
    This implementation written from pseudocode found at
    http://en.wikipedia.org/w/index.php?title=RANSAC&oldid=116358182
    input:
      data - a set of observed data points
      model - a model that can be fitted to data points
      datum_min - min number of data values required to fit the model
      iters_max  - max number of iterations allowed in the algorithm
      threshold - threshold value for determining when a data point fits a model
      inliers_min - the number of close data values required to assert that a model fits well to data
    return:
      bestfit - model parameters which best fit the data """
  def __init__(self,m1,m2, quest_model, max_iters,threshold,min_inliers,\
               nbug=True,return_all=True,_dtype=np.float128):
    self.m1 = m1  # inputs X
    self.m2 = m2  # outputs Y
    self.max_iters = max_iters
    self.threshold = threshold
    self.min_inliers = min_inliers
    self.NBUG = nbug
    self.return_all = return_all
    self.dtype = _dtype

  def get_best_fit(self):
    st()

    # todo working here .........


    iterations = 0
    bestfit = None
    besterr = np.inf
    best_inlier_idxs = None
    for i in range(self.max_iters):
      #todo explore ways to get points with SOME distance from each other
      #todo test enforcing min distance between correspondences
      maybe_idxs = self.random_partition(self.min_inliers,self.m1.shape[0])

      maybeinliers = self.m1[:,maybe_idxs]



      maybemodel = self.fit(maybeinliers)
      alsoinliers,errs,bestM = self.get_error(maybemodel)


      if self.NBUG:
        print ('test_err.min()'+errs.min())
        print ('test_err.max()'+errs.max())
        print ('np.mean(test_err)'+np.mean(errs))
        print ('iteration: {}', iterations)
        print('len(alsoinliers): {}', len(alsoinliers))
        print('\n\n')
      if len(alsoinliers) > self.min_inliers:
        betterdata = np.concatenate( (maybeinliers, alsoinliers) )
        bettermodel = self.fit(betterdata)
        better_errs = self.get_error( betterdata, bettermodel)
        thiserr = np.mean( better_errs )
        if thiserr < besterr:
          bestfit = bettermodel
          besterr = thiserr
          best_inlier_idxs = np.concatenate( (maybe_idxs, also_idxs) )
      iterations+=1
    if bestfit is None:
      raise ValueError("did not meet fit acceptance criteria")
    if self.return_all:
      return bestfit, {'inliers':best_inlier_idxs}
    else:
      return bestfit


  def fit(self,m1,m2,_dtype=np.float128):
    ''' pose estimation: estimates the relative rotation and translation
    between two camera views using the 5 point quaternion algorithm.
    Input:
      x1, x2 - Two sets of N matched feature point coordinates. x1, x2
               are 3xN matrices with each column corresponding to a point.
      x      - If a single argument is supplied it is assumed that it
               is in the form x = [x1; x2]
    Output:
      M      - A structure that contains the estimated pose.
    Copyright (c) 2016, Kaveh Fathian. The University of Texas at Dallas. '''
    # A = np.vstack([data[:,i] for i in self.input_columns]).T
    # B = np.vstack([data[:,i] for i in self.output_columns]).T
    # npprint('A',A)
    # npprint('',A)
    # x,resids,rank,s = scipy.linalg.lstsq(A,B) # replace with QuEst
    # est pose with QuEst
    qs = QuEst(m=m1, n=m2)
    npprint('qs', qs)
    st()
    # pose = QuEst_Ver1_1(A,B) # run QuEst algorithm
    # find the best solution
    qs_residues = get_residues(m1,m2,qs) # Scoring function
    npprint('qs', qs)
    npprint('qs_residues', qs_residues)
    npprint('idx', idx)
    npprint('qs[idx]', qs[idx])

    idx = np.amin(qs_residues[:])
    # resMin, mIdx = min(abs(qs_residues))
    q = qs[idx]
    t, dep1, dep2, res = get_Txyz(m1,m2,q)
    # t = pose.T(:,mIdx);

    # make a fundamental matrix from the recovered rotation and translation
    q_arr = np.asarray([q.w,q.x,q.y,q.z], dtype=_dtype)
    R = sc_R.from_quat(q_arr).as_matrix()


    npprint('t',t)
    st()

    Tx = skew(t/np.norm(t))
    npprint('Tx',Tx)
    st()
    F = Tx @ R
    npprint('F',F)
    st()
    M = rmodel(q=q,t=t,m1=m1,m2=m2,F=F)
    return M


  def get_error(self, M:rmodel):
    ''' use Sampson distance to compute the first order approximation
    geometric error of the fitted fundamental matrix given a set of matched
    correspondences '''
    # todo working here
    npprint('M.m1', M.m1)
    npprint('M.m2', M.m2)
    npprint('M.F', M.F)
    st()
    if M.F.ndim > 3: # we have multiple essential matrices
      nF = M.F.shape[3]
      bestM = M
      bestM.F = M.F[1] # initial allocation of best solution
      num_inliers = 0 # number of inliers
      for k in range(len(nF)):
        m2tFm1 = np.zeros(1,M.m1.shape[1])
        for n in M.m1.shape[1]:
          m2tFm1[n] = M.m2[:,n].T @ M.F[k] @ M.m1[:,n]
        Fm1  = M.F[k]   @ M.m1
        Ftm2 = M.F[k].T @ M.m2

      # evaluate distances
      d =  m2tFm1 **2 /\
        (Fm1[0,:]**2 + Fm1[1,:]**2 + Ftm2[0,:]**2 + Ftm2[1,:]**2)
      inliers = np.where(abs(d) <  self.threshold)
      if len(inliers) > ninliers:  # record best solution
        ninliers = len(inliers)
        bestM.F = M.F[k]
        bestInlier_idxs = inliers
    else: # single solution
      m2tFm1 = np.zeros(1,len(M.m1))
      for n in range(len(M.m1)):
        m2tFm1[n] = M.m2[:,n].T @ M.F @ M.m1[:,n]
      Fm1  = M.F @ M.m1
      Ftm2 = M.F.T @ M.m2
      # evaluate distances
      d =  m2tFm1**2 /\
         (Fm1[0,:]**2 + Fm1[1,:]**2 + Ftm2[0,:]**2 + Ftm2[1,:]**2)
      bestInlier_idxs = np.where(abs(d) < self.threshold) # indices of inlying points
      bestM = M # copy M directly to bestM
    return bestInlier_idxs, d[bestInlier_idxs], bestM



  def random_partition(self,n,n_data):
    #todo try loading sparse random data points
    """return n random rows of data (and also the other len(data)-n rows)"""
    all_idxs = np.arange( n_data )
    np.random.shuffle(all_idxs)
    idxs = all_idxs[:n]
    return idxs




# EOF

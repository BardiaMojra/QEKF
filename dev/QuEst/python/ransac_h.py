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
      inliers_min - the number of close data values required to assert that a
      model fits well to data
    return:
      bestfit - model parameters which best fit the data """
  def __init__(self,m1,m2, quest, get_Txyz, max_iters,threshold,min_inliers,\
               nbug=True,return_all=True,_dtype=np.float128):
    self.m1 = m1  # inputs X
    self.m2 = m2  # outputs Y
    self.quest = quest
    self.get_Txyz = get_Txyz
    self.max_iters = max_iters
    self.threshold = threshold
    self.min_inliers = min_inliers
    self.NBUG = nbug
    self.return_all = return_all
    self.dtype = _dtype
    # end of __init__

  def get_best_fit(self):
    data = np.concatenate((self.m1,self.m2), axis=0).T
    best_fit = None
    best_err = np.inf
    best_inlier_idxs = None

    for i in range(self.max_iters):
      #todo explore ways to get points with SOME distance from each other
      #todo test enforcing min distance between correspondences
      maybe_idxs, else_idxs = self.random_partition(self.min_inliers,data.shape[0])
      maybeinliers = data[maybe_idxs,:]
      test_data = data[else_idxs,:]

      # npprint('maybe_idxs', maybe_idxs)
      # npprint('maybeinliers', maybeinliers)
      # npprint('else_data', else_data)

      maybemodel = self.fit(maybeinliers)
      errs, also_idxs, maybemodel = self.get_error(test_data, maybemodel)

      # npprint('test_data', test_data)
      # npprint('maybeinliers', maybeinliers)
      # npprint('also_idxs', also_idxs)

      alsoinliers = test_data[also_idxs,:]

      if self.NBUG:
        nprint('errs.min()',errs.min())
        nprint('errs.max()',errs.max())
        nprint('np.mean(errs)',np.mean(errs))
        nprint('iteration: ', i)
        nprint('len(alsoinliers): ', len(alsoinliers))

      # todo working here .........
      if len(alsoinliers) > self.min_inliers:
        npprint('maybeinliers', maybeinliers)
        npprint('alsoinliers', alsoinliers)

        st()

        bet_dat = np.concatenate((maybeinliers,alsoinliers),axis=0)#.copy()
        npprint('bet_dat', bet_dat)

        bet_mod = self.fit(bet_dat)
        bet_errs, bet_also_idxs, bet_mod = self.get_error(bet_dat,bet_mod)
        this_err = np.mean(bet_errs)

        npprint('bet_errs',bet_errs)
        npprint('bet_also_idxs',bet_also_idxs)
        nprint('bet_mod',bet_mod)
        npprint('this_err',this_err)
        st()


        if this_err < best_err:
          best_fit = bet_mod
          best_err = this_err
          best_inlier_idxs = np.concatenate((maybe_idxs,also_idxs),axis=0)#.copy()
          npprint('best_fit',best_fit)
          npprint('best_err',best_err)
          npprint('best_inlier_idxs',best_inlier_idxs)
        st()
      # end of for i in range(self.max_iters):
    st()
    if best_fit is None:
      raise ValueError("did not meet fit acceptance criteria")
    if self.return_all:
      return best_fit, {'inliers':best_inlier_idxs}
    else:
      return best_fit


  def fit(self,data):
    ''' pose estimation: estimates the relative rotation and translation
    between two camera views using the 5 point quaternion algorithm.
      input:
        data  - A structure
      output:
        M     - A structure that contains the estimated pose.
    Copyright (c) 2016, Kaveh Fathian. The University of Texas at Dallas. '''
    m1 = data[:,:3].T
    m2 = data[:,3:].T

    npprint('m1',m1)
    npprint('m2',m2)

    #todo working here
    st()
    qs = self.quest(m=m1, n=m2) # est rotation with QuEst
    qs_residues = get_residues(m1,m2,qs) # Scoring function
    idx = np.where(qs_residues==qs_residues.min())[0][0] # find best sol
    q = qs[idx,0]
    t, dep1, dep2, res = self.get_Txyz(m1,m2,q)
    # make a fundamental matrix from the recovered rotation and translation
    R = sc_R.from_quat(quat2arr(q)).as_matrix()
    Tx = skew(t/lalg.norm(t))
    F = Tx @ R # compute fundamental matrix
    return rmodel(q=q,t=t,m1=m1,m2=m2,F=F)

  def get_error(self,dat,M:rmodel):
    ''' use Sampson distance to compute the first order approximation
    geometric error of the fitted fundamental matrix given the set of ALL
    matched correspondences between two consecutive frames.
      input: M (rmodel) - maybe model
      output: all_errs '''
    m1 = dat[:,:3].T
    m2 = dat[:,3:].T
    npprint('m1',m1)
    npprint('m2',m2)
    if M.F.ndim > 3: # we have multiple essential matrices
      print(f'M.F.ndim = {M.F.ndim} and > 3')  # todo step through and confirm
      st()
      bestM = M
      bestM.F = M.F[0] # initial allocation of best solution
      ninliers = 0 # number of inliers
      for k in range(M.F.shape[2]):
        m2TFm1 = np.zeros((1,m1.shape[1]),dtype=self.dtype)
        for n in m1.shape[1]:
          st()
          m2TFm1[n] = m2[:,n].T @ M.F[k] @ m1[:,n]
        Fm1  = M.F[k]   @ m1
        Fm2 = M.F[k].T @ m2
      all_ds = self.get_SampsonDist(m2TFm1,Fm1,Fm2)
      inliers = np.asarray(all_ds<self.threshold).nonzero()
      if len(inliers) > ninliers:  # record best solution
        ninliers = len(inliers)
        bestM.F = M.F[k]
        bestInlier_idxs = inliers
    else: # single solution
      m2TFm1 = np.zeros((1,m1.shape[1]),dtype=self.dtype)
      for n in range(m1.shape[1]):
        m2TFm1[0,n] = m2[:,n].T @ M.F @ m1[:,n]
      Fm1  = M.F @ m1
      Fm2 = M.F.T @ m2
      all_ds = self.get_SampsonDist(m2TFm1,Fm1,Fm2)
      bestInlier_idxs = np.asarray(all_ds<self.threshold).nonzero()[0]
      bestM = M
      # nprint('all_ds',all_ds)
      # nprint('bestInlier_idxs',bestInlier_idxs)
    return all_ds, bestInlier_idxs, bestM

  def get_SampsonDist(self,m2TFm1:np.ndarray,Fm1:np.ndarray,Fm2:np.ndarray):
    d = (m2TFm1**2)/(Fm1[0,:]**2 + Fm1[1,:]**2 + Fm2[0,:]**2 + Fm2[1,:]**2)
    return np.asarray(np.abs(d[:][0]))

  def random_partition(self,n,n_data):
    #todo try loading sparse random data points
    """return n random rows of data (and also the other len(data)-n rows)"""
    all_idxs = np.arange( n_data )
    np.random.shuffle(all_idxs)
    rand_idxs = all_idxs[:n]
    else_idxs = all_idxs[n:]
    return rand_idxs, else_idxs




# EOF

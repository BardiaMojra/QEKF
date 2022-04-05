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
  def __init__(self,q,t,idxs,F):
    self.q        = q
    self.t        = t
    self.mod_idxs = idxs # indices of corresponding correspondences
    self.F        = F # fundamental matrix corresponding to mod_idxs
    self.errs     = None
    self.inl_idxs = None
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
  def __init__(self,m1,m2, quest, get_Txyz, max_iters,threshold,num_corresps,\
               nbug=True,return_all=True,_dtype=np.float128):
    self.m1 = m1  # inputs X
    self.m2 = m2  # outputs Y
    self.dat = np.concatenate((self.m1,self.m2), axis=0).T
    self.quest = quest # func pointer
    self.get_Txyz = get_Txyz # func pointer
    self.max_iters = max_iters
    self.threshold = threshold
    self.num_corresps = num_corresps
    self.NBUG = nbug
    self.return_all = return_all
    self.dtype = _dtype
    # end of __init__

  def get_best_fit(self):
    Best_M = None
    B_err = np.inf
    B_inl_idxs = None

    for i in range(self.max_iters):
      #todo explore ways to get points with SOME distance from each other
      #todo test enforcing min distance between correspondences


      mod_idxs, test_idxs = self.random_partition(self.num_corresps,self.dat.shape[0])
      mod = self.fit(mod_idxs)

      errs, inl_idxs, mod = self.get_error(test_idxs, mod)

      npprint('mod_idxs', mod_idxs)
      npprint('test_idxs', test_idxs)
      npprint('inl_idxs', inl_idxs)

      inl_dat = test_data[inl_idxs,:]




      if self.NBUG:
        print(lhead+f'iter:  {i}')
        print('errs.min(): {:0.10f}'.format(errs.min()))
        print('errs.max(): {:0.10f}'.format(errs.max()))
        print('np.mean(errs): {:0.10f}'.format(np.mean(errs)))
        print(f'len(inl_idxs): ', len(inl_idxs))
        print('\n\n')

      # todo working here .........
      if len(inl_idxs) > len(B_inl_idxs):
        # npprint('maybeinliers', maybeinliers)
        # npprint('alsoinliers', alsoinliers)

        # st()

        bet_dat = np.concatenate((mod_data,inl_dat),axis=0)#.copy()
        npprint('bet_dat', bet_dat)

        bet_mod = self.fit(bet_dat)
        bet_errs, bet_also_idxs, bet_mod = self.get_error(bet_dat,bet_mod)
        this_err = np.mean(bet_errs)

        npprint('bet_errs',bet_errs)
        npprint('bet_also_idxs',bet_also_idxs)
        nprint('bet_mod',bet_mod)
        npprint('this_err',this_err)
        st()


        if this_err < B_err:
          Best_M = bet_mod
          B_err = this_err
          B_inl_idxs = np.concatenate((mod_idxs,inl_idxs),axis=0)#.copy()
          npprint('best_fit',Best_M)
          npprint('best_err',B_err)
          npprint('best_inlier_idxs',B_inl_idxs)
        st()
      # end of for i in range(self.max_iters):
    st()
    if Best_M is None:
      raise ValueError("did not meet fit acceptance criteria")
    if self.return_all:
      return Best_M, {'inliers':B_inl_idxs}
    else:
      return Best_M

  def fit(self,mod_idxs):
    ''' pose estimation: estimates the relative rotation and translation
      between two camera views using the 5 point quaternion algorithm.
      input:
        mod_idxs  - selected indices
      output:
        mod       - model object '''
    m1 = self.dat[mod_idxs,:3].T
    m2 = self.dat[mod_idxs,3:].T

    npprint('m1',m1)
    npprint('m2',m2)
    st()

    qs = self.quest(m=m1, n=m2) # est rotation with QuEst
    qs_residues = get_residues(m1,m2,qs) # Scoring function
    idx = np.asarray(qs_residues==qs_residues.min()).nonzero()[0][0] # find best sol

    nprint('idx',idx)
    st()

    q = qs[idx,0]
    t, dep1, dep2, res = self.get_Txyz(m1,m2,q)
    # fundamental matrix from recovered rotation and translation
    R = sc_R.from_quat(quat2arr(q)).as_matrix()
    Tx = skew(t/lalg.norm(t))
    F = Tx @ R # compute fundamental matrix
    return rmodel(q=q,t=t,idxs=mod_idxs,F=F)

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
    all_idxs = np.arange(n_data)
    np.random.shuffle(all_idxs)
    rand_idxs = all_idxs[:n]
    else_idxs = all_idxs[n:]
    return rand_idxs, else_idxs




# EOF

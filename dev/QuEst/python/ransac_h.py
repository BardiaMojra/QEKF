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
  def __init__(self,q,t,idxs,F,qs,residues,q_idx):
    self.q        = q
    self.t        = t
    self.kp_idxs  = idxs # indices of corresponding correspondences
    self.F        = F # fundamental matrix corresponding to mod_idxs
    self.qs       = qs
    self.res      = residues
    self.qs_idx   = q_idx
    self.dists    = None
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
    B_mod = None
    # B_err = np.inf
    B_inl_idxs = list()

    for i in range(self.max_iters):
      #todo explore ways to get points with SOME distance from each other
      #todo test enforcing min distance between correspondences
      mod_idxs, test_idxs = self.random_partition(self.num_corresps,self.dat.shape[0])
      mod = self.fit(mod_idxs)
      dists, inl_idxs, mod = self.get_error(test_idxs, mod)
      npprint('mod_idxs', mod_idxs)
      npprint('test_idxs', test_idxs)
      npprint('inl_idxs', inl_idxs)
      # inl_dat = test_data[inl_idxs,:]
      if len(inl_idxs) > len(B_inl_idxs):
        print(lhead+'new model found...\n')
        # npprint('maybeinliers', maybeinliers)
        # npprint('alsoinliers', alsoinliers)
        # st()
        # bet_dat = np.concatenate((mod_data,inl_dat),axis=0)#.copy()
        # npprint('bet_dat', bet_dat)
        # bet_mod = self.fit(bet_dat)
        # bet_errs, bet_also_idxs, bet_mod = self.get_error(bet_dat,bet_mod)
        # this_err = np.mean(bet_errs)
        # npprint('bet_errs',bet_errs)
        # npprint('bet_also_idxs',bet_also_idxs)
        # nprint('bet_mod',bet_mod)
        # npprint('this_err',this_err)
        # st()
        # if this_err < B_err:
        B_mod = mod
        B_inl_idxs = inl_idxs
        # B_dists = dists
        # B_inl_idxs = np.concatenate((mod_idxs,inl_idxs),axis=0)#.copy()
        npprint('B_inl_idxs',B_inl_idxs)
        # npprint('B_dists',B_dists)
        # st()
      if self.NBUG:
        print(lhead+f'iter:  {i}')
        print('dists.min(): {:0.10f}'.format(dists.min()))
        print('dists.max(): {:0.10f}'.format(dists.max()))
        print('np.mean(dists): {:0.10f}'.format(np.mean(dists)))
        print(f'len(inl_idxs): ', len(inl_idxs))
        print('\n\n')
      # end of for i in range(self.max_iters):
    if B_mod is None:
      raise ValueError("did not meet fit acceptance criteria")
    if self.return_all:
      return B_mod,np.append(B_inl_idxs,B_mod.kp_idxs),B_mod.q,B_mod.t,B_mod.qs
    else:
      return B_mod.q,B_mod.t,B_mod.qs

  def fit(self,kp_idxs):
    ''' pose estimation: estimates the relative rotation and translation
      between two camera views using the 5 point quaternion algorithm.
      input:
        mod_idxs  - selected indices
      output:
        mod       - model object '''
    m1 = self.dat[kp_idxs,:3].T
    m2 = self.dat[kp_idxs,3:].T

    # npprint('m1',m1)
    # npprint('m2',m2)
    # st()

    qs = self.quest(m=m1, n=m2) # est rotation with QuEst
    qs_residues = get_residues(m1,m2,qs) # Scoring function
    idx = np.asarray(qs_residues==qs_residues.min()).nonzero()[0][0] # find best sol

    nprint('idx',idx)
    nprint('qs[idx,0]',qs[idx,0])
    nprint('res',qs_residues[idx])
    # st()

    q = qs[idx,0]
    t, dep1, dep2, res = self.get_Txyz(m1,m2,q)
    # fundamental matrix from recovered rotation and translation
    R = sc_R.from_quat(quat2arr(q)).as_matrix()
    Tx = skew(t/lalg.norm(t))
    F = Tx @ R # compute fundamental matrix
    return rmodel(q=q,t=t,idxs=kp_idxs,F=F,qs=qs,residues=qs_residues,q_idx=idx)

  def get_error(self,test_idx,mod:rmodel):
    ''' use Sampson distance to compute the first order approximation
    geometric error of the fitted fundamental matrix given the set of ALL
    matched correspondences between two consecutive frames.
      input: M (rmodel) - maybe model
      output: all_errs '''
    m1 = self.dat[test_idx,:3].T
    m2 = self.dat[test_idx,3:].T
    # npprint('m1',m1)
    # npprint('m2',m2)
    if mod.F.ndim > 3: # we have multiple essential matrices
      print(f'M.F.ndim = {mod.F.ndim} and > 3')  # todo step through and confirm
      st()
      # mod = mod
      # mod.F = mod.F[0] # initial allocation of best solution
      B_inl_idxs = list() # number of inliers
      B_dists = list()
      for k in range(mod.F.shape[2]):
        m2TFm1 = np.zeros((1,m1.shape[1]),dtype=self.dtype)
        for n in m1.shape[1]:
          st()
          m2TFm1[n] = m2[:,n].T @ mod.F[k] @ m1[:,n]
        Fm1 = mod.F[k]   @ m1
        Fm2 = mod.F[k].T @ m2
      dists = self.get_SampsonDist(m2TFm1,Fm1,Fm2)
      inliers = np.asarray(dists<self.threshold).nonzero()
      if len(inliers) > len(B_inl_idxs):  # record best solution
        B_inl_idxs = inliers
        B_dists = dists
        B_F = mod.F[k]
      mod.F = B_F
      mod.dists = B_dists
      mod.inl_idxs = B_inl_idxs
    else: # single solution
      m2TFm1 = np.zeros((1,m1.shape[1]),dtype=self.dtype)
      for n in range(m1.shape[1]):
        m2TFm1[0,n] = m2[:,n].T @ mod.F @ m1[:,n]
      Fm1  = mod.F @ m1
      Fm2 = mod.F.T @ m2
      mod.dists = self.get_SampsonDist(m2TFm1,Fm1,Fm2)
      mod.inl_idxs = np.asarray(mod.dists<self.threshold).nonzero()[0]
      # nprint('all_ds',all_ds)
      # nprint('bestInlier_idxs',bestInlier_idxs)
      # mod.dists = dists
      # mod.inl_idxs = inl_idxs
    return mod.dists, mod.inl_idxs, mod

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

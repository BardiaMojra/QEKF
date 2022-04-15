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
  def __init__(self, dat, quest, get_Txyz, max_iters,threshold,num_corresps,\
               nbug=True,return_all=True,_dtype=np.float128,lock=False):
    dat_ = np.absolute(dat) # keep it this way
    m1s = (dat_[:,:3].T).sum(axis=0)
    m2s = (dat_[:,3:].T).sum(axis=0)
    m1u = dat[:,:3]/m1s[:,None]
    m2u = dat[:,3:]/m2s[:,None]
    # npprint('m1u',m1u); npprint('m2u',m2u); st()
    ''' Data integrity: matches line 64 of QuEst_RANSAC_Ver1_2.m '''
    self.dat = np.concatenate((m1u,m2u), axis=1)
    self.quest = quest # func pointer
    self.get_Txyz = get_Txyz # func pointer
    self.max_iters = max_iters
    self.threshold = threshold # ransac inlier threshold
    self.num_corresps = num_corresps
    self.NBUG = nbug
    self.return_all = return_all
    self.dtype = _dtype
    self.lock = lock
    # end of __init__

  def get_best_fit(self):
    B_mod = None
    B_inl_idxs = list()
    for i in range(self.max_iters):
      #todo explore ways to get points with SOME distance from each other
      #todo test enforcing min distance between correspondences

      mod_idxs, test_idxs = self.random_partition(self.num_corresps,self.dat.shape[0])
      # npprint('mod_idxs',mod_idxs)
      # npprint('test_idxs',test_idxs)
      # st()


      #todo we here
      mod = self.fit(mod_idxs)
      dists, inl_idxs, mod = self.get_error(test_idxs, mod)

      # st()

      if len(inl_idxs) > len(B_inl_idxs):
        print(lhead+'new model found...\n')
        B_mod = mod
        B_inl_idxs = inl_idxs
      if self.NBUG and len(inl_idxs)>0:
        print(lhead+f'iter:  {i}')
        print('B_inl_idxs: ',B_inl_idxs)
        print('inl_idxs: ', inl_idxs)
        print('dists[inl_idx].min(): {:0.10f}'.format(dists[inl_idxs].min()))
        print('dists[inl_idx].max(): {:0.10f}'.format(dists[inl_idxs].max()))
        print('np.mean(dists): {:0.10f}'.format(np.mean(dists[inl_idxs])))
        print(f'len(inl_idxs): ', len(inl_idxs))
        print('\n\n')
        # st()
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
      input:    mod_idxs  - selected indices
      output:   mod       - model object '''
    # m1 = self.dat[:,:3].T
    # m2 = self.dat[:,3:].T
    # npprint('m1',m1)
    # npprint('m2',m2)
    # m1 = m1/np.sum(np.abs(m1), axis=0).reshape(1,-1)
    # m2 = m2/np.sum(np.abs(m2), axis=0).reshape(1,-1)
    # npprint('m1',m1)
    # npprint('self.dat',self.dat)
    # npprint('self.dat[kp_idxs,:]',self.dat[kp_idxs,:])

    # st()

    qs = self.quest(self.dat[kp_idxs,:]) # est rotation with QuEst
    qs_residues = get_residues(self.dat[kp_idxs,:],qs) # Scoring function
    write_np2txt(qs_residues,fname='nbug_QuEst_Qs_residues_python.txt')

    idx = np.asarray(qs_residues==qs_residues.min()).nonzero()[0][0] # find best sol

    #todo test with this for finding the best q solution
    # q, q_idx = get_closestQuat(qr,qs)

    nprint('idx',idx)
    nprint('qs[idx,0]',qs[idx,0])
    nprint('res',qs_residues[idx])
    # st()

    q = qs[idx,0]
    # write_np2txt(quaternion.as_float_array(q),fname='nbug_QuEst_Q_python.txt')

    t, dep1, dep2, res = self.get_Txyz(self.dat[kp_idxs,:],q) # est trans
    # write_np2txt(t,fname='nbug_QuEst_T_python.txt')

    R = sc_R.from_quat(quat2arr(q)).as_matrix() # compute rot mat
    Tx = skew(t/lalg.norm(t))
    F = Tx @ R # compute essential matrix
    # write_np2txt(F,fname='nbug_PseEst_F_python.txt')

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
    if mod.F.ndim < 3: # single solution
      m2TFm1 = np.zeros((1,m1.shape[1]),dtype=self.dtype)
      for n in range(m1.shape[1]):
        m2TFm1[0,n] = m2[:,n].T @ mod.F @ m1[:,n]
      Fm1  = mod.F @ m1
      Fm2 = mod.F.T @ m2
      mod.dists = self.get_SampsonDist(m2TFm1,Fm1,Fm2)
      mod.inl_idxs = np.asarray(mod.dists<self.threshold).nonzero()[0]
    else: # multiple essential matrices
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
      # end of else:
    return mod.dists, mod.inl_idxs, mod

  def get_SampsonDist(self,m2TFm1:np.ndarray,Fm1:np.ndarray,Fm2:np.ndarray):
    d = (m2TFm1**2)/(Fm1[0,:]**2 + Fm1[1,:]**2 + Fm2[0,:]**2 + Fm2[1,:]**2)
    return np.asarray(np.abs(d[:][0]))

  def random_partition(self,n,n_data):
    #todo try loading sparse random data points
    """return n random rows of data (and also the other len(data)-n rows)"""
    all_idxs = np.arange(n_data)
    if self.lock == False:
      np.random.shuffle(all_idxs)
    rand_idxs = all_idxs[:n]
    else_idxs = all_idxs[n:]
    return rand_idxs, else_idxs




# EOF

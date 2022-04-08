import numpy as np
# import matplotlib
# import pandas as pd
import cv2 as cv
import scipy as sc
from scipy.spatial.transform import Rotation as sc_R

''' private modules '''
from dmm import *
from nbug import *
from pdb import set_trace as st


''' matplotlib config '''
# matplotlib.pyplot.ion()
# plt.style.use('ggplot')


def skew(v):
  if len(v) == 4: v = v[:3]/v[3]
  skv = np.roll(np.roll(np.diag(v.flatten()),1,1),-1,0)
  return skv - skv.T


def quat2arr(q:np.quaternion,_dtype=np.float128):
  q = q.normalized()
  w = np.float128(q.w)
  x = np.float128(q.x)
  y = np.float128(q.y)
  z = np.float128(q.z)
  return np.asarray([w,x,y,z],dtype=_dtype) # make vec

def quat2np(q:np.quaternion,_dtype=np.float128):
  q = q.normalized()
  w = np.float128(q.w)
  x = np.float128(q.x)
  y = np.float128(q.y)
  z = np.float128(q.z)
  return np.asarray([w,x,y,z],dtype=_dtype).reshape(1,-1) # make row vec

def quats2np(qs:np.ndarray,axis=0,_dtype=np.float128):
  qs_np = list()
  for q in qs[:,0]:
    qs_np.append(quat2np(q,_dtype))
  if axis: qs_np = np.asarray(qs_np,_dtype).reshape(-1,4)
  else: qs_np = np.asarray(qs_np,_dtype).reshape(1,-1)
  return qs_np

def get_closestQuat(qr:quaternion, Qs, metric:str='phi03'):
  ''' compute quaternion error and rank using designated metric.
      more on the metrics could be found in [1].
      [1]: 'Metrics for 3D Rotations: Comparison and Analysis'
      '''
  if metric == 'pureQ':
    dists = get_qErr_simpDiff_np(qr, Qs) # compute error for quat solutions
    ranks = get_ranks_pureQuats(dists)
  elif metric == 'phi03' or metric == 'phi04':
    dists = get_qErr_phi0X_np(qr,Qs,metric)
    ranks = get_ranks_dists(dists)
  else:
    assert False, lhead+f'UNKNOWN method for calculating quat_err: '+metric+ltail

  #todo: make a df that gets passed in and returned and would be used to
  # concatenate other metric results
  # npprint('dists', dists)
  # nprint('ranks', ranks)
  # st()
  return Qs[ranks[0]][0], ranks[0]

def get_qErr_phi0X_np(qr:np.quaternion, Qs:np.ndarray, metric='phi03'):
  dists = np.ndarray(Qs.shape, dtype=np.float128)
  for i, q in enumerate(Qs):
    if metric == 'phi03':   dists[i] = phi03_dist(qr,q[0])
    elif metric == 'phi04': dists[i] = phi04_dist(qr,q[0])
    else: assert False, lhead+f'UNKNOWN metric for error evaluation: '+metric+ltail
  return dists

def get_ranks_dists(dists:np.ndarray):
  return sorted(range(dists.shape[0]), key=lambda i:dists[i])

def phi03_dist(qr:np.quaternion, q:np.quaternion,_dtype=np.float128):
  qr = quat2arr(qr)
  q = quat2arr(q)
  return (1/np.pi)*np.arccos(abs(qr @ q)) # divide by 1/pi to make range 0-1.

def phi04_dist(qr:np.quaternion, q:np.quaternion):
  ''' estimates phi03_dist by replacing arccos and estimating it '''
  qr = quat2np(qr)
  q = quat2np(q)
  return (1/np.pi)*(1 - abs(qr @ q)) # divide by 1/pi to make range 0-1.

def get_ranks_pureQuats(QErrs:np.ndarray):
  mags = QMags(QErrs)
  return sorted(range(QErrs.shape[0]), key=lambda i:mags[i])

def QMags(Qs):
  qMags = np.ndarray(Qs.shape, dtype=np.float128)
  for i, q in enumerate(Qs):
    qMags[i] =  quatMag(q[0])
  return qMags

def quatMag(q:np.quaternion):
  ''' get magnitude of a quaternion by first normalizing it (unit quat), then
  compute the magnitude of its "pure quat" form or "vectors" or "real parts" '''
  q = q.normalized() # todo: show to dr Gans and explain
  mag = np.sqrt(q.x**2+q.y**2+q.z**2)
  return mag

def get_qErr_simpDiff_np(q_ref:quaternion, Qs:np.ndarray):
  assert isinstance(q_ref,np.quaternion), shead+'q_ref is not a quaternion!'+ltail
  assert isinstance(Qs,np.ndarray), shead+'Qs is not a ndarray!'+ltail
  # should be already normalized, but normalize again before computing error
  q_ref = q_ref.normalized()
  Qs_err = np.ndarray(Qs.shape, dtype=np.quaternion)
  for i in range(Qs.shape[0]):
    Qs_err[i][0] = get_qDiff_simpQ(q_ref, Qs[i][0].normalized())
  return Qs_err

def get_qDiff_simpQ(qr:quaternion, q:quaternion):
  q_err = qr * q.inverse() # get quaternion error
  return q_err.normalized() # normalize the error once again

def get_qDiff_advQ(qr:np.quaternion, q:np.quaternion, _dtype=np.float128):
  r''' computes the quaternion representation of v1 using v0 as the origin.
  #todo: understand this.
  Dr. Gans:
  if c < (-1 + EPSILON):
  it seems this is a special case if the two quaternions are pointing in
  opposite directions, i.e. maximum rotation different.
  The rotations are maximally different, there is not a unique solution. In a
  sense, you could rotation about any axis by 180 degrees and get the maximal
  rotation difference. computationsally, the cross product will return a 0
  vector if they are pointing in opposite directions. so that special case
  assigns the axis in a different way. I don't totally understand what they do,
  but you can see it involves SVD, which always returns orthogonal vectors so it
  won't return a 0 vector like a cross product. '''
  npprint('v0',v0)
  v0 = quat2np(qr)
  v1 = quat2np(q)
  npprint('v0',v0)
  npprint('v1',v1)
  st()
  v0 = np.float128(v0)
  v1 = np.float128(v1)
  npprint('v0',v0)
  npprint('v1',v1)
  st()
  v0 = v0 / np.linalg.norm(v0)
  v1 = v1 / np.linalg.norm(v1)
  c = v0.dot(v1)
  EPSILON = np.finfo(_dtype).eps
  # Epsilon prevents issues at poles.
  if c < (-1 + EPSILON):
    c = max(c, -1)
    m = np.stack([v0, v1], 0)
    _, _, vh = np.linalg.svd(m, full_matrices=True)
    axis = vh.T[:, 2]
    w2 = (1 + c) * 0.5
    w = np.sqrt(w2)
    axis = axis * np.sqrt(1 - w2)
    return np.quaternion(w, *axis)
  axis = np.cross(v0, v1)
  s = np.sqrt((1 + c) * 2)
  return np.quaternion(s * 0.5, *(axis / s))

def get_TransError(tr:np.ndarray,t:np.ndarray):
  trn = tr/np.sqrt(sum(tr**2))
  tn = t/np.sqrt(sum(t**2))
  return (1/np.pi) * np.arccos(trn @ tn)

def prt_file_save(string, *args):
  print(shead+(string+' ')); print(*args);

def get_fignum_str(fignum):
  ''' usage: fignum+=1;get_fignum_str(fignum) '''
  return 'fig_%03i' % fignum

def show_image_w_kps(im,kps):
  imageKeys = cv.drawKeypoints(im,kps,None,(255,0,0),4)
  plt.imshow(imageKeys); plt.show(); cv.waitKey(0)
  return imageKeys

def show_image_w_mats(im_p,kp_p,im_n,kp_n,matches):
  imageKeys = cv.drawMatches(im_p,kp_p,im_n,kp_n,matches,None,flags=4)
  plt.imshow(imageKeys); plt.show(); cv.waitKey(0)
  return

def write_image(num:int, image, outDir):
  assert os.path.exists(outDir), lhead+'DO NOT EXIST: '+outDir+stail
  figname = get_fignum_str(num)
  figname = outDir+'_'+figname+'.png'
  cv.imwrite(figname, image)
  prt_file_save(shead+'saving figure: '+figname)
  return

class Dmatch_obj(object):
  def __init__(self,dat,K):
    p1 = dat[:,0:3]
    p2 = dat[:,3:6]

    m1 = np.absolute(sc.linalg.pinv(K) @ p1.T) #
    m2 = np.absolute(sc.linalg.pinv(K) @ p2.T)
    m1_rsum = m1.sum(axis=0)
    m2_rsum = m2.sum(axis=0)
    nprint('m1_rsum',m1_rsum)
    nprint('m2_rsum',m2_rsum)

    st()


    m1u = m1/m1_rsum[:,np.newaxis]
    m2u = m2/m2_rsum[:,np.newaxis]

    np.concatenate((m1u,m2u), axis=0).T


    self.p1 = p1
    self.p2 = p2
    self.m1 = m1
    self.m2 = m2
    self.m1u = m1u
    self.m2u = m2u
    self.numPoints = numPoints

  def prt(self):
    nprint('p1', self.p1)
    nprint('p2', self.p2)
    npprint('m1', self.m1)
    npprint('m2', self.m2)
    npprint('m1u', self.m1u)
    npprint('m2u', self.m2u)
    print('numPoints: ', self.numPoints)
    return
  # end of class Dmatch_obj(object): ------------------->> //

def GetFeaturePoints(alg,i:int,dat:dmm,threshold:int,minFeat=64,_show=False):
  f = dat.imgpath+dat.fnames[i]
  img = cv.imread(f, 0) # read image in gray scale (0)
  h, w = img.shape[:2] # not sure why
  image = img.copy()
  # plt.imshow(image); plt.show(); cv.waitKey(0); st(); plt.close()
  if dat.bench == 'TUM' or dat.bench == 'ICL' or dat.bench == 'NAIST':
    newcameramtx,roi = cv.getOptimalNewCameraMatrix(dat.K,dat.dist,(w,h),1,(w,h))
    image = cv.undistort(img, dat.K, dat.dist, None, newcameramtx)
    x, y, w, h = roi
    image = image[y:y+h, x:x+w]
  try:
    kps = alg.detect(image, None)
    kps = sorted(kps, key=lambda x: -x.response)[:minFeat]
    kps, dscs = alg.compute(image, kps)
    if dscs.shape[0] < minFeat:
      assert dscs.shape[0] >= minFeat,\
        lhead+'not enough features for image: {:02}'.format(i)+stail
      nprint('dscs.shape', dscs.shape)
      dscs = np.concatenate([dscs, np.zeros(minFeat - dscs.shape[0])], axis=0)
      nprint('dscs.shape', dscs.shape)
      # st()
    # imageKeys = cv.drawKeypoints(image, kps, None, (255,0,0), 4)
    # plt.imshow(imageKeys); plt.show(); cv.waitKey(); st(); plt.close()
    if _show: show_image_w_kps(image,kps)
  except cv.error as e:
    assert 0, 'Error: '+e
  return image, kps, dscs

def prep_matches(dat, matches, kp_p, kp_n, minPts=5, _dtype=np.float128):
  ''' convert cv.DMatch obj to np.ndarray and python obj '''
  matches = sorted(matches, key = lambda x:x.distance)
  matches = matches[:minPts]
  mat = list()
  for m in matches:
    # queryIdx = np.asarray(m.queryIdx, dtype=_dtype).copy().reshape(-1,1)
    # trainIdx = np.asarray(m.trainIdx, dtype=_dtype).copy().reshape(-1,1)
    p_p = kp_p[m.queryIdx].pt
    p_n = kp_n[m.trainIdx].pt
    p1 = np.asarray([p_p[0],p_p[1],1], dtype=_dtype).copy().reshape(-1,3)
    p2 = np.asarray([p_n[0],p_n[1],1], dtype=_dtype).copy().reshape(-1,3)
    mat.append(np.concatenate((p1,p2),axis=1)) # Idx not used
  mat = np.asarray(mat,dtype=_dtype).reshape(-1,8)
  mats = Dmatch_obj(mat,dat.K)
  # mats.prt() # keep
  # return mats, mats.dat # return data (not normalized)
  return mats, mats.ndat # return norm-data

def load_matlab_matches(i,_dtype=np.float128):
  path = '../matlab/quest_5p_ransac/out/KITTI/feature_matches/matches_dat_m1m2_'
  dpath = path+str(i).zfill(2)+'.txt'
  dat = np.loadtxt(dpath, delimiter=' ', dtype=_dtype)
  mats = Dmatch_obj(dat,dat.K)
  # mats.prt() # keep
  # return mats, mats.dat # return data (not normalized)
  return mats, mats.ndat # return norm-data


def match_features(fmatcher,des_p,des_n,QUEST_MAX_MKP,_show,Im_p,kp_p,Im_n,kp_n):
  matches = fmatcher.match(des_p, des_n)
  matches = sorted(matches, key = lambda x:x.distance)
  print(lhead+'found '+str(len(matches))+' matched correspondences...'+stail)
  matches = matches[:QUEST_MAX_MKP]
  if _show: show_image_w_mats(Im_p,kp_p,Im_n,kp_n,matches)
  return matches

def retKPs_pxl(matches:Dmatch_obj):
  kps1 = matches.p1[:,:2].astype(np.int64).copy()
  kps2 = matches.p2[:,:2].astype(np.int64).copy()
  npprint('kps1', kps1)
  npprint('kps2', kps2)
  st()
  return kps1, kps2

def RelativeGroundTruth(i, dset):
  q1 = dset.q0
  t1 = dset.t0
  bench = dset.bench
  Q_gt = dset.Q_gt
  T_gt = dset.T_gt
  if bench == 'KITTI' or  bench == 'ICL' or bench == 'NAIST':
    q2 = Q_gt[i]
    t2 = T_gt[i]
  elif bench == 'TUM':
    fname = dset.fnames[i]
    # ftime = str2double( fname(1:end-4) ); % Time at the current frame
    # [q2, t2] = InterpPoseVer1_1(ftime,dataset.times, Q_gt, T_gt); % Interpolate data to find the pose of the current camera frame
  else:
    assert False, 'unknown benchtype '+bench
  if bench == 'KITTI' or  bench == 'ICL' or  bench ==  'TUM':
    # relative rotation between two frames (^2R_1 : rotation of frame 1
    # given in frame 2)
    qr = np.conj(q2) * q1
    # relative trans. vec. in current coord. frame (^2t_21 : relative
    # translation given in frame 2)
    rot = quaternion.as_rotation_matrix(np.conj(q2))
    # nprint('rot', rot)
    tr = rot @ (t2 - t1)
    # npprint('t1', t1)
    # npprint('t2', t2)
    # npprint('tr', tr)
    # st()
  elif bench == 'NAIST':
    # In this dataset the absolute pose is given in the camera coordinate
    # frame, i.e., c^R_w, c^t_w.(This is opposite of what is claimed in
    # their website, unfortunately!)
    # Relative rotation between two frames (^2R_1 : rotation of frame 1
    # given in frame 2)
    # qr = QuatMult(q2,QuatConj(q1));
    # Relative trans. vec. in current coord. frame (^2t_21 : relative
    # translation given in frame 2)
    # tr = t2 - Q2R(q2)*Q2R(QuatConj(q1)) * t1;
    pass
  else:
    assert False, 'unknown benchtype '+bench
  # store the current pose for the next iteration
  dset.q0 = q2;
  dset.t0 = t2;
  return qr, tr

# EOF

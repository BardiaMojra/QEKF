import numpy as np
import matplotlib
import pandas as pd
import cv2 as cv
import scipy as sc

''' private modules '''
from dmm import *
from nbug import *
from pdb import set_trace as st

''' matplotlib config '''
matplotlib.pyplot.ion()
plt.style.use('ggplot')

def GetFeaturePoints(alg, i:int, dat:dmm, threshold:int, minFeat=64):
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
      st()
    # imageKeys = cv.drawKeypoints(image, kps, None, (255,0,0), 4)
    # plt.imshow(imageKeys); plt.show(); cv.waitKey(); st(); plt.close()
  except cv.error as e:
    assert 0, 'Error: '+e
  return image, kps, dscs

def prep_matches(dat, matches, kp_p, kp_n, minPts=5, _dtype=np.float64):
  matches = sorted(matches, key = lambda x:x.distance)
  matches = matches[:minPts]
  mat = list()
  for m in matches:
    queryIdx = np.asarray(m.queryIdx, dtype=_dtype).copy().reshape(-1,1)
    trainIdx = np.asarray(m.trainIdx, dtype=_dtype).copy().reshape(-1,1)
    p_p = kp_p[m.queryIdx].pt
    p_n = kp_n[m.trainIdx].pt
    p1 = np.asarray([p_p[0],p_p[1],1], dtype=_dtype).copy().reshape(-1,3)
    p2 = np.asarray([p_n[0],p_n[1],1], dtype=_dtype).copy().reshape(-1,3)
    mat.append(np.concatenate((p1,p2,queryIdx,trainIdx), axis=1))

  mat = np.asarray(mat,dtype=_dtype).reshape(-1,8)
  p1 = mat[:,0:3]
  p2 = mat[:,3:6]

  m1 = sc.linalg.pinv(dat.K) @ p1.T
  m2 = sc.linalg.pinv(dat.K) @ p2.T

  m1u = m1/np.sqrt(sum(m1**2))
  m2u = m2/np.sqrt(sum(m2**2))

  mats = Dmatch_obj(p1, p2, m1, m2, m1u, m2u, p1.shape[0])
  # mats.prt()
  return mats

# def MatchFeaturePoints(Ip, ppoints, In, npoints, dset, maxPts, alg='ORB'):
#   f1, vp1 = GetFeaturePoints(Ip,ppoints);
#   f2, vp2 = GetFeaturePoints(In,npoints);
#   indexPairs, mmetric = matchFeatures(f1, f2, 'MaxRatio',0.7, ...
#       'MatchThreshold',1, 'Unique',true);
#   matchedPoints1 = vp1(indexPairs(:,1));
#   matchedPoints2 = vp2(indexPairs(:,2));
#   p1 = matchedPoints1.Location;
#   p2 = matchedPoints2.Location;
#   %   figure; showMatchedFeatures(Ip,In,p1,p2);
#   % Feature points
#   numMatch = size(p1, 1);  % Number of matched feature points
#   numPts = min(numMatch, maxPts);
#   p1 = p1(1:numPts, :);
#   p2 = p2(1:numPts, :);
#   # point coordinates on image plane
#   m1 = dataset.K \ double( [p1 ones(numPts, 1)].' );
#   m2 = dataset.K \ double( [p2 ones(numPts, 1)].' );
#   # Unit norm coordinates
#   m1u = bsxfun(@rdivide, m1, sqrt(sum(m1.^2,1)));
#   m2u = bsxfun(@rdivide, m2, sqrt(sum(m2.^2,1)));
#   matches.p1 = p1;
#   matches.p2 = p2;
#   matches.m1 = m1;
#   matches.m2 = m2;
#   matches.m1u = m1u;
#   matches.m2u = m2u;
#   matches.numPts = numPts;
#   return matches

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
    tr = rot * (t2 - t1)
    # nprint('t1', t1)
    # nprint('t2', t2)
    # nprint('tr', tr)
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

def get_QuatError(q_ref:quaternion, Q2):
  # # metric for 3D rotation error in quaternions
  # # err = 1 - abs(q1.' * q2);
  # # normalize s.t. each column of Q has norm 1
  # Q2_sq = Q2 * Q2
  # QNrm = np.sqrt(sum(Q2_sq,1));
  # Q2 = bsxfun(@rdivide, Q2, QNrm);
  # # Normalize s.t. each column of Q has norm 1
  # QNrm = sqrt(sum(q1.^2,1));
  # q1 = bsxfun(@rdivide, q1, QNrm);
  # numSols = size(Q2,2);
  # err = zeros(1,numSols);
  # for i = 1 : numSols
  #     q2 = Q2(:,i);
  #     err(i) = (1/pi) * acos(min([abs(q1.' * q2),1]));
  # end
  q_ref = q_ref.normalized()
  Q2 = Q2.normalized()
  nprint('q1', q_ref)
  nprint('Q2', Q2)
  nppshape('Q2', Q2)
  st()

  lenQ2 = Q2.shape[0]
  err = list()
  for q2 in Q2:
    q_err = q_ref * q2.inverse()
    err.append(q_err)

  nprint('err', err)
  st()
  return np.asarray(err)

def get_TransError(t_ref, t2):

  nprint('t_ref', t_ref)
  nprint('t_ref**2', t_ref**2)

  st()
  trn = t_ref / np.sqrt(sum(t_ref**2))
  t2n = t2 / np.sqrt(sum(t2**2))
  dotErr = sum(trn * t2n)
  err = (1/np.pi) * np.arccos(dotErr)
  return err


def prt_file_save(string, *args):
  print(shead+(string+' ')); print(*args);


def get_fignum_str(fignum):
  ''' usage: fignum+=1;get_fignum_str(fignum) '''
  return 'fig_%03i' % fignum


def write_image(num:int, image, outDir):
  assert os.path.exists(outDir), lhead+'DO NOT EXIST: '+outDir+stail
  figname = get_fignum_str(num)
  figname = outDir+'_'+figname+'.png'
  cv.imwrite(figname, image)
  prt_file_save(shead+'saving figure: '+figname)
  return

class Dmatch_obj(object):
  def __init__(self, p1, p2, m1, m2, m1u, m2u, numPoints):
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
    nprint('m1', self.m1)
    nprint('m2', self.m2)
    nprint('m1u', self.m1u)
    nprint('m2u', self.m2u)
    nprint('numPoints', self.numPoints)
    return
# EOF

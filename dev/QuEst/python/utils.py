import numpy as np
import pandas as pd
import cv2 as cv

''' private modules '''
from dmm import *
from nbug import *
from pdb import set_trace as st

def GetFeaturePoints(alg, i:int, dat:dmm, threshold:int, minFeat=64):
  f = dat.imgpath+dat.fnames[i]
  img = cv.imread(f, 0) # read image in gray scale (0)
  h, w = img.shape[:2] # not sure why
  image = img.copy()
  plt.imshow(image); plt.show(); cv.waitKey(5); plt.close()
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
    imageKeys = cv.drawKeypoints(image, kps, None, (255,0,0), 4)
    plt.imshow(image); plt.show(); cv.waitKey(5); plt.close()
  except cv.error as e:
    assert 0, 'Error: '+e
  return image, kps, dscs


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
  qTru = dset.qTru
  tTru = dset.tTru
  if bench == 'KITTI' or  bench == 'ICL' or bench == 'NAIST':
    q2 = qTru[:,i]
    t2 = tTru[:,i]
  elif bench == 'TUM':
    fname = dset.fnames[i]
    # ftime = str2double( fname(1:end-4) ); % Time at the current frame
    # [q2, t2] = InterpPoseVer1_1(ftime,dataset.times, qTru, tTru); % Interpolate data to find the pose of the current camera frame
  else:
    assert False, 'unknown benchtype '+bench

  if bench == 'KITTI' or  bench == 'ICL' or  bench ==  'TUM':
    # relative rotation between two frames (^2R_1 : rotation of frame 1
    # given in frame 2)
    nprint('q1', q1)
    nprint('q2', q2)
    st()

    qr = np.conj(q2) * q1

    # relative trans. vec. in current coord. frame (^2t_21 : relative
    # translation given in frame 2)
    rot = np.conj(q2).as_rotation_matrix
    nprint('rot', rot)
    tr = rot * (t2 - t1)

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



# EOF

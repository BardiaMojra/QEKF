import numpy as np
import pandas as pd
import cv2 as cv


''' private modules '''
from dmm import *
from nbug import *
from pdb import set_trace as st

def GetFeaturePoints(alg, i:int, dat:dmm, threshold:int, vector_size=32):
  f = dat.imgpath+dat.fnames[i]
  img = cv.imread(f, 0) # read image in gray scale (0)
  h, w = img.shape[:2] # not sure why
  image = img.copy()
  plt.imshow(image)
  plt.show()
  cv.wait(5)

  if dat.bench == 'TUM' or dat.bench == 'ICL' or dat.bench == 'NAIST':
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(dat.K, dat.dist, (w,h), 1, (w,h))
    image = cv.undistort(img, dat.K, dat.dist, None, newcameramtx)
    x, y, w, h = roi
    image = image[y:y+h, x:x+w]

  try:
    kps = alg.detect(image, None)
    print(kps)
    kps = sorted(kps, key=lambda x: -x.response)[:vector_size]
    print(kps)
    st()

    kps, dscs = alg.compute(image, kps)
    nprint('kps',kps)
    nprint('dscs',dscs)
    st()


    dscs = dscs.flatten()

    needed_size = vector_size * 64
    if dscs.size < needed_size:
      dscs = np.concatenate([dscs, np.zeros(needed_size - dscs.size)])


    nprint('len(keypoints)', len(kps))
    imageKeys = cv.drawKeypoints(image, kps, None, (255,0,0), 4)
    plt.imshow(imageKeys)
    plt.show()

  except cv.error as e:
    assert 0, 'Error: '+e

  return image, kps, dscs


def MatchFeaturePoints(Ip, ppoints, In, npoints, dset, maxPts, alg='ORB'):


  f1, vp1 = GetFeaturePoints(Ip,ppoints);
  f2, vp2 = GetFeaturePoints(In,npoints);

  indexPairs, mmetric = matchFeatures(f1, f2, 'MaxRatio',0.7, ...
      'MatchThreshold',1, 'Unique',true);
  matchedPoints1 = vp1(indexPairs(:,1));
  matchedPoints2 = vp2(indexPairs(:,2));

  p1 = matchedPoints1.Location;
  p2 = matchedPoints2.Location;
  %   figure; showMatchedFeatures(Ip,In,p1,p2);

  % Feature points
  numMatch = size(p1, 1);  % Number of matched feature points
  numPts = min(numMatch, maxPts);
  p1 = p1(1:numPts, :);
  p2 = p2(1:numPts, :);

  # point coordinates on image plane
  m1 = dataset.K \ double( [p1 ones(numPts, 1)].' );
  m2 = dataset.K \ double( [p2 ones(numPts, 1)].' );

  # Unit norm coordinates
  m1u = bsxfun(@rdivide, m1, sqrt(sum(m1.^2,1)));
  m2u = bsxfun(@rdivide, m2, sqrt(sum(m2.^2,1)));

  matches.p1 = p1;
  matches.p2 = p2;
  matches.m1 = m1;
  matches.m2 = m2;
  matches.m1u = m1u;
  matches.m2u = m2u;
  matches.numPts = numPts;




  return matches

def get_relativeGroundTruth(i, posp, dset):


  return relPose, posp



# EOF

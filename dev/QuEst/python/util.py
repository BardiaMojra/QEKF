import numpy as np
import pandas as pd
import cv2 as cv


''' private modules '''
from dmm import *
from nbug import *
from pdb import set_trace as st

def GetFeaturePoints(i:int, dat:dmm, threshold:int):
  benchtype = dat.benchtype
  K = dat.K
  dist = dat.dist
  f = dat.imgpath+dat.fnames[i]
  img = cv.imread(f, 0) # read image in gray scale (0)
  h, w = img.shape[:2] # not sure why
  image = img.copy()
  plt.imshow(image)
  plt.show()

  if benchtype == 'TUM' or benchtype == 'ICL' or benchtype == 'NAIST':
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(K, dist, (w,h), 1, (w,h))
    image = cv.undistort(img, K, dist, None, newcameramtx)
    x, y, w, h = roi
    image = image[y:y+h, x:x+w]

  # create SURF object and set Hessian threshold
  surf = cv.xfeatures2d.SURF_create(threshold)
  #surf.setHessianThreshold(50000) # use this to set threshold on the fly
  keypoints, desciptors = surf.detectAndCompute(image, None)
  nprint('len(keypoints)', len(keypoints))
  imageKeys = cv.drawKeypoints(image, keypoints, None, (255,0,0), 4)
  plt.imshow(imageKeys)
  plt.show()

  return keypoints, image



# EOF

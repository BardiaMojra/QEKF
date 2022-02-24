import numpy as np
import pandas as pd
import opencv as cv


''' private modules '''
from dmm import *

from nbug import *
from pdb import set_trace as st


def GetFeaturePoints(i:int, dset:dmm, surfThresh:dmm.dtype):
  benchtype = dset.benchtype
  K = dset.K


  for i, fname in dset.fnames:
    f = dset.imgpath+fname
    img = cv.imread(f, 0) # read image in gray scale (0)
    h, w = img.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    if benchtype == 'TUM' or benchtype == 'ICL' or benchtype == 'NAIST':

      image = cv.undistort(img, )

      img = undistortImage(img, dataset.camParams);
  end


  points = detectSURFFeatures(img, 'MetricThreshold', surfThresh)

  return points, img



# EOF

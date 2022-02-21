import numpy as np
import pandas as pd


''' private modules '''
from dmm import *

from nbug import *
from pdb import set_trace as st


def GetFeaturePoints(i:int, dataset:dmm, surfThresh:np.float32):
  benchtype =

  return points, Im



#todo: convert matlab code
function [points, Im] = GetFeaturePoints(i, dataset, surfThresh)

benchtype = dataset.benchtype;

% Read image
Im = imread([dataset.imgpath '\' dataset.fnames{i}]);

if (strcmp(benchtype, 'TUM') || strcmp(benchtype, 'ICL'))|| strcmp(benchtype,'NAIST')
    Im = rgb2gray(Im);
    Im = undistortImage(Im, dataset.camParams);
end

% Surf feature points
points = detectSURFFeatures(Im, 'MetricThreshold', surfThresh);   % Detect feature points




# EOF

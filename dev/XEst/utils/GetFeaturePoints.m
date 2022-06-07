function [points, Im] = GetFeaturePoints(i, dset, surfThresh)
  btype = dset.benchtype;
  Im = imread([dset.imgpath '/' dset.fnames{i}]);
  if (strcmp(btype, 'TUM') || strcmp(btype, 'ICL'))|| strcmp(btype,'NAIST')
      Im = rgb2gray(Im);
      Im = undistortImage(Im, dset.camParams);
  end
  points = detectSURFFeatures(Im, 'MetricThreshold', surfThresh); 



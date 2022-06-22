%% XEst main 

%% init
close all; clear; clc;  
cfg   = config_class(TID  = 'T00001', desc = "test description.");
dat   = dat_class(); dat.load_cfg(cfg);
%trkr  = tracker_class(); tracker.load_cfg(cfg);


%% init
currFrameIdx = 1;
currI = readimage(dat.imds, currFrameIdx);
himage = imshow(currI);
rng(0); % Set random seed for reproducibility

% Create a cameraIntrinsics object to store the camera intrinsic parameters.
% The intrinsics for the dataset can be found at the following page:
% https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
% Note that the images in the dataset are already undistorted, hence there
% is no need to specify the distortion coefficients.
focLen      = [535.4, 539.2];    % in units of pixels
princPoint  = [320.1, 247.6];    % in units of pixels
imgSize     = size(currI,[1 2]);  % in units of pixels

intrinsics  = cameraIntrinsics(focLen, princPoint, imgSize);
%cam = camera_class();


%% init pfeat mod and get init frame features  
scaleFactor = 1.2;
numLevels   = 8;
numPoints   = 1000;
[preFeatures, prePoints] = helperDetectAndExtractFeatures(currI, ...
  scaleFactor, numLevels, numPoints); 
currFrameIdx = currFrameIdx + 1;
firstI       = currI; % Preserve the first frame 
isMapInitialized  = false;

% Map initialization loop
while ~isMapInitialized && currFrameIdx < numel(dat.imds.Files)
  currI = readimage(dat.imds, currFrameIdx);
  [currFeatures, currPoints] = helperDetectAndExtractFeatures(currI, ...
    scaleFactor, numLevels, numPoints);   
  currFrameIdx = currFrameIdx + 1;
  % Find putative feature matches
  indexPairs = matchFeatures(preFeatures, currFeatures, 'Unique', true, ...
    'MaxRatio', 0.9, 'MatchThreshold', 40);
  preMatchedPoints  = prePoints(indexPairs(:,1),:);
  currMatchedPoints = currPoints(indexPairs(:,2),:);
  % If not enough matches are found, check the next frame
  minMatches = 100;
  if size(indexPairs, 1) < minMatches
    continue
  end
  preMatchedPoints  = prePoints(indexPairs(:,1),:);
  currMatchedPoints = currPoints(indexPairs(:,2),:);
  % Compute homography and evaluate reconstruction
  [tformH, scoreH, inliersIdxH] = helperComputeHomography(preMatchedPoints, currMatchedPoints);
  % Compute fundamental matrix and evaluate reconstruction
  [tformF, scoreF, inliersIdxF] = helperComputeFundamentalMatrix(preMatchedPoints, currMatchedPoints);
  % Select the model based on a heuristic
  ratio = scoreH/(scoreH + scoreF);
  ratioThreshold = 0.45;
  if ratio > ratioThreshold
    inlierTformIdx = inliersIdxH;
    tform          = tformH;
  else
    inlierTformIdx = inliersIdxF;
    tform          = tformF;
  end

  % Computes the camera location up to scale. Use half of the 
  % points to reduce computation
  inlierPrePoints  = preMatchedPoints(inlierTformIdx);
  inlierCurrPoints = currMatchedPoints(inlierTformIdx);
  [relOrient, relLoc, validFraction] = relativeCameraPose(tform, intrinsics, ...
    inlierPrePoints(1:2:end), inlierCurrPoints(1:2:end));
  
  % If not enough inliers are found, move to the next frame
  if validFraction < 0.9 || numel(size(relOrient))==3
    continue
  end
  
  % Triangulate two views to obtain 3-D map points
  relPose = rigid3d(relOrient, relLoc);
  minParallax = 1; % In degrees
  [isValid, xyzWorldPoints, inlierTriangulationIdx] = helperTriangulateTwoFrames(...
    rigid3d, relPose, inlierPrePoints, inlierCurrPoints, intrinsics, minParallax);
  if ~isValid
    continue
  end
  % Get the original index of features in the two key frames
  indexPairs = indexPairs(inlierTformIdx(inlierTriangulationIdx),:);
  isMapInitialized = true;
  disp(['Map initialized with frame 1 and frame ', num2str(currFrameIdx-1)])
end % End of map initialization loop

if isMapInitialized
  close(himage.Parent.Parent); % Close the previous figure
  % Show matched features
  hfeature = showMatchedFeatures(firstI, currI, prePoints(indexPairs(:,1)), ...
    currPoints(indexPairs(:, 2)), 'Montage');
else
  error('Unable to initialize map.')
end
% Create an empty imageviewset object to store key frames
vSetKeyFrames = imageviewset;
% Create an empty worldpointset object to store 3-D map points
mapPointSet   = worldpointset;


%% store initial key frames and map points
% Create a helperViewDirectionAndDepth object to store view direction and depth 
directionAndDepth = helperViewDirectionAndDepth(size(xyzWorldPoints, 1));
% Add the first key frame. Place the camera associated with the first 
% key frame at the origin, oriented along the Z-axis
preViewId     = 1;
vSetKeyFrames = addView(vSetKeyFrames, preViewId, rigid3d, 'Points', prePoints,...
    'Features', preFeatures.Features);
% Add the second key frame
currViewId    = 2;
vSetKeyFrames = addView(vSetKeyFrames, currViewId, relPose, 'Points', currPoints,...
    'Features', currFeatures.Features);
% Add connection between the first and the second key frame
vSetKeyFrames = addConnection(vSetKeyFrames, preViewId, currViewId, relPose, 'Matches', indexPairs);
% Add 3-D map points
[mapPointSet, newPointIdx] = addWorldPoints(mapPointSet, xyzWorldPoints);
% Add observations of the map points
preLocations  = prePoints.Location;
currLocations = currPoints.Location;
preScales     = prePoints.Scale;
currScales    = currPoints.Scale;
% Add image points corresponding to the map points in the first key frame
mapPointSet   = addCorrespondences(mapPointSet, preViewId, newPointIdx, indexPairs(:,1));
% Add image points corresponding to the map points in the second key frame
mapPointSet   = addCorrespondences(mapPointSet, currViewId, newPointIdx, indexPairs(:,2));


%% initialize place recognition database 
% Load the bag of features data created offline
bofData         = load('bagOfFeaturesDataSLAM.mat');
% Initialize the place recognition database
loopDatabase    = invertedImageIndex(bofData.bof,"SaveFeatureLocations", false);
% Add features of the first two key frames to the database
addImageFeatures(loopDatabase, preFeatures, preViewId);
addImageFeatures(loopDatabase, currFeatures, currViewId);


%% refine and visualize the initial reconstruction 
% Run full bundle adjustment on the first two key frames
tracks       = findTracks(vSetKeyFrames);
cameraPoses  = poses(vSetKeyFrames);
[refinedPoints, refinedAbsPoses] = bundleAdjustment(xyzWorldPoints, tracks, ...
    cameraPoses, intrinsics, 'FixedViewIDs', 1, ...
    'PointsUndistorted', true, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-15, 'MaxIteration', 20, ...
    'Solver', 'preconditioned-conjugate-gradient');
% Scale the map and the camera pose using the median depth of map points
medianDepth   = median(vecnorm(refinedPoints.'));
refinedPoints = refinedPoints / medianDepth;
refinedAbsPoses.AbsolutePose(currViewId).Translation = ...
    refinedAbsPoses.AbsolutePose(currViewId).Translation / medianDepth;
relPose.Translation = relPose.Translation/medianDepth;
% Update key frames with the refined poses
vSetKeyFrames = updateView(vSetKeyFrames, refinedAbsPoses);
vSetKeyFrames = updateConnection(vSetKeyFrames, preViewId, currViewId, relPose);
% Update map points with the refined positions
mapPointSet   = updateWorldPoints(mapPointSet, newPointIdx, refinedPoints);
directionAndDepth = update(directionAndDepth, mapPointSet, ...
  vSetKeyFrames.Views, newPointIdx, true); % Update view direction and depth 
close(hfeature.Parent.Parent); % close prev fig
featurePlot   = helperVisualizeMatchedFeatures(currI, currPoints(indexPairs(:,2)));


%% reconstruction of init map 
% Visualize initial map points and camera trajectory
mapPlot       = helperVisualizeMotionAndStructure(vSetKeyFrames, mapPointSet);
showLegend(mapPlot);

%% init tracking 
currKeyFrameId   = currViewId; % viewID of currKF
lastKeyFrameId   = currViewId; % viewID of prevKF
lastKeyFrameIdx  = currFrameIdx - 1; % idx of prevKF
addedFramesIdx   = [1; lastKeyFrameIdx]; % idxs of all KFs so far
isLoopClosed     = false;

%% run main loop - on all frames 
% - get frame orb feats and match against prevKF's corresponding 3D map pts
% - est world cam pose
% - given cam world pose, project prevKF's map pts onto currFrame and radial  
% search for feature matches
% - with 3D-to-2D correspondances in currFrame, refine cam pos using a
% motion-only BA-Motion 
% - project local map pts onto currFrame to search for more corresps in
% radius and then use BA-Motion again 
% - Decide if currFrame should be marked as KF, if yes, then continue to local
% mapping step, if not KF then start tracking next frame. 
isLastFrameKeyFrame = true;
while ~isLoopClosed && currFrameIdx < numel(dat.imds.Files)  
  currI = readimage(dat.imds, currFrameIdx);
  [currFeatures, currPoints] = helperDetectAndExtractFeatures(currI, ...
    scaleFactor, numLevels, numPoints);
  % Track the last key frame
  % mapPointsIdx:   Indices of the map points observed in the current frame
  % featureIdx:     Indices of the corresponding feature points in the 
  %                 current frame
  [currPose, mapPointsIdx, featureIdx] = helperTrackLastKeyFrame(mapPointSet, ...
    vSetKeyFrames.Views, currFeatures, currPoints, lastKeyFrameId, ...
    intrinsics, scaleFactor);
  
  % Track the local map and check if the current frame is a key frame.
  % A frame is a key frame if both of the following conditions are satisfied:
  % 1. At least 20 frames have passed since the last key frame or the
  %    current frame tracks fewer than 100 map points.
  % 2. The map points tracked by the current frame are fewer than 90% of
  %    points tracked by the reference key frame.
  % Tracking performance is sensitive to the value of numPointsKeyFrame.  
  % If tracking is lost, try a larger value.
  % localKeyFrameIds:   ViewId of the connected key frames of the current frame
  numSkipFrames     = 20;
  numPointsKeyFrame = 100;
  [localKeyFrameIds, currPose, mapPointsIdx, featureIdx, isKeyFrame] = ...
      helperTrackLocalMap(mapPointSet, directionAndDepth, vSetKeyFrames, ...
      mapPointsIdx, featureIdx, currPose, currFeatures, currPoints, ...
      intrinsics, scaleFactor, numLevels, isLastFrameKeyFrame, ...
      lastKeyFrameIdx, currFrameIdx, numSkipFrames, numPointsKeyFrame);
  updatePlot(featurePlot, currI, currPoints(featureIdx)); % shw mat-feat
  if ~isKeyFrame
    currFrameIdx        = currFrameIdx + 1;
    isLastFrameKeyFrame = false;
    continue
  else
    isLastFrameKeyFrame = true;
  end  
  currKeyFrameId  = currKeyFrameId + 1;


  %% local mapping 
  % Add the new key frame 
  [mapPointSet, vSetKeyFrames] = helperAddNewKeyFrame(mapPointSet, vSetKeyFrames, ...
      currPose, currFeatures, currPoints, mapPointsIdx, featureIdx, localKeyFrameIds);
  % Remove outlier map points that are observed in fewer than 3 key frames
  [mapPointSet, directionAndDepth, mapPointsIdx] = helperCullRecentMapPoints(mapPointSet, ...
      directionAndDepth, mapPointsIdx, newPointIdx);
  % Create new map points by triangulation
  minNumMatches = 20;
  minParallax   = 3;
  [mapPointSet, vSetKeyFrames, newPointIdx] = helperCreateNewMapPoints(mapPointSet, vSetKeyFrames, ...
      currKeyFrameId, intrinsics, scaleFactor, minNumMatches, minParallax);
  % Update view direction and depth
  directionAndDepth = update(directionAndDepth, mapPointSet, vSetKeyFrames.Views, ...
    [mapPointsIdx; newPointIdx], true);
  % Local bundle adjustment
  [mapPointSet, directionAndDepth, vSetKeyFrames, newPointIdx] = helperLocalBundleAdjustment( ...
    mapPointSet, directionAndDepth, vSetKeyFrames, ...
    currKeyFrameId, intrinsics, newPointIdx); 
  % Visualize 3D world points and camera trajectory
  updatePlot(mapPlot, vSetKeyFrames, mapPointSet); 


  %% loop closure 
  % Check loop closure after some key frames have been created    
  if currKeyFrameId > 20        
    % Minimum number of feature matches of loop edges
    loopEdgeNumMatches = 50;
    % Detect possible loop closure key frame candidates
    [isDetected, validLoopCandidates] = helperCheckLoopClosure(vSetKeyFrames, currKeyFrameId, ...
      loopDatabase, currI, loopEdgeNumMatches);
    if isDetected 
      % Add loop closure connections
      [isLoopClosed, mapPointSet, vSetKeyFrames] = helperAddLoopConnections(...
        mapPointSet, vSetKeyFrames, validLoopCandidates, currKeyFrameId, ...
        currFeatures, loopEdgeNumMatches);
    end
  end
    
  % If no loop closure is detected, add current features into the database
  if ~isLoopClosed
    addImageFeatures(loopDatabase,  currFeatures, currKeyFrameId);
  end
    
  % Update IDs and indices
  lastKeyFrameId  = currKeyFrameId;
  lastKeyFrameIdx = currFrameIdx;
  addedFramesIdx  = [addedFramesIdx; currFrameIdx]; %#ok<AGROW>
  currFrameIdx    = currFrameIdx + 1;
end % End of main loop

%% post processing 
% run a similarity pose graph optimization on the essential graph to compensate for drift
% Optimize the poses
minNumMatches      = 30;
[vSetKeyFramesOptim, poseScales] = optimizePoses(vSetKeyFrames, ...
  minNumMatches, 'Tolerance', 1e-16);
% Update map points after optimizing the poses
mapPointSet = helperUpdateGlobalMap(mapPointSet, directionAndDepth, ...
    vSetKeyFrames, vSetKeyFramesOptim, poseScales);
updatePlot(mapPlot, vSetKeyFrames, mapPointSet);
% Plot the optimized camera trajectory
optimizedPoses  = poses(vSetKeyFramesOptim);
plotOptimizedTrajectory(mapPlot, optimizedPoses)
showLegend(mapPlot); % Update legend

%% results 
gTruthData = load('orbslamGroundTruth.mat'); % Load ground truth 
gTruth     = gTruthData.gTruth;
% Plot gtruth camera trajectory 
plotActualTrajectory(mapPlot, gTruth(addedFramesIdx), optimizedPoses);
showLegend(mapPlot);
% Evaluate tracking accuracy
helperEstimateTrajectoryError(gTruth(addedFramesIdx), optimizedPoses);


%% the end 
disp("end of process...");
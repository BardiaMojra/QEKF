%% ORB-SLAM main 
% @link https://www.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html


%% init
close all; clear; clc;  
cfg   = config_class(TID  = 'T00001', desc = "test description.");
dat   = dat_class(); dat.load_cfg(cfg);
cam   = camera_class(); cam.load_cfg(cfg);
tkr   = tracker_class(); tkr.load_cfg(cfg);
pft   = pfeature_class(); pft.load_cfg(cfg);
rng(0); % Set random seed for reproducibility

% init
pft.init_initFrame(tkr);

initMap(tkr, dat, pft, cfg, cam);
check_initMap(tkr);


% Create an empty imageviewset object to store key frames
vSetKeyFrames = imageviewset;
% Create an empty worldpointset object to store 3-D map points
mapPointSet   = worldpointset;




%% store initial key frames and map points
% Create a helperViewDirectionAndDepth object to store view direction and depth 
directionAndDepth = helperViewDirectionAndDepth(size(tkr.xyzWorldPoints, 1));
% Add the first key frame. Place the camera associated with the first 
% key frame at the origin, oriented along the Z-axis
preViewId     = 1;
vSetKeyFrames = addView(vSetKeyFrames, preViewId, rigid3d, 'Points', tkr.prevPts,...
    'Features', tkr.prevFts.Features);
% Add the second key frame
currViewId    = 2;
vSetKeyFrames = addView(vSetKeyFrames, currViewId, tkr.relPose, 'Points', tkr.currPts,...
    'Features', tkr.currFts.Features);
% Add connection between the first and the second key frame
vSetKeyFrames = addConnection(vSetKeyFrames, preViewId, currViewId, tkr.relPose, 'Matches', tkr.idxPairs);
% Add 3-D map points
[mapPointSet, newPointIdx] = addWorldPoints(mapPointSet, tkr.xyzWorldPoints);
% Add observations of the map points
preLocations  = tkr.prevPts.Location;
currLocations = tkr.currPts.Location;
preScales     = tkr.prevPts.Scale;
currScales    = tkr.currPts.Scale;
% Add image points corresponding to the map points in the first key frame
mapPointSet   = addCorrespondences(mapPointSet, preViewId, newPointIdx, tkr.idxPairs(:,1));
% Add image points corresponding to the map points in the second key frame
mapPointSet   = addCorrespondences(mapPointSet, currViewId, newPointIdx, tkr.idxPairs(:,2));


%% initialize place recognition database 
% Load the bag of features data created offline
bofData         = load('bagOfFeaturesDataSLAM.mat');
% Initialize the place recognition database
loopDatabase    = invertedImageIndex(bofData.bof,"SaveFeatureLocations", false);
% Add features of the first two key frames to the database
addImageFeatures(loopDatabase, tkr.prevFts, preViewId);
addImageFeatures(loopDatabase, tkr.currFts, currViewId);







%% refine and visualize the initial reconstruction 
% Run full bundle adjustment on the first two key frames
tkr.tracks       = findTracks(vSetKeyFrames);
cameraPoses  = poses(vSetKeyFrames);
[refinedPoints, refinedAbsPoses] = bundleAdjustment(tkr.xyzWorldPoints, tkr.tracks, ...
    cameraPoses, cam.ins, 'FixedViewIDs', 1, ...
    'PointsUndistorted', true, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-15, 'MaxIteration', 20, ...
    'Solver', 'preconditioned-conjugate-gradient');
% Scale the map and the camera pose using the median depth of map points
medianDepth   = median(vecnorm(refinedPoints.'));
refinedPoints = refinedPoints / medianDepth;
refinedAbsPoses.AbsolutePose(currViewId).Translation = ...
    refinedAbsPoses.AbsolutePose(currViewId).Translation / medianDepth;
tkr.relPose.Translation = tkr.relPose.Translation/medianDepth;
% Update key frames with the refined poses
vSetKeyFrames = updateView(vSetKeyFrames, refinedAbsPoses);
vSetKeyFrames = updateConnection(vSetKeyFrames, preViewId, currViewId, tkr.relPose);
% Update map points with the refined positions
mapPointSet   = updateWorldPoints(mapPointSet, newPointIdx, refinedPoints);
directionAndDepth = update(directionAndDepth, mapPointSet, ...
  vSetKeyFrames.Views, newPointIdx, true); % Update view direction and depth 
close(tkr.hfeature.Parent.Parent); % close prev fig
featurePlot   = helperVisualizeMatchedFeatures(tkr.currI, tkr.currPts(tkr.idxPairs(:,2)));


%% reconstruction of init map 
% Visualize initial map points and camera trajectory
mapPlot       = helperVisualizeMotionAndStructure(vSetKeyFrames, mapPointSet);
showLegend(mapPlot);

%% init tracking 
currKeyFrameId   = currViewId; % viewID of currKF
lastKeyFrameId   = currViewId; % viewID of prevKF
lastKeyFrameIdx  = tkr.idx_currFr - 1; % idx of prevKF
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
while ~isLoopClosed && tkr.idx_currFr < numel(dat.imds.Files)  
  tkr.currI = readimage(dat.imds, tkr.idx_currFr);
  [tkr.currFts, tkr.currPts] = helperDetectAndExtractFeatures(tkr.currI, ...
    pft.scaleFactor, pft.numLevels, pft.numPoints);
  % Track the last key frame
  % mapPointsIdx:   Indices of the map points observed in the current frame
  % featureIdx:     Indices of the corresponding feature points in the 
  %                 current frame
  [currPose, mapPointsIdx, featureIdx] = helperTrackLastKeyFrame(mapPointSet, ...
    vSetKeyFrames.Views, tkr.currFts, tkr.currPts, lastKeyFrameId, ...
    cam.ins, pft.scaleFactor);
  
  % Track the local map and check if the current frame is a key frame.
  % A frame is a key frame if both of the following conditions are satisfied:
  % 1. At least 20 frames have passed since the last key frame or the
  %    current frame tkr.tracks fewer than 100 map points.
  % 2. The map points tracked by the current frame are fewer than 90% of
  %    points tracked by the reference key frame.
  % Tracking performance is sensitive to the value of numPointsKeyFrame.  
  % If tracking is lost, try a larger value.
  % localKeyFrameIds:   ViewId of the connected key frames of the current frame
  numSkipFrames     = 20;
  numPointsKeyFrame = 100;
  [localKeyFrameIds, currPose, mapPointsIdx, featureIdx, isKeyFrame] = ...
      helperTrackLocalMap(mapPointSet, directionAndDepth, vSetKeyFrames, ...
      mapPointsIdx, featureIdx, currPose, tkr.currFts, tkr.currPts, ...
      cam.ins, pft.scaleFactor, pft.numLevels, isLastFrameKeyFrame, ...
      lastKeyFrameIdx, tkr.idx_currFr, numSkipFrames, numPointsKeyFrame);
  updatePlot(featurePlot, tkr.currI, tkr.currPts(featureIdx)); % shw mat-feat
  if ~isKeyFrame
    tkr.idx_currFr       = tkr.idx_currFr+ 1;
    isLastFrameKeyFrame = false;
    continue
  else
    isLastFrameKeyFrame = true;
  end  
  currKeyFrameId  = currKeyFrameId + 1;


  %% local mapping 
  % Add the new key frame 
  [mapPointSet, vSetKeyFrames] = helperAddNewKeyFrame(mapPointSet, vSetKeyFrames, ...
      currPose, tkr.currFts, tkr.currPts, mapPointsIdx, featureIdx, localKeyFrameIds);
  % Remove outlier map points that are observed in fewer than 3 key frames
  [mapPointSet, directionAndDepth, mapPointsIdx] = helperCullRecentMapPoints(mapPointSet, ...
      directionAndDepth, mapPointsIdx, newPointIdx);
  % Create new map points by triangulation
  minNumMatches = 20;
  minParallax   = 3;
  [mapPointSet, vSetKeyFrames, newPointIdx] = helperCreateNewMapPoints(mapPointSet, vSetKeyFrames, ...
      currKeyFrameId, cam.ins, pft.scaleFactor, minNumMatches, minParallax);
  % Update view direction and depth
  directionAndDepth = update(directionAndDepth, mapPointSet, vSetKeyFrames.Views, ...
    [mapPointsIdx; newPointIdx], true);
  % Local bundle adjustment
  [mapPointSet, directionAndDepth, vSetKeyFrames, newPointIdx] = helperLocalBundleAdjustment( ...
    mapPointSet, directionAndDepth, vSetKeyFrames, ...
    currKeyFrameId, cam.ins, newPointIdx); 
  % Visualize 3D world points and camera trajectory
  updatePlot(mapPlot, vSetKeyFrames, mapPointSet); 


  %% loop closure 
  % Check loop closure after some key frames have been created    
  if currKeyFrameId > 20        
    % Minimum number of feature matches of loop edges
    loopEdgeNumMatches = 50;
    % Detect possible loop closure key frame candidates
    [isDetected, validLoopCandidates] = helperCheckLoopClosure(vSetKeyFrames, currKeyFrameId, ...
      loopDatabase, tkr.currI, loopEdgeNumMatches);
    if isDetected 
      % Add loop closure connections
      [isLoopClosed, mapPointSet, vSetKeyFrames] = helperAddLoopConnections(...
        mapPointSet, vSetKeyFrames, validLoopCandidates, currKeyFrameId, ...
        tkr.currFts, loopEdgeNumMatches);
    end
  end
    
  % If no loop closure is detected, add current features into the database
  if ~isLoopClosed
    addImageFeatures(loopDatabase,  tkr.currFts, currKeyFrameId);
  end
    
  % Update IDs and indices
  lastKeyFrameId  = currKeyFrameId;
  lastKeyFrameIdx = tkr.idx_currFr;
  addedFramesIdx  = [addedFramesIdx; tkr.idx_currFr,]; %#ok<AGROW>
  tkr.idx_currFr   = tkr.idx_currFr+ 1;
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
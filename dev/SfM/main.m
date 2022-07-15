%% SfM main
%
% Structure from motion from multiple views
% @link https://www.mathworks.com/help/vision/ug/structure-from-motion-from-multiple-views.html
%
% System overview:
%   1. est cam pose using sparse matched feat pts across views
%     1.1. detect and match pt feats and track pts
%     1.2. est rel pos for curr frame ===>>
%     1.3. transform rel pos into world frame
%     1.4. store curr frame pose and pts
%     1.5. find inliners between prev and curr
%     1.6. find pt tracks across all frames so far
%     1.7. use triang-multiview funct to compute init 3D pt correspondances
%     1.8. use bundle adjustment to refine camera poses and 3D pts
%   2. refine reconst by iterating over the sequence of views again and
%   apply bundle adjustment
%
%
%% init
close all; clear; clc;
cfg   = config_class(TID  = 'T00001'); %, pos_alg = 'RQuEst');


%% init dataset
% Use |imageDatastore| to get a list of all image file names in a
% directory.
imageDir = fullfile(toolboxdir('vision'), 'visiondata', 'structureFromMotion');
%imageDir = '/home/smerx/DATA/TUM_RGBD/rgbd_dataset_freiburg3_long_office_household/rgb';
imds = imageDatastore(imageDir);
figure % Display images
montage(imds.Files, 'Size', [3, 2]);
title('Input Image Sequence');
% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
  I = readimage(imds, i);
  images{i} = im2gray(I);
end


%% init cam
data = load(fullfile(imageDir, 'cameraParams.mat'));
cam = data.cameraParams;


%% init firstView
%intrinsics = cam.Intrinsics;
I = undistortImage(images{1}, cam.Intrinsics);
% Detect features. Increasing 'NumOctaves' helps detect large-scale
% features in high-resolution images. Use an ROI to eliminate spurious
% features around the edges of the image.
border = 50;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];
prevPts   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
prevFeats = extractFeatures(I, prevPts, 'Upright', true);
% Create an empty imageviewset object to manage the data associated with each
% view.
vSet = imageviewset;
% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, rigid3d, 'Points', prevPts);


%% add rest of the views
% Go through the rest of the images. For each image
% 1. Match points between the previous and the current image.
% 2. Estimate the camera pose of the current view relative to the previous view.
% 3. Compute the camera pose of the current view in the global coordinate system relative to the first view.
% 4. Triangulate the initial 3-D world points.
% 5. Use bundle adjustment to refine all camera poses and the 3-D world points.
for i = 2:numel(images)

  % undistort
  I = undistortImage(images{i}, cam.Intrinsics);
  % get matched pt feats
  currPts   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
  currFeats = extractFeatures(I, currPts, 'Upright', true);
  idxPairs   = matchFeatures(prevFeats, currFeats, 'MaxRatio', .7, 'Unique',  true);
  mPts1 = prevPts(idxPairs(:, 1));
  mPts2 = currPts(idxPairs(:, 2));





  %% est rel pose <<-------------------------------------------------------------------------------------
  [relQ, relT, inIdx] = get_rPos(mPts1, mPts2, cam.Intrinsics, cfg.pos_alg);







  prevPose = poses(vSet, i-1).AbsolutePose;
  relPose  = rigid3d(relQ, relT);
  currPose = rigid3d(relPose.T * prevPose.T);
  vSet = addView(vSet, i, currPose, 'Points', currPts);

  vSet = addConnection(vSet, i-1, i, relPose, 'Matches', idxPairs(inIdx,:));




  % Find point tracks across all views.
  tracks = findTracks(vSet);
  % Get the table containing camera poses for all views.
  camPoses = poses(vSet);
  % Triangulate initial locations for the 3-D world points.
  xyzPoints = triangulateMultiview(tracks, camPoses, cam.Intrinsics);
  % Refine the 3-D world points and camera poses.
  [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
      tracks, camPoses, cam.Intrinsics, 'FixedViewId', 1, ...
      'PointsUndistorted', true);
  % Store the refined camera poses.
  vSet = updateView(vSet, camPoses);
  prevFeats = currFeats;
  prevPts   = currPts;
end


%% dips cam poses 3D
camPoses = poses(vSet);
refPoses_fig = figure();
plotCamera(camPoses, 'Size', 0.2);
hold on
goodIdx = (reprojectionErrors < 5); % Exclude noisy 3-D points.
xyzPoints = xyzPoints(goodIdx, :);
% Display the 3-D points.
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off
% Specify the viewing volume.
loc1 = camPoses.AbsolutePose(1).Translation;
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);
title('Refined Camera Poses');


%% compute dense reconstruction
% Read and undistort the first image
I = undistortImage(images{1}, cam.Intrinsics);
% Detect corners in the first image.
prevPts = detectMinEigenFeatures(I, 'MinQuality', 0.001);
% Create the point tracker object to track the points across views.
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);
% Initialize the point tracker.
prevPts = prevPts.Location;
initialize(tracker, prevPts, I);
% Store the dense points in the view set.
vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
vSet = updateView(vSet, 1, 'Points', prevPts);
% Track the points across all views.
for i = 2:numel(images)
  % Read and undistort the current image.
  I = undistortImage(images{i}, cam.Intrinsics);
  % Track the points.
  [currPts, validIdx] = step(tracker, I);
  % Clear the old matches between the points.
  if i < numel(images)
    vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
  end
  vSet = updateView(vSet, i, 'Points', currPts);
  % Store the point matches in the view set.
  matches = repmat((1:size(prevPts, 1))', [1, 2]);
  matches = matches(validIdx, :);
  vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
end
% Find point tracks across all views.
tracks = findTracks(vSet);
% Find point tracks across all views.
camPoses = poses(vSet);
% Triangulate initial locations for the 3-D world points.
xyzPoints = triangulateMultiview(tracks, camPoses, cam.Intrinsics);
% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
  xyzPoints, tracks, camPoses, cam.Intrinsics, 'FixedViewId', 1, ...
  'PointsUndistorted', true);


%% disp dense reconstruction
% Display the refined camera poses.
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on
% Exclude noisy 3-D world points.
goodIdx = (reprojectionErrors < 5);
% Display the dense 3-D world points.
pcshow(xyzPoints(goodIdx, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off
% Specify the viewing volume.
loc1 = camPoses.AbsolutePose(1).Translation;
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);
title('Dense Reconstruction');


%% the end
disp("end of process...");

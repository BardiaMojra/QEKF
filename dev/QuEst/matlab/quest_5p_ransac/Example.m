%% RANSAC Implementation of QuEst
%
% Run this script to see how the RANSAC implementation of the QuEst 
% algorithm can be used to estimate the relative pose between keyframes in 
% the TUM image dataset.
%
% TUM dataset: https://vision.in.tum.de/data/datasets/rgbd-dataset. 
%
% NOTE: You can increase the execution speed by compiling the .mex version  
%       of the QuEst algorithm.
%
%
% Copyright (C) 2018, by Kaveh Fathian.
% Email: kavehfathian@gmail.com
%
% This program is a free software: you can redistribute it and/or modify it
% under the terms of the GNU lesser General Public License, either version 
% 3, or any later version.
%
% This program is distributed in the hope that it will be useful, but 
% WITHOUT ANY WARRANTY. See the GNU Lesser General Public License for more 
% details <http://www.gnu.org/licenses/>.
%
%
%
%% Set path

warning('off','all'); % Ignore warnings for Matlab version compatibility

% Folders which contain the required helper functions
addpath('Helpers');
addpath(genpath('Algorithms'));
addpath(genpath('RANSAC'));

% Path to the dataset files
dataroot = [pwd '/Datasets'];

% Load dataset
benchtype = 'KITTI';
benchnum = 3;
[dataset, posp] = LoadDataset(dataroot, benchtype, benchnum);


%% Set parameters

% Algorithms to run
algorithms = {'QuEst_RANSAC_v0102'};
%algorithms = {'QuEst_v0708'};

% Keyframes
skipFrame = 0;         % This is the number of frames that are skiped between two key frames

ranThresh   = 1e-6;    % RANSAC outlier threshold
surfThresh  = 200;     % SURF feature point detection threshold

% Number of feature points
maxPts  = 30;   % Maximum number of feature points to use for pose estimation (lower value increases speed)
minPts  = 6;    % Minimum number of feature points required (6 to estimate a unique pose from RANSAC)


%% Preallocate

numImag     = length(dataset.fnames);       % Total number of images
keyFrames   = 2+skipFrame : 1+skipFrame : numImag; 
numKeyFrames = length(keyFrames);           % Number of key frames

% Preallocate pose estimation variables 
numMethods  = length(algorithms);               % Number of algorithms used in the comparison
rotErr      = NaN(numKeyFrames,numMethods);     % Rotation error for each method
tranErr     = NaN(numKeyFrames,numMethods);     % Translation error for each method
Q           = cell(numKeyFrames,numMethods);    % Recovered quaternions 
T           = cell(numKeyFrames,numMethods);    % Recovered translations


%% Recover Pose using RANSAC and compare with ground truth

% Initialize with the first image feature points
[ppoints, Ip] = GetFeaturePoints(1, dataset, surfThresh);

cntr = 0; % key frame counter
for i = keyFrames
    
    cntr = cntr + 1; % Counter
        
    % Feature points matching    
    [npoints, In] = GetFeaturePoints(i, dataset, surfThresh); % Get the next image feature points           
%     matches = MatchFeaturePoints(Ip,ppoints, In,npoints, maxPts, dataset, i); % Match feature points
    matches = LoadMatches(i,dataset.K); % Match feature points
%     m_dat = cat(2,matches.m1',matches.m2');
%     dispn(m_dat,6); % must match python dat
    % Relative ground truth 
    [relPose, posp] = RelativeGroundTruth(i, posp, dataset);
    
    % In case there are not enough matched points move to the next iteration
    % (This should be checked after 'RelativeGroundTruth')
    if (matches.numPts < minPts)         
        % Use current image for the next iteration
        Ip = In;
        ppoints = npoints;         
        disp('Not enough matched feature points. Frame skipped!')
        continue
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    % Recover pose and find error by comparing with the ground truth     
    for mthd = 1 : numMethods        
        if strcmp(algorithms{mthd}, 'QuEst_RANSAC_v0102') % RANSAC with QuEst algorithm
            [M, inliers] = QuEst_RANSAC_Ver1_2(matches.m1, matches.m2, ranThresh);   
            q = M.Q;
            tOut = M.t;
        elseif strcmp(algorithms{mthd}, 'QuEst_v0708') 
          kp1 = matches.m1(:,1:5);
          kp2 = matches.m2(:,1:5);
          q = QuEst_5Pt_Ver7_8(kp1, kp2, cntr);
          tOut = FindTransDepth_Ver1_0(kp1, kp2, q);
%             q = M.Q;
%             tOut = M.t;
        else
            error('Undefined algorithm.')
        end

%         % Find the closet quaternion and translation to the ground truth    
        [q, matchIdx] = FindClosetQVer2_2(relPose.qr, q);
        t = FindClosetTrans(relPose.tr, [tOut,-tOut]);        
        t = -tOut;
        
        % Calculate the estimation error
        rotErr(cntr,mthd)  = QuatError(relPose.qr, q);
        tranErr(cntr,mthd) = TransError(relPose.tr, t);    
    
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    
    % Display images with matched feature points    
    imshow(Ip);
    hold on;
    p1 = matches.p1;
    p2 = matches.p2;
    numPts = size(p1,2);
    plot(p1(:,1), p1(:,2), 'g+');
    plot(p2(:,1), p2(:,2), 'yo');    
    for j = 1 : numPts
        if any(j == inliers)
            plot([p1(j,1) p2(j,1)], [p1(j,2) p2(j,2)], 'g'); % Detected inlier
        else
            plot([p1(j,1) p2(j,1)], [p1(j,2) p2(j,2)], 'r'); % Detected outlier
        end
    end
    drawnow;    
    hold off;
    
    % Store the current image for the next iteration
    Ip = In;
    ppoints = npoints;                           
    % Print iteration number
    if mod(cntr,10) == 0
        display(['Iteration ' num2str(cntr) ' of ' num2str(numKeyFrames)]);
    end     

end



%% Display results

% Remove NaN entries (corresponding to skipped frames)
nanIdx = find( sum(isnan(rotErr),2) );
rotErr(nanIdx,:) = [];
tranErr(nanIdx,:) = [];
numKeyFrames = size(rotErr,1);

% Statistics of error for rotation and translation estimates
rotErrM = mean(rotErr,1);            % Mean
rotErrS = std(rotErr,1);             % Standard deviation
rotErrMd = median(rotErr,1);         % Median
rotErrQ1 = quantile(rotErr,0.25, 1); % 25% quartile
rotErrQ3 = quantile(rotErr,0.75, 1); % 75% quartile
tranErrM = mean(tranErr,1);
tranErrS = std(tranErr,1);
tranErrMd = median(tranErr,1);
tranErrQ1 = quantile(tranErr,0.25, 1);
tranErrQ3 = quantile(tranErr,0.75, 1);

% Table of results
RowNames = {'Rot err mean'; 'Rot err std'; 'Rot err median'; 'Rot err Q_1'; 'Rot err Q_3';...
    'Tran err mean'; 'Tran err std'; 'Tran err median'; 'Tran err Q_1'; 'Tran err Q_3'};
data = [rotErrM; rotErrS; rotErrMd; rotErrQ1; rotErrQ3; ...
    tranErrM; tranErrS; tranErrMd; tranErrQ1; tranErrQ3;];
table(data(:,1),'RowNames',RowNames, 'VariableNames', algorithms) 



%%

warning('on','all');


%% QuEst with RANSAC for benchmarking 
%  For benchmarking the the 5-point algorithms (including the 8-pt algorithm)
%
% Ver 1_5_1:
%       - Organized Ver 1_5
%
% (c) Kaveh Fathian, Juan-Pablo Ramirez-Paredes


% warning('off','all'); % Ignore warnings for Matlab version compatibility
addpath('Helpers')
addpath(genpath('.Algorithms'))
dataroot = [pwd '/Datasets'];
% Load dataset
benchtype = 'KITTI';
benchnum = 3;
[dataset, posp] = LoadDataset(dataroot, benchtype, benchnum);


% Algorithms to run
algorithms = {'EightPt'; 
              'Nister'; 
              'Kneip'; 
              'Kukelova'; 
              'Stewenius'; 
              'QuEst'};

% keyframes
if strcmp(benchtype,'KITTI')
    skipFrame = 1; 
elseif strcmp(benchtype,'NAIST')
    skipFrame = 1; 
elseif strcmp(benchtype,'ICL')
    skipFrame = 1; 
elseif strcmp(benchtype,'TUM')
    skipFrame = 1;     
end

numImag       = length(dataset.fnames);  % Total number of images
keyFrames     = [2+skipFrame : 1+skipFrame : numImag]; 
numKeyFrames  = length(keyFrames);  % Number of key frames

% Number of feature points
maxPts  = 30;   % Maximum number of feature points to use for pose estimation (lower value increases speed)
minPts = 8;     % Minimum number of feature points required (at least 8 for 8-pt algorithm)

% Preallocate pose estimation variables 
numMethods = length(algorithms);        % Number of algorithms used in the comparison
rotErr  = NaN(numKeyFrames,numMethods); % Rotation error for each method
tranErr = NaN(numKeyFrames,numMethods); % Translation error for each method
Q = cell(numKeyFrames,numMethods);      % Recovered quaternions 
T = cell(numKeyFrames,numMethods);      % Recovered translations


%% Recover Pose using different methods and compare with ground truth

% Initialize with the first image feature points
[ppoints, Ip] = GetFeaturePoints(1, dataset);

cntr = 0; % key frame counter
for i = keyFrames
    
    cntr = cntr + 1; % Counter
        
    % Feature points matching    
    [npoints, In] = GetFeaturePoints(i, dataset); % Get the next image feature points           
    matches = MatchFeaturePoints(Ip,ppoints, In,npoints, maxPts, dataset); % Match feature points
    
    % Relative ground truth 
    [relPose, posp] = RelativeGroundTruth(i, posp, dataset);
    
    % In case there are not enough inlier matches move to the next iteration
    % (This should be after 'RelativeGroundTruth')
    if (matches.numPts < minPts) || (matches.status~=0)        
        % Use current image for the next iteration
        Ip = In;
        ppoints = npoints;         
        disp('Not enough matched feature points. Frame skipped!')
        continue
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    % Recover pose and find error by comparing with the ground truth     
    for mthd = 1 : numMethods        
        
        if strcmp(algorithms{mthd}, 'EightPt') % Eight Point algorithm

            EOut = eightp_Ver2_0(matches.m1,matches.m2);
            [ROut, tOut] = TransformEssentialsVer2_0(EOut);          
            q = R2Q(ROut);

        elseif strcmp(algorithms{mthd}, 'Nister') % Five Point algorithm (Nister)

            EOut = opengv('fivept_nister',matches.m2u,matches.m1u);
            [ROut, tOut] = TransformEssentialsVer2_0(EOut);          
            q = R2Q(ROut);

        elseif strcmp(algorithms{mthd}, 'Li') % Five Point algorithm (Li + Hatley)

            EOut = Ematrix5pt_v2(matches.m2u(:,1:5),matches.m1u(:,1:5));
            [ROut, tOut] = TransformEssentialsVer2_0(EOut);
            q = R2Q(ROut);

        elseif strcmp(algorithms{mthd}, 'Kneip') % Five Point algorithm (Kneip)

            ROut = opengv('fivept_kneip',1:5,matches.m2u,matches.m1u);  
            if ~isempty(ROut) % If a solution is returned
                q = R2Q(ROut);
                [q, matchIdx] = FindClosetQVer2_2(relPose.qr, q); % Closest solution to ground truth
                rotErr(cntr,mthd)  = QuatError(relPose.qr, q); % Estimation error
            end
            % Kneip does not return a translation estimate, so continue to
            % the next iteration.
            continue;

        elseif strcmp(algorithms{mthd}, 'Kukelova') % Five Point algorithm (Polynomial eigenvalue)

            Eout = PolyEigWrapper(matches.m1,matches.m2);                
            [ROut, tOut] = TransformEssentialsVer2_0(EOut);          
            q = R2Q(ROut);

        elseif strcmp(algorithms{mthd}, 'Stewenius') % Five Point algorithm (Stewenius)

            EOut = opengv('fivept_stewenius',matches.m2u,matches.m1u); % Same results as fivep.m                
            [ROut, tOut] = TransformEssentialsVer2_0(EOut);          
            q = R2Q(ROut);

        elseif strcmp(algorithms{mthd}, 'QuEst') % Five Point algorithm (QuEst)

            sol = QuEst_Ver1_1(matches.m1,matches.m2);                
            q = sol.Q;
            tOut = sol.T;

        else
            error('Undefined algorithm.')
        end

        % Find the closet quaternion and translation to the ground truth    
        [q, matchIdx] = FindClosetQVer2_2(relPose.qr, q);
        t = FindClosetTrans(relPose.tr, [tOut(:,matchIdx),-tOut(:,matchIdx)]); 
        % Calculate the estimation error
        rotErr(cntr,mthd)  = QuatError(relPose.qr, q);
        tranErr(cntr,mthd) = TransError(relPose.tr, t);    
    
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    % Store the current image for the next iteration
    Ip = In;
    ppoints = npoints;                           
    % Print iteration number
    if mod(cntr,10) == 0
        disp(['Iteration ' num2str(cntr) ' of ' num2str(numKeyFrames)]);
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
table(data(:,1),data(:,2),data(:,3),data(:,4),data(:,5), data(:,6), ...
     'RowNames',RowNames, 'VariableNames', algorithms) 

% warning('off','all'); % Ignore warnings for Matlab version compatibility


% QuEst_RANSAC estimates the pose between two camera views using RANSAC and
% QuEst algorithm.
% Input:
%          x1  - 3xN set of mathced feature point coordinates 
%                (Euclidean coordinates).  
%          x2  - 3xN set of feature point coordinates points such that x1 
%                is matched with x2.
%          t   - The distance threshold between data point and the model
%                used to decide whether a point is an inlier or not. 
%                Note that point coordinates are normalised to that their
%                mean distance from the origin is sqrt(2).  The value of
%                t should be set relative to this, say in the range 
%                0.001 - 0.01  
% Note that it is assumed that the matching of x1 and x2 are putative and it
% is expected that a percentage of matches will be wrong.
% Output:
%          M       - A structure that contains the best estimated pose.
%          inliers - An array of indices of the elements of x1, x2 that were
%                    the inliers for the best pose.
% This code is based on RANSAC provided by Peter Kovesi. 
% Copyright (c) Kaveh Fathian,  October 2018.
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the software without restriction, subject to the following conditions:
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
% The Software is provided "as is", without warranty of any kind.
% ------------------------------------------------------------------
% Ver 1_1: RANSAC based on Fundamental matrix 
% Ver 1_2: x1, x2 are Euclidean coordinates
%
function [M, inliers] = relPos_QuEst_RANSAC_old(x1, x2, t, feedback)
  if ~all(size(x1)==size(x2))  % Check input
    error('Image dataset must have the same dimension.');
  end
  if nargin == 3
    feedback = 0;
  end
  % Normalize the points in l^1 norm
  x1n = sum(abs(x1),1);
  x2n = sum(abs(x2),1);
  x1 = bsxfun(@rdivide, x1,x1n);
  x2 = bsxfun(@rdivide, x2,x2n);
  s = 6;  % Number of points needed to uniquely fit a fundamental matrix. 
          % Note that only 5 points are needed to estimate the pose, but 
          % with 5 points the solution is not unique.
  fittingfn = @PoseEstimator;
  distfn    = @get_SampsonDist;
  degenfn   = @IsDegenerate;
  % x1 and x2 are 'stacked' to create a 6xN array for ransac
  [M, inliers] = ransac([x1; x2], fittingfn, distfn, degenfn, s, t, feedback);
  % Now do a final fit on the data points considered to be inliers
  % Mb = feval(fittingfn, [x1(:,inliers); x2(:,inliers)]);
end    
    
    
%% Distance function
% Function to evaluate the first order approximation of the geometric error
% (Sampson distance) of the fit of a fundamental matrix with respect to a
% set of matched points as needed by RANSAC.  See: Hartley and Zisserman,
% 'Multiple View Geometry in Computer Vision', page 270.
% Note that this code allows for F being a cell array of fundamental matrices of
% which we have to pick the best one. (A 7 point solution can return up to 3
% solutions)
function [bestInliers, bestM] = get_SampsonDist(M, x, t)
  x1 = x(1:3,:);    % Extract x1 and x2 from x
  x2 = x(4:6,:);
  F = M.F;
  if iscell(F)        % We have several solutions each of which must be tested
    nF = length(F); % Number of solutions to test
    bestM = M;
    bestM.F = F{1}; % Initial allocation of best solution
    ninliers = 0;   % Number of inliers
    length(x1)
    for k = 1:nF
      x2tFx1 = zeros(1,length(x1));
      for n = 1:length(x1)
        x2tFx1(n) = x2(:,n)'*F{k}*x1(:,n);
      end
      Fx1  = F{k}*x1;
      Ftx2 = F{k}'*x2;     
      % Evaluate distances
      d =  x2tFx1.^2 ./ (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);
      inliers = find(abs(d) < t);     % Indices of inlying points
      if length(inliers) > ninliers   % Record best solution
        ninliers = length(inliers);
        bestM.F = F{k};
        bestInliers = inliers;
      end
    end
  else     % We just have one solution
    x2tFx1 = zeros(1,length(x1));
    for n = 1:length(x1)
      x2tFx1(n) = x2(:,n)'*F*x1(:,n);
    end
    Fx1  = F*x1;
    Ftx2 = F'*x2;     
    % Evaluate distances
    d =  x2tFx1.^2 ./ ...
         (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);
    bestInliers = find(abs(d) < t);     % Indices of inlying points
    bestM = M;                          % Copy M directly to bestM
  end
end


%% Check degeneracy
% Function to determine if a set of matched points will result
% in a degeneracy in the calculation of a fundamental matrix as needed by
% RANSAC.  This function assumes this cannot happen.
function r = IsDegenerate(x)
  r = 0;    
end    


%% Pose Estimation
% Estimates the relative rotation and translation between two camera views
% using the 5 point quatenion algoirithm.
%
%
% Arguments:
%          x1, x2 - Two sets of N matched feature point coordinates. x1, x2  
%                   are 3xN matrices with each column corresponding to a point.
%         
%          x      - If a single argument is supplied it is assumed that it
%                   is in the form x = [x1; x2]
% Output:
%          M      - A structure that contains the estimated pose.
%
%
% Copyright (c) 2016, Kaveh Fathian.
% The University of Texas at Dallas.
% 
%
function M = PoseEstimator(varargin)
  [x1, x2, npts] = checkargs(varargin(:));
  % Recover the pose
  pose = QuEst_Ver1_1(x1(:,1:5), x2(:,1:5));  % QuEst algorithm
  % Pick the best pose solution
  res = QuatResidueVer3_1(x1, x2, pose.Q); % Scoring function
  [resMin,mIdx] = min(abs(res)); 
  q = pose.Q(:,mIdx); 
  t = pose.T(:,mIdx);
  % Make a fundamental matrix from the recovered rotation and translation
  R = Q2R(q);
  Tx = Skew(t/norm(t));    
  F = Tx * R; 
  %path = '../../mout/nbug_PoseEst_F_matlab.txt';
  %writematrix(F,path,'Delimiter',' ');
  M.Q  = q;
  M.t  = t;
  M.m1 = x1;
  M.m2 = x2;
  M.F  = F;
end
   
  
%% Function to check argument values and set defaults
%
function [x1, x2, npts] = checkargs(arg)
  if length(arg) == 2
    x1 = arg{1};
    x2 = arg{2};
    if ~all(size(x1)==size(x2))
        error('Image dataset must have the same size.');
    elseif size(x1,1) ~= 3
        error('Image cordinates must come in a 3xN matrix.');
    end
  elseif length(arg) == 1
    if size(arg{1},1) ~= 6
      error('Single input argument must be 6xN');
    else
      x1 = arg{1}(1:3,:);
      x2 = arg{1}(4:6,:);
    end
  else
    error('Wrong number of arguments supplied');
  end
  npts = size(x1,2);
  if npts < 6
    error('At least 6 points are needed to compute the fundamental matrix');
  end
end


function [M, inliers] = ransac(x, fittingfn, distfn, degenfn, s, t, feedback, ...
                               maxDataTrials, maxTrials)
  % Test number of parameters
  narginchk(6, 9) ;
  if nargin < 9; maxTrials = 1000;    end;
  if nargin < 8; maxDataTrials = 100; end;
  if nargin < 7; feedback = 0;        end;
  [rows, npts] = size(x);
  p = 0.99;         % Desired probability of choosing at least one sample
                    % free from outliers (probably should be a parameter)
  bestM = NaN;      % Sentinel value allowing detection of solution failure.
  trialcount = 0;
  bestscore =  0;
  N = 1;            % Dummy initialisation for number of trials.
  while N > trialcount
      % Select at random s datapoints to form a trial model, M.
      % In selecting these points we have to check that they are not in
      % a degenerate configuration.
      degenerate = 1;
      count = 1;
      while degenerate
          % Generate s random indicies in the range 1..npts
          % (If you do not have the statistics toolbox with randsample(),
          % use the function RANDOMSAMPLE from my webpage)
          if ~exist('randsample', 'file')
              ind = randomsample(npts, s);
          else
              ind = randsample(npts, s);
          end
          
          %ind = [1,2,3,4,5,6]; % nbug 
  
          % Test that these points are not a degenerate configuration.
          degenerate = feval(degenfn, x(:,ind));
  
          if ~degenerate
              % Fit model to this random selection of data points.
              % Note that M may represent a set of models that fit the data in
              % this case M will be a cell array of models
              M = feval(fittingfn, x(:,ind));
  
              % Depending on your problem it might be that the only way you
              % can determine whether a data set is degenerate or not is to
              % try to fit a model and see if it succeeds.  If it fails we
              % reset degenerate to true.
              if isempty(M.F)
                  degenerate = 1;
              end
          end
  
          % Safeguard against being stuck in this loop forever
          count = count + 1;
          if count > maxDataTrials
              warning('Unable to select a nondegenerate data set');
              break
          end
      end
  
      % Once we are out here we should have some kind of model...
      % Evaluate distances between points and model returning the indices
      % of elements in x that are inliers.  Additionally, if M is a cell
      % array of possible models 'distfn' will return the model that has
      % the most inliers.  After this call M will be a non-cell object
      % representing only one model.
      [inliers, M] = feval(distfn, M, x, t);
      dispn(inliers, 6)
      % Find the number of inliers to this model.
      ninliers = length(inliers);
      if ninliers > bestscore    % Largest set of inliers so far...
        bestscore = ninliers;  % Record data for this model
        bestinliers = inliers;
        bestM = M;
        % Update estimate of N, the number of trials to ensure we pick,
        % with probability p, a data set with no outliers.
        fracinliers =  ninliers/npts;
        pNoOutliers = 1 -  fracinliers^s;
        pNoOutliers = max(eps, pNoOutliers);  % Avoid division by -Inf
        pNoOutliers = min(1-eps, pNoOutliers);% Avoid division by 0.
        N = log(1-p)/log(pNoOutliers);
      end
      trialcount = trialcount+1;
      if feedback
        fprintf('trial %d out of %d         \r',trialcount, ceil(N));
      end
      % Safeguard against being stuck in this loop forever
      if trialcount > maxTrials
        warning( ...
        sprintf('ransac reached the maximum number of %d trials',...
                maxTrials));
        break
      end
  end  
  if feedback, fprintf('\n'); end
  if isstruct(bestM)   % We got a solution
    M = bestM;
    inliers = bestinliers;
  else
    M = [];
    inliers = [];
    error('ransac was unable to find a useful solution');
  end
end






































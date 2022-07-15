function [isFound, bestE, inliers, reachedMaxSkipTrials] = msac_def(...
    allPoints, params, funcs, varargin)
% MSAC M-estimator SAmple Consensus (MSAC) algorithm that is used for point
% cloud model fitting. allPoints must be an M-by-N matrix, where each point
% is a row vector.
%
% allPoints - M-by-2 or M-by-2-by-2 array of [x y] coordinates
%
% params    - struct containing the following fields:
%               sampleSize
%               maxDistance
%               confidence
%               maxNumTrials
%
% funcs     - struct containing the following function handles
%               fitFunc
%               evalFunc
%               checkFunc

% Copyright 2015-2020 The MathWorks, Inc.
%
% References:
% ----------
%   P. H. S. Torr and A. Zisserman, "MLESAC: A New Robust Estimator with
%   Application to Estimating Image Geometry," Computer Vision and Image
%   Understanding, 2000.

%#codegen

confidence = params.confidence;
sampleSize = params.sampleSize;
maxDistance = params.maxDistance;

threshold = cast(maxDistance, 'like', allPoints);
numPts    = size(allPoints,1);
idxTrial  = 1;
numTrials = int32(params.maxNumTrials);
maxDis    = cast(threshold * numPts, 'like', allPoints);
bestDis   = maxDis;

if isfield(params, 'defaultModel')
    bestE = params.defaultModel;
else
    bestE = zeros(0, 'like', allPoints);
end

if isfield(params, 'maxSkipTrials')
    maxSkipTrials = params.maxSkipTrials;
else
    maxSkipTrials = params.maxNumTrials * 10;
end
skipTrials = 0;
reachedMaxSkipTrials = false;

bestInliers = false(numPts, 1);

% Create a random stream. It uses a fixed seed for the testing mode and a
% random seed for other mode.
coder.extrinsic('vision.internal.testEstimateGeometricTransform');
if isempty(coder.target) && vision.internal.testEstimateGeometricTransform
    rng('default');
end


while idxTrial <= numTrials && skipTrials < maxSkipTrials
    % Random selection without replacement
    %indices = randperm(numPts, sampleSize);
    indices = [1, 2, 3, 4, 5]; % NBUG
    % Compute a model from samples
    samplePoints = allPoints(indices, :, :);
    MPars = funcs.fitFunc(samplePoints, varargin{:});
    fprintf("indices: \n");
    fprintf("%2d ", indices);
    fprintf("\n");
    for i=1:size(MPars,3)
      fprintf("Es[%2d]:\n", i);
      disp(MPars{i});
    end
    % Validate the model
    isValidModel = funcs.checkFunc(MPars, varargin{:});
    
    if isValidModel
        % Evaluate model with truncated loss
        [model, dis, accDis] = evaluateModel(funcs.evalFunc, MPars, ...
            allPoints, threshold, varargin{:});
        
        % Update the best model found so far
        if accDis < bestDis
            bestDis = accDis;
            bestInliers = dis < threshold;
            bestE = model;
            inlierNum = cast(sum(dis < threshold), 'like', allPoints);
            num = vision.internal.ransac.computeLoopNumber(sampleSize, ...
                confidence, numPts, inlierNum);
            numTrials = min(numTrials, num);
        end
        
        idxTrial = idxTrial + 1;
    else
        skipTrials = skipTrials + 1;
    end
end

isFound = funcs.checkFunc(bestE(:), varargin{:}) && ...
    ~isempty(bestInliers) && sum(bestInliers(:)) >= sampleSize;
if isFound
    if isfield(params, 'recomputeModelFromInliers') && ...
            params.recomputeModelFromInliers
        MPars = funcs.fitFunc(allPoints(bestInliers, :, :), varargin{:});
        [bestE, dis] = evaluateModel(funcs.evalFunc, MPars, ...
            allPoints, threshold, varargin{:});
        isValidModel = funcs.checkFunc(bestE(:), varargin{:});
        inliers = (dis < threshold);
        if ~isValidModel || ~any(inliers)
            isFound = false;
            inliers = false(size(allPoints, 1), 1);
            return;
        end
    else
        inliers = bestInliers;
    end
    
    if numTrials >= int32(params.maxNumTrials)
        coder.internal.warning('vision:ransac:maxTrialsReached');
    end
else
    inliers = false(size(allPoints, 1), 1);
end

reachedMaxSkipTrials = skipTrials >= maxSkipTrials;

%--------------------------------------------------------------------------
function [modelOut, distances, sumDistances] = evaluateModel(evalFunc, modelIn, ...
    allPoints, threshold, varargin)
dis = evalFunc(modelIn, allPoints, varargin{:});
dis(dis > threshold) = threshold;
accDis = sum(dis);
if iscell(modelIn)
    [sumDistances, minIdx] = min(accDis);
    distances = dis(:, minIdx);
    modelOut = modelIn{minIdx(1)};
else
    distances = dis;
    modelOut = modelIn;
    sumDistances = accDis;
end



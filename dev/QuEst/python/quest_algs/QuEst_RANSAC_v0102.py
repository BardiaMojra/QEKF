# import cv2 as cv

# from sklearn.base import RegressorMixin
# from sklearn.base import TransformerMixin
# from sklearn.cluster import KMeans
# from sklearn.preprocessing import oneHotEncoder
# from sklearn.preprocessing import StandardScaler
# from sklearn.linear_model import LogisticRegression

# ''' private modules '''
# from nbug import *
# from pdb import set_trace as st

# # Pipeline([
# #   ('sc', StandardScaler()),
# #   ('km', KMeansSomehow()),
# #   ('lr', LogisticRegression())
# # ])




# class QuEst_RANSAC(RegressorMixin):
#   def fit(self, X, y, RANSAC_threshold):
#     ''' QuEst_RANSAC estimates the pose between two camera views using RANSAC
#     and QuEst algorithm.
#     Input:
#           x1  - 3xN set of matched feature point coordinates
#                 (Euclidean coordinates).
#           x2  - 3xN set of feature point coordinates points such that x1
#                 is matched with x2.
#           t   - The distance threshold between data point and the model
#                 used to decide whether a point is an inlier or not.
#                 Note that point coordinates are normalised to that their
#                 mean distance from the origin is sqrt(2).  The value of
#                 t should be set relative to this, say in the range
#                 0.001 - 0.01
#     Note that it is assumed that the matching of x1 and x2 are putative and it
#     is expected that a percentage of matches will be wrong.
#     Output:
#           M       - A structure that contains the best estimated pose.
#           inliers - An array of indices of the elements of x1, x2 that were
#                     the inliers for the best pose.
#     This code is based on RANSAC provided by Peter Kovesi. Copyright (c)
#     Kaveh Fathian,  October 2018. Permission is hereby granted, free of
#     charge, to any person obtaining a copy of this software and associated
#     documentation files (the "Software"), to deal in the software without
#     restriction, subject to the following conditions: The above copyright
#     notice and this permission notice shall be included in all copies or
#     substantial portions of the Software. The software is provided "as is",
#     without warranty of any kind.
#     ------------------------------------------------------------------
#     Ver 1_1: RANSAC based on Fundamental matrix
#     Ver 1_2: x1, x2 are Euclidean coordinates '''
#     assert x1.shape == x2.shape, lhead+'ERR: images have diff dimensions'+stail
#     # normalize the points in l^1 norm
#     x1 = x1 / sum(abs(x1))
#     x2 = x2 / sum(abs(x2))
#     s = 6; # Number of points needed to uniquely fit a fundamental matrix.
#            # Note that only 5 points are needed to estimate the pose, but
#            # with 5 points the solution is not unique.


#   # ptrs = {'fittingfn': PoseEstimator,
#           #  'distfn': FundDist,
#           #  'degenfn': IsDegenerate}
#   # x1 and x2 are 'stacked' to create a 6xN array for ransac

#     x = np.concatenate([x1, x2], axis=1)
#     nprint('x[5:,:]', x[5:,:])
#     # M, inliers = ransac(x, ptrs['fittingfn'], ptrs['distfn'],\
#                         # ptrs['degenfn'], s, RANSAC_threshold)
#     # now do a final fit on the data points considered to be inliers
#     # Mb = feval(fittingfn, [x1(:,inliers); x2(:,inliers)]);


#     self.y_bar = np.mean(y)
#     return # end of fit

#   def predict(self,M,X,t):
#     ''' Distance function
#     Function to evaluate the first order approximation of the geometric error
#     (Sampson distance) of the fit of a fundamental matrix with respect to a
#     set of matched points as needed by RANSAC. See: Hartley and Zisserman,
#     'Multiple View Geometry in Computer Vision', page 270. Note that this code
#     is allows for F being a cell array of fundamental matrices of
#     which we have to pick the best one. (A 7 point solution can return up to 3
#     solutions) '''

#     x1 = X[:,0:3]
#     x2 = X[:,3:6]
#     F = M.F;

#     if len(F) > 1: # we have several solutions each of which must be tested
#       nF = len(F) # Number of solutions to test
#       bestM = M
#       bestM.F = F[0] # initial allocation of best solution
#       ninliers = 0 # number of inliers

#       for k in range(1,nF):
#         x2tFx1 = np.zeros(1, len(x1))
#         for n in range(1,len(x1)):
#           x2tFx1[n] = x2[:,n].T @ F[k] @ x1[:,n]
#         Fx1  = F[k] @ x1
#         Ftx2 = F[k].T @ x2
#         # evaluate distances
#         d = x2tFx1**2/(Fx1[:,0]**2 + Fx1[1,:]**2 + Ftx2[0,:]**2 + Ftx2[1,:]**2)

#         inliers = np.nonzero(abs(d) < t) # indices of inlying points

#         if len(inliers) > ninliers:   # Record best solution
#           ninliers = len(inliers)
#           bestM.F = F[k]
#           bestInliers = inliers;
#     else: # we just have one solution
#       x2tFx1 = np.zeros(1,len(x1))
#       for n in range(len(x1)):
#         x2tFx1[n] = x2[:,n].T @ F @ x1[:,n]

#       Fx1  = F @ x1
#       Ftx2 = F.T @ x2

#       # Evaluate distances (Sampson Distance)
#       d =  x2tFx1**2 / (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);

#       bestInliers = find(abs(d) < t);     % Indices of inlying points
#       bestM = M;                          % Copy M directly to bestM

#     # return bestInliers, bestM

#     return np.ones(X.shape[0]) * self.y_bar


# class KMeansTransformer(TransformerMixin)




# %% Check degeneracy
# % Function to determine if a set of matched points will result
# % in a degeneracy in the calculation of a fundamental matrix as needed by
# % RANSAC.  This function assumes this cannot happen.

# function r = IsDegenerate(x)
# r = 0;
# end


# %% Pose Estimation
# % Estimates the relative rotation and translation between two camera views
# % using the 5 point quatenion algoirithm.
# %
# %
# % Arguments:
# %          x1, x2 - Two sets of N matched feature point coordinates. x1, x2
# %                   are 3xN matrices with each column corresponding to a point.
# %
# %          x      - If a single argument is supplied it is assumed that it
# %                   is in the form x = [x1; x2]
# % Output:
# %          M      - A structure that contains the estimated pose.
# %
# %
# % Copyright (c) 2016, Kaveh Fathian.
# % The University of Texas at Dallas.
# %
# %
# function M = PoseEstimator(varargin)

# [x1, x2, npts] = checkargs(varargin(:));

# % Recover the pose
# pose = QuEst_Ver1_1(x1(:,1:5), x2(:,1:5));  % QuEst algorithm

# % Pick the best pose solution
# res = QuatResidueVer3_1(x1, x2, pose.Q); % Scoring function
# [resMin,mIdx] = min(abs(res));
# q = pose.Q(:,mIdx);
# t = pose.T(:,mIdx);

# % Make a fundamental matrix from the recovered rotation and translation
# R = Q2R(q);
# Tx = Skew(t/norm(t));
# F = Tx * R;

# M.Q  = q;
# M.t  = t;
# M.m1 = x1;
# M.m2 = x2;
# M.F  = F;

# end


# %% Function to check argument values and set defaults
# %
# function [x1, x2, npts] = checkargs(arg)

# if length(arg) == 2
#     x1 = arg{1};
#     x2 = arg{2};
#     if ~all(size(x1)==size(x2))
#         error('Image dataset must have the same size.');
#     elseif size(x1,1) ~= 3
#         error('Image cordinates must come in a 3xN matrix.');
#     end

# elseif length(arg) == 1
#     if size(arg{1},1) ~= 6
#         error('Single input argument must be 6xN');
#     else
#         x1 = arg{1}(1:3,:);
#         x2 = arg{1}(4:6,:);
#     end
# else
#     error('Wrong number of arguments supplied');
# end

# npts = size(x1,2);
# if npts < 6
#     error('At least 6 points are needed to compute the fundamental matrix');
# end

# end


# %%
# % RANSAC - Robustly fits a model to data with the RANSAC algorithm
# %
# % Usage:
# %
# % [M, inliers] = ransac(x, fittingfn, distfn, degenfn s, t, feedback, ...
# %                       maxDataTrials, maxTrials)
# %
# % Arguments:
# %     x         - Data sets to which we are seeking to fit a model M
# %                 It is assumed that x is of size [d x Npts]
# %                 where d is the dimensionality of the data and Npts is
# %                 the number of data points.
# %
# %     fittingfn - Handle to a function that fits a model to s
# %                 data from x.  It is assumed that the function is of the
# %                 form:
# %                    M = fittingfn(x)
# %                 Note it is possible that the fitting function can return
# %                 multiple models (for example up to 3 fundamental matrices
# %                 can be fitted to 7 matched points).  In this case it is
# %                 assumed that the fitting function returns a cell array of
# %                 models.
# %                 If this function cannot fit a model it should return M as
# %                 an empty matrix.
# %
# %     distfn    - Handle to a function that evaluates the
# %                 distances from the model to data x.
# %                 It is assumed that the function is of the form:
# %                    [inliers, M] = distfn(M, x, t)
# %                 This function must evaluate the distances between points
# %                 and the model returning the indices of elements in x that
# %                 are inliers, that is, the points that are within distance
# %                 't' of the model.  Additionally, if M is a cell array of
# %                 possible models 'distfn' will return the model that has the
# %                 most inliers.  If there is only one model this function
# %                 must still copy the model to the output.  After this call M
# %                 will be a non-cell object representing only one model.
# %
# %     degenfn   - Handle to a function that determines whether a
# %                 set of datapoints will produce a degenerate model.
# %                 This is used to discard random samples that do not
# %                 result in useful models.
# %                 It is assumed that degenfn is a boolean function of
# %                 the form:
# %                    r = degenfn(x)
# %                 It may be that you cannot devise a test for degeneracy in
# %                 which case you should write a dummy function that always
# %                 returns a value of 1 (true) and rely on 'fittingfn' to return
# %                 an empty model should the data set be degenerate.
# %
# %     s         - The minimum number of samples from x required by
# %                 fittingfn to fit a model.
# %
# %     t         - The distance threshold between a data point and the model
# %                 used to decide whether the point is an inlier or not.
# %
# %     feedback  - An optional flag 0/1. If set to one the trial count and the
# %                 estimated total number of trials required is printed out at
# %                 each step.  Defaults to 0.
# %
# %     maxDataTrials - Maximum number of attempts to select a non-degenerate
# %                     data set. This parameter is optional and defaults to 100.
# %
# %     maxTrials - Maximum number of iterations. This parameter is optional and
# %                 defaults to 1000.
# %
# % Returns:
# %     M         - The model having the greatest number of inliers.
# %     inliers   - An array of indices of the elements of x that were
# %                 the inliers for the best model.
# %
# %
# % Note that the desired probability of choosing at least one sample free from
# % outliers is set at 0.99.  You will need to edit the code should you wish to
# % change this (it should probably be a parameter)
# %
# % For an example of the use of this function see RANSACFITHOMOGRAPHY or
# % RANSACFITPLANE

# % References:
# %    M.A. Fishler and  R.C. Boles. "Random sample concensus: A paradigm
# %    for model fitting with applications to image analysis and automated
# %    cartography". Comm. Assoc. Comp, Mach., Vol 24, No 6, pp 381-395, 1981
# %
# %    Richard Hartley and Andrew Zisserman. "Multiple View Geometry in
# %    Computer Vision". pp 101-113. Cambridge University Press, 2001

# % Copyright (c) 2003-2013 Peter Kovesi
# % Centre for Exploration Targeting
# % The University of Western Australia
# % peter.kovesi at uwa edu au
# % http://www.csse.uwa.edu.au/~pk
# %
# % Permission is hereby granted, free of charge, to any person obtaining a copy
# % of this software and associated documentation files (the "Software"), to deal
# % in the Software without restriction, subject to the following conditions:
# %
# % The above copyright notice and this permission notice shall be included in
# % all copies or substantial portions of the Software.
# %
# % The Software is provided "as is", without warranty of any kind.
# %
# % May      2003 - Original version
# % February 2004 - Tidied up.
# % August   2005 - Specification of distfn changed to allow model fitter to
# %                 return multiple models from which the best must be selected
# % Sept     2006 - Random selection of data points changed to ensure duplicate
# %                 points are not selected.
# % February 2007 - Jordi Ferrer: Arranged warning printout.
# %                               Allow maximum trials as optional parameters.
# %                               Patch the problem when non-generated data
# %                               set is not given in the first iteration.
# % August   2008 - 'feedback' parameter restored to argument list and other
# %                 breaks in code introduced in last update fixed.
# % December 2008 - Octave compatibility mods
# % June     2009 - Argument 'MaxTrials' corrected to 'maxTrials'!
# % January  2013 - Separate code path for Octave no longer needed

# function [M, inliers] = ransac(x, fittingfn, distfn, degenfn, s, t, feedback, ...
#                                maxDataTrials, maxTrials)

# % Test number of parameters
# narginchk(6, 9) ;

# if nargin < 9; maxTrials = 1000;    end;
# if nargin < 8; maxDataTrials = 100; end;
# if nargin < 7; feedback = 0;        end;

# [rows, npts] = size(x);

# p = 0.99;         % Desired probability of choosing at least one sample
#                   % free from outliers (probably should be a parameter)

# bestM = NaN;      % Sentinel value allowing detection of solution failure.
# trialcount = 0;
# bestscore =  0;
# N = 1;            % Dummy initialisation for number of trials.

# while N > trialcount

#     % Select at random s datapoints to form a trial model, M.
#     % In selecting these points we have to check that they are not in
#     % a degenerate configuration.
#     degenerate = 1;
#     count = 1;
#     while degenerate
#         % Generate s random indicies in the range 1..npts
#         % (If you do not have the statistics toolbox with randsample(),
#         % use the function RANDOMSAMPLE from my webpage)
#         if ~exist('randsample', 'file')
#             ind = randomsample(npts, s);
#         else
#             ind = randsample(npts, s);
#         end

#         % Test that these points are not a degenerate configuration.
#         degenerate = feval(degenfn, x(:,ind));

#         if ~degenerate
#             % Fit model to this random selection of data points.
#             % Note that M may represent a set of models that fit the data in
#             % this case M will be a cell array of models
#             M = feval(fittingfn, x(:,ind));

#             % Depending on your problem it might be that the only way you
#             % can determine whether a data set is degenerate or not is to
#             % try to fit a model and see if it succeeds.  If it fails we
#             % reset degenerate to true.
#             if isempty(M.F)
#                 degenerate = 1;
#             end
#         end

#         % Safeguard against being stuck in this loop forever
#         count = count + 1;
#         if count > maxDataTrials
#             warning('Unable to select a nondegenerate data set');
#             break
#         end
#     end

#     % Once we are out here we should have some kind of model...
#     % Evaluate distances between points and model returning the indices
#     % of elements in x that are inliers.  Additionally, if M is a cell
#     % array of possible models 'distfn' will return the model that has
#     % the most inliers.  After this call M will be a non-cell object
#     % representing only one model.
#     [inliers, M] = feval(distfn, M, x, t);

#     % Find the number of inliers to this model.
#     ninliers = length(inliers);

#     if ninliers > bestscore    % Largest set of inliers so far...
#         bestscore = ninliers;  % Record data for this model
#         bestinliers = inliers;
#         bestM = M;

#         % Update estimate of N, the number of trials to ensure we pick,
#         % with probability p, a data set with no outliers.
#         fracinliers =  ninliers/npts;
#         pNoOutliers = 1 -  fracinliers^s;
#         pNoOutliers = max(eps, pNoOutliers);  % Avoid division by -Inf
#         pNoOutliers = min(1-eps, pNoOutliers);% Avoid division by 0.
#         N = log(1-p)/log(pNoOutliers);
#     end

#     trialcount = trialcount+1;
#     if feedback
#         fprintf('trial %d out of %d         \r',trialcount, ceil(N));
#     end

#     % Safeguard against being stuck in this loop forever
#     if trialcount > maxTrials
#         warning( ...
#         sprintf('ransac reached the maximum number of %d trials',...
#                 maxTrials));
#         break
#     end
# end

# if feedback, fprintf('\n'); end

# if isstruct(bestM)   % We got a solution
#     M = bestM;
#     inliers = bestinliers;
# else
#     M = [];
#     inliers = [];
#     error('ransac was unable to find a useful solution');
# end




# end

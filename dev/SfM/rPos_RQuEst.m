%   rPos_RQuEst Compute relative up-to-scale pose of calibrated camera
%   [relativeOrientation, relativeLocation] =   rPos_RQuEst(M,
%   cameraParams, inlierPoints1, inlierPoints2) returns the orientation and
%   up-to-scale location of a calibrated camera relative to its previous
%   pose. relativeLocation is always a unit vector.
%   M is an essential or fundamental 3-by-3 matrix, or a projective2d
%   object containing a homography matrix. cameraParams is a
%   cameraParameters or cameraIntrinsics object. inlierPoints1 and
%   inlierPoints2 are matching inlier points from the two views
%   corresponding to the two poses. M, inlierPoints1, and inlierPoints2 are
%   returned by the estimateEssentialMatrix, estimateFundamentalMatrix,
%   or estimateGeometricTransform2D functions. relativeOrientation is a
%   3-by-3-by-N rotation matrix. relativeLocation is a N-by-3 matrix with a
%   unit vector at each row. N is one if M is an essential or fundamental
%   matrix, and is up to two if M is a projective2d object.
%   [...] =   rPos_RQuEst(M, cameraParams1, cameraParams2, inlierPoints1, inlierPoints2)
%   returns the orientation and location of camera 2 relative to camera 1.
%   cameraParams1 and cameraParams2 are cameraParameters or
%   cameraIntrinsics objects containing the parameters of camera 1 and
%   camera 2 respectively.
%   [..., validPointsFraction] =   rPos_RQuEst(...) additionally
%   returns the fraction of the inlier points that project in front of both
%   cameras. If this fraction is too small (e. g. less than 0.9), that can
%   indicate that the input matrix M is unreliable.
%   Notes
%   -----
%   - You can compute the camera extrinsics as follows:
%     [rotationMatrix, translationVector] = cameraPoseToExtrinsics(
%       relativeOrientation, relativeLocation)
%   - The   rPos_RQuEst function uses inlierPoints1 and inlierPoints2 to
%     determine which one of the multiple possible solutions is physically
%     realizable. If the input M is a projective2d object, there could be
%     up to two solutions that are equally realizable.
%    Class Support
%    -------------
%    M must be double or single. cameraParams must be a cameraParameters or
%    cameraIntrinsics object. inlierPoints1 and inlierPoints2 can be
%    double, single, or any of the point feature types. location and
%    orientation are the same class as M.
%  Example: Structure from motion from two views
%  ---------------------------------------------
%  % This example shows you how to build a point cloud based on features
%  % matched between two images of an object.
%  % <a href="matlab:helpview(fullfile(docroot,'toolbox','vision','vision.map'),'StructureFromMotionExample')">View example</a>
%  See also estimateWorldCameraPose, cameraCalibrator, estimateCameraParameters, 
%    estimateEssentialMatrix, estimateFundamentalMatrix, cameraMatrix,
%    plotCamera, triangulate, triangulateMultiview, cameraPoseToExtrinsics,
%    extrinsics, estimateGeometricTransform2D.
% Copyright 2015-2020 The MathWorks, Inc.
% References:
% -----------
% [1] R. Hartley, A. Zisserman, "Multiple View Geometry in Computer
% Vision," Cambridge University Press, 2003.
%
% [2] R. Hartley, P. Sturm. "Triangulation." Computer Vision and
% Image Understanding. Vol 68, No. 2, November 1997, pp. 146-157
%
% [3] O. Faugeras and F. Lustman, Motion and structure from motion in a
% piecewise planar environment, in International Journal of Pattern
% Recognition and Artificial Intelligence, 2(3):485508, 1988.
%#codegen
function [Q, T, validFrac] =  rPos_RQuEst(F, varargin)
  [camPar1, camPar2, inLPts1, inLPts2] = parseInputs(F, varargin{:});
  if isa(camPar1, 'cameraIntrinsics')
    camPar1 = camPar1.CameraParameters;
  end 
  if isa(camPar2, 'cameraIntrinsics')
    camPar2 = camPar2.CameraParameters;
  end
  K1 = camPar1.IntrinsicMatrix;
  K2 = camPar2.IntrinsicMatrix;
  
  
  if ~isa(F, 'images.geotrans.internal.GeometricTransformation')
    if isFundamentalMatrix(F, inLPts1, inLPts2, K1, K2)
      E = K2 * F * K1'; % Compute the essential matrix
    else
      E = F;    % We already have the essential matrix
    end
    [Rs, Ts] = vision.internal.calibration.decomposeEssentialMatrix(E);
  else
   H = (K1 * F.T / K2)';
   [Rs, Ts] = vision.internal.calibration.decomposeHomographyMatrix(H);
  end




  [R, t, validFrac] = chooseRealizableSol(Rs, Ts, camPar1, camPar2, inLPts1, ...
    inLPts2);
  % R and t are currently the transformation from camera1's coordinates into
  % camera2's coordinates. To find the location and orientation of camera2 in
  % camera1's coordinates we must take their inverse.
  Q = R;
  T = t;
  if size(T, 1) == 1
    Q = R(:,:,1)';
    T = -t * Q;    
  else
    [~, idx] = sort(t(:, 3)); % Sort t to make consistent order of output
    for n = 1:size(t, 1)
      Q(:,:,n) = R(:,:,idx(n))';
      T(n, :) = -t(idx(n), :) * Q(:,:,n);
    end
  end
%--------------------------------------------------------------------------


function tf = isFundamentalMatrix(M, inLPts1, inLPts2, K1, K2)
  % Assume M is F
  numPoints = size(inLPts1, 1);
  pts1h = [inLPts1, ones(numPoints, 1, 'like', inLPts1)];
  pts2h = [inLPts2, ones(numPoints, 1, 'like', inLPts2)];
  err_F = mean(abs(diag(pts2h * M * pts1h')));
  F = K2 \ M / K1'; % Assume M is E
  err_E = mean(abs(diag(pts2h * F * pts1h')));
  tf = err_F < err_E;
%--------------------------------------------------------------------------


function [camPar1, camPar2, inLPts1, inLPts2] = parseInputs(F, varargin)
  narginchk(4, 5);
  if ~isnumeric(F)
    validateattributes(F, {'affine2d','projective2d'}, {'scalar','nonempty'}, mfilename, 'F');
  else
    validateattributes(F, {'single', 'double'}, {'real','nonsparse','finite', '2d', 'size',[3 3]}, mfilename, 'F');
  end
  camPar1 = varargin{1};
  if isa(varargin{2}, 'cameraParameters') || isa(varargin{2}, 'cameraIntrinsics')
    camPar2 = varargin{2};
    paramVarName = 'cameraParams';
    idx = 2;
  else
    paramVarName = 'cameraParams1';
    camPar2 = camPar1;
    idx = 1;
  end
  vision.internal.inputValidation.checkIntrinsicsAndParameters( ...
    camPar1, true, mfilename, paramVarName);
  vision.internal.inputValidation.checkIntrinsicsAndParameters( ...
    camPar2, true, mfilename, paramVarName);
  pts1 = varargin{idx + 1};
  pts2 = varargin{idx + 2};
  [inLPts1, inLPts2] = vision.internal.inputValidation.checkAndConvertMatchedPoints(...
    pts1, pts2, mfilename, 'inlierPoints1', 'inlierPoints2');
  %coder.internal.errorIf(isempty(points1), 'vision:relativeCameraPose:emptyInlierPoints');
  assert(false, "[  rPos_RQuEst}->> empty inliers!");
%--------------------------------------------------------------------------


% Determine which of the 4 possible solutions is physically realizable.
% A physically realizable solution is the one which puts reconstructed 3D
% points in front of both cameras. There could be two solutions if the R
% and t are extracted from homography matrix
function [R, t, validFract] = chooseRealizableSol(Rs, Ts, camPar1, ...
  camPar2, pts1, pts2)
  numNegatives = zeros(1, size(Ts, 1));
  camMatrix1 = cameraMatrix(camPar1, eye(3), [0 0 0]);
  for i = 1:size(Ts, 1)
    camMatrix2 = cameraMatrix(camPar2, Rs(:,:,i)', Ts(i, :));
    m1 = triangulateMidPoint(pts1, pts2, camMatrix1, camMatrix2);
    m2 = bsxfun(@plus, m1 * Rs(:,:,i)', Ts(i, :));
    numNegatives(i) = sum((m1(:,3) < 0) | (m2(:,3) < 0));
  end
  val = min(numNegatives);
  idx = find(numNegatives == val);
  validFract = 1 - (val / size(pts1, 1));
  R = zeros(3, 3, length(idx), 'like', Rs);
  t = zeros(length(idx), 3, 'like', Ts);
  for n = 1 : length(idx)
    R0 = Rs(:,:,idx(n))';
    t0 = Ts(idx(n), :);
    tNorm = norm(t0);
    if tNorm ~= 0
      t0 = t0 ./ tNorm;
    end
    R(:,:,n) = R0;
    t(n, :) = t0;
  end
%--------------------------------------------------------------------------
% Simple triangulation algorithm from
% Hartley, Richard and Peter Sturm. "Triangulation." Computer Vision and
% Image Understanding. Vol 68, No. 2, November 1997, pp. 146-157
function pts3D = triangulateMidPoint(pts1, pts2, P1, P2)
  numPts = size(pts1, 1);
  pts3D = zeros(numPts, 3, 'like', pts1);
  P1 = P1';
  P2 = P2';
  M1 = P1(1:3, 1:3);
  M2 = P2(1:3, 1:3);
  c1 = -M1 \ P1(:,4);
  c2 = -M2 \ P2(:,4);
  y = c2 - c1;
  u1 = [pts1, ones(numPts, 1, 'like', pts1)]';
  u2 = [pts2, ones(numPts, 1, 'like', pts1)]';
  a1 = M1 \ u1;
  a2 = M2 \ u2;
  isCodegen  = ~isempty(coder.target);
  condThresh = eps(class(pts1));
  for i = 1:numPts
    A   = [a1(:,i), -a2(:,i)];  
    AtA = A'*A;
    if rcond(AtA) < condThresh
      % Guard against matrix being singular or ill-conditioned
      p    = inf(3, 1, class(pts1));
      p(3) = -p(3);
    else
      if isCodegen
        % mldivide on square matrix is faster in codegen mode.
        alpha = AtA \ A' * y;
      else
        alpha = A \ y;        
      end
      p = (c1 + alpha(1) * a1(:,i) + c2 + alpha(2) * a2(:,i)) / 2;
    end
    pts3D(i, :) = p';
  end

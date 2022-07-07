function [Ori, Loc, inLFract] = RQuEst_relCamPose(F, varargin)
  [camP1, camP2, inLPts1, inLPts2] = parseInputs(F, varargin{:});
  if isa(camP1, 'cameraIntrinsics')
    camP1 = camP1.CameraParameters;
  end
  if isa(camP2, 'cameraIntrinsics')
    camP2 = camP2.CameraParameters;
  end
  K1 = camP1.IntrinsicMatrix;
  K2 = camP2.IntrinsicMatrix;



  
  if ~isa(F, 'images.geotrans.internal.GeometricTransformation')
    if isFundamentalMatrix(F, inLPts1, inLPts2, K1, K2)
      E = K2 * F * K1'; % Compute the essential matrix
    else
      E = F; % We already have the essential matrix
    end
    [Rs, Ts] = vision.internal.calibration.decomposeEssentialMatrix(E);
  else
   H = (K1 * F.T / K2)';
   [Rs, Ts] = vision.internal.calibration.decomposeHomographyMatrix(H);
  end




  
  [R, t, inLFract] = chooseRealizableSolution(Rs, Ts, camP1, camP2, inLPts1, ...
      inLPts2);
  % R and t are currently the transformation from camera1's coordinates into
  % camera2's coordinates. To find the location and orientation of camera2 in
  % camera1's coordinates we must take their inverse.
  Ori = R;
  Loc = t;
  if size(Loc, 1) == 1
    Ori = R(:,:,1)';
    Loc = -t * Ori;    
  else
    % Sort t to make consistent order of output
    [~, idx] = sort(t(:, 3));
    for n = 1:size(t, 1)
      Ori(:,:,n) = R(:,:,idx(n))';
      Loc(n, :) = -t(idx(n), :) * Ori(:,:,n);
    end
  end
end 
%--------------------------------------------------------------------------
function tf = isFundamentalMatrix(M, inlierPoints1, inlierPoints2, K1, K2)
  % Assume M is F
  numPoints = size(inlierPoints1, 1);
  pts1h = [inlierPoints1, ones(numPoints, 1, 'like', inlierPoints1)];
  pts2h = [inlierPoints2, ones(numPoints, 1, 'like', inlierPoints2)];
  errorF = mean(abs(diag(pts2h * M * pts1h')));
  F = K2 \ M / K1'; % Assume M is E
  errorE = mean(abs(diag(pts2h * F * pts1h')));
  tf = errorF < errorE;
end 
%--------------------------------------------------------------------------
function [camP1, camP2, inLPts1, inLPts2] = parseInputs(F, varargin)
  narginchk(4, 5);
  if ~isnumeric(F)
  validateattributes(F, {'affine2d','projective2d'}, {'scalar','nonempty'}, mfilename, 'F');
  else
    validateattributes(F, {'single', 'double'}, {'real','nonsparse','finite', '2d', 'size',[3 3]}, mfilename, 'F');
  end
  camP1 = varargin{1};
  if isa(varargin{2}, 'cameraParameters') || isa(varargin{2}, 'cameraIntrinsics')
    camP2 = varargin{2};
    paramVarName = 'cameraParams';
    idx = 2;
  else
    paramVarName = 'cameraParams1';
    camP2 = camP1;
    idx = 1;
  end
  vision.internal.inputValidation.checkIntrinsicsAndParameters(camP1, true, mfilename, paramVarName);
  vision.internal.inputValidation.checkIntrinsicsAndParameters(camP2, true, mfilename, paramVarName);
  pts1 = varargin{idx + 1};
  pts2 = varargin{idx + 2};
  [inLPts1, inLPts2] = vision.internal.inputValidation.checkAndConvertMatchedPoints(...
    pts1, pts2, mfilename, 'inlierPoints1', 'inlierPoints2');
  coder.internal.errorIf(isempty(pts1), 'vision:relativeCameraPose:emptyInlierPoints');
end 
%--------------------------------------------------------------------------
% Determine which of the 4 possible solutions is physically realizable.
% A physically realizable solution is the one which puts reconstructed 3D
% points in front of both cameras. There could be two solutions if the R
% and t are extracted from homography matrix
function [R, t, valFract] = chooseRealizableSolution(Rs, Ts, camP1, camP2, pts1, pts2)
  numNegs = zeros(1, size(Ts, 1));
  camMat1 = cameraMatrix(camP1, eye(3), [0 0 0]);
  for i = 1:size(Ts, 1)
    camMat2 = cameraMatrix(camP2, Rs(:,:,i)', Ts(i, :));
    m1 = triangulateMidPoint(pts1, pts2, camMat1, camMat2);
    m2 = bsxfun(@plus, m1 * Rs(:,:,i)', Ts(i, :));
    numNegs(i) = sum((m1(:,3) < 0) | (m2(:,3) < 0));
  end
  val = min(numNegs);
  idx = find(numNegs == val);
  valFract = 1 - (val / size(pts1, 1));
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
end 
%--------------------------------------------------------------------------
% Simple triangulation algorithm from
% Hartley, Richard and Peter Sturm. "Triangulation." Computer Vision and
% Image Understanding. Vol 68, No. 2, November 1997, pp. 146-157
function points3D = triangulateMidPoint(points1, points2, P1, P2)
  numPoints = size(points1, 1);
  points3D = zeros(numPoints, 3, 'like', points1);
  P1 = P1';
  P2 = P2';
  M1 = P1(1:3, 1:3);
  M2 = P2(1:3, 1:3);
  c1 = -M1 \ P1(:,4);
  c2 = -M2 \ P2(:,4);
  y = c2 - c1;
  u1 = [points1, ones(numPoints, 1, 'like', points1)]';
  u2 = [points2, ones(numPoints, 1, 'like', points1)]';
  a1 = M1 \ u1;
  a2 = M2 \ u2;
  isCodegen  = ~isempty(coder.target);
  condThresh = eps(class(points1));
  for i = 1:numPoints
    A   = [a1(:,i), -a2(:,i)];  
    AtA = A'*A;
    if rcond(AtA) < condThresh
      p    = inf(3, 1, class(points1)); % Guard against matrix being singular or ill-conditioned
      p(3) = -p(3);
    else
      if isCodegen
        alpha = AtA \ A' * y; % mldivide on square matrix is faster in codegen mode.
      else
        alpha = A \ y;        
      end
      p = (c1 + alpha(1) * a1(:,i) + c2 + alpha(2) * a2(:,i)) / 2;
    end
    points3D(i, :) = p';
  end
end 

function [orientation, location, validPointsFraction] = RQuEst_relCamPose(F, varargin)

[cameraParams1, cameraParams2, inlierPoints1, inlierPoints2] = ...
    parseInputs(F, varargin{:});

if isa(cameraParams1, 'cameraIntrinsics')
    cameraParams1 = cameraParams1.CameraParameters;
end

if isa(cameraParams2, 'cameraIntrinsics')
    cameraParams2 = cameraParams2.CameraParameters;
end

K1 = cameraParams1.IntrinsicMatrix;
K2 = cameraParams2.IntrinsicMatrix;

if ~isa(F, 'images.geotrans.internal.GeometricTransformation')
    if isFundamentalMatrix(F, inlierPoints1, inlierPoints2, K1, K2)
        % Compute the essential matrix
        E = K2 * F * K1';
    else
        % We already have the essential matrix
        E = F;
    end
    [Rs, Ts] = vision.internal.calibration.decomposeEssentialMatrix(E);
else
     H = (K1 * F.T / K2)';
     [Rs, Ts] = vision.internal.calibration.decomposeHomographyMatrix(H);
end

[R, t, validPointsFraction] = chooseRealizableSolution(Rs, Ts, cameraParams1, cameraParams2, inlierPoints1, ...
    inlierPoints2);

% R and t are currently the transformation from camera1's coordinates into
% camera2's coordinates. To find the location and orientation of camera2 in
% camera1's coordinates we must take their inverse.
orientation = R;
location = t;
if size(location, 1) == 1
    orientation = R(:,:,1)';
    location = -t * orientation;    
else
    % Sort t to make consistent order of output
    [~, idx] = sort(t(:, 3));
    for n = 1:size(t, 1)
        orientation(:,:,n) = R(:,:,idx(n))';
        location(n, :) = -t(idx(n), :) * orientation(:,:,n);
    end
end

%--------------------------------------------------------------------------
function tf = isFundamentalMatrix(M, inlierPoints1, inlierPoints2, K1, K2)
% Assume M is F
numPoints = size(inlierPoints1, 1);
pts1h = [inlierPoints1, ones(numPoints, 1, 'like', inlierPoints1)];
pts2h = [inlierPoints2, ones(numPoints, 1, 'like', inlierPoints2)];
errorF = mean(abs(diag(pts2h * M * pts1h')));

% Assume M is E
F = K2 \ M / K1';
errorE = mean(abs(diag(pts2h * F * pts1h')));

tf = errorF < errorE;

%--------------------------------------------------------------------------
function [cameraParams1, cameraParams2, inlierPoints1, inlierPoints2] = ...
    parseInputs(F, varargin)
narginchk(4, 5);

if ~isnumeric(F)
   validateattributes(F, {'affine2d','projective2d'}, {'scalar','nonempty'}, mfilename, 'F');
else
   validateattributes(F, {'single', 'double'}, {'real','nonsparse','finite', '2d', 'size',[3 3]}, mfilename, 'F');
end

cameraParams1 = varargin{1};
if isa(varargin{2}, 'cameraParameters') || isa(varargin{2}, 'cameraIntrinsics')
    cameraParams2 = varargin{2};
    paramVarName = 'cameraParams';
    idx = 2;
else
    paramVarName = 'cameraParams1';
    cameraParams2 = cameraParams1;
    idx = 1;
end
vision.internal.inputValidation.checkIntrinsicsAndParameters( ...
    cameraParams1, true, mfilename, paramVarName);

vision.internal.inputValidation.checkIntrinsicsAndParameters( ...
    cameraParams2, true, mfilename, paramVarName);

points1 = varargin{idx + 1};
points2 = varargin{idx + 2};
[inlierPoints1, inlierPoints2] = ...
    vision.internal.inputValidation.checkAndConvertMatchedPoints(...
    points1, points2, mfilename, 'inlierPoints1', 'inlierPoints2');

coder.internal.errorIf(isempty(points1), 'vision:relativeCameraPose:emptyInlierPoints');

%--------------------------------------------------------------------------
% Determine which of the 4 possible solutions is physically realizable.
% A physically realizable solution is the one which puts reconstructed 3D
% points in front of both cameras. There could be two solutions if the R
% and t are extracted from homography matrix
function [R, t, validFraction] = chooseRealizableSolution(Rs, Ts, cameraParams1, ...
    cameraParams2, points1, points2)
numNegatives = zeros(1, size(Ts, 1));

camMatrix1 = cameraMatrix(cameraParams1, eye(3), [0 0 0]);
for i = 1:size(Ts, 1)
    camMatrix2 = cameraMatrix(cameraParams2, Rs(:,:,i)', Ts(i, :));
    m1 = triangulateMidPoint(points1, points2, camMatrix1, camMatrix2);
    m2 = bsxfun(@plus, m1 * Rs(:,:,i)', Ts(i, :));
    numNegatives(i) = sum((m1(:,3) < 0) | (m2(:,3) < 0));
end

val = min(numNegatives);
idx = find(numNegatives == val);

validFraction = 1 - (val / size(points1, 1));

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
        % Guard against matrix being singular or ill-conditioned
        p    = inf(3, 1, class(points1));
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
    points3D(i, :) = p';

end

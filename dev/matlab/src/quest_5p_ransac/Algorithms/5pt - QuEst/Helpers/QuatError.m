%% Metric for 3D rotation error in quaternions
function err = QuatError(q1, Q2)

% err = 1 - abs(q1.' * q2);

% Normalize s.t. each column of Q has norm 1
QNrm = sqrt(sum(Q2.^2,1));
Q2 = bsxfun(@rdivide, Q2, QNrm);

% Normalize s.t. each column of Q has norm 1
QNrm = sqrt(sum(q1.^2,1));
q1 = bsxfun(@rdivide, q1, QNrm);

numSols = size(Q2,2);
err = zeros(1,numSols);

for i = 1 : numSols
    q2 = Q2(:,i);
    err(i) = (1/pi) * acos(min([abs(q1.' * q2),1]));
end
    
end

























































%% Code to veryfy the five point algorithm: 


% NOTE: If a mex wrapper is not generated for the .c file yet, first run
%           mex calibrated_fivepoint_helper.c
%       at the command prompt.

Q1 = rand(3,5);
Q2 = rand(3,5);

Evec   = calibrated_fivepoint(Q1,Q2); % Five point algorithm base on GB
% Evec   = calibrated_fivepoint_non_gb (Q1, Q2); % Non GB five point algorithm

for i=1:size(Evec,2)
  E = reshape(Evec(:,i),3,3);
  % Check determinant constraint! 
  det( E )
  % Check trace constraint
  2 *E*transpose(E)*E -trace( E*transpose(E))*E
  % Check reprojection errors
  diag( Q1'*E*Q2)
end




































































































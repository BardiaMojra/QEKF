% Example:
%       X = rand(4,3)-0.5
%       dispn(X,8)
%       disp(' ')
%       dispn(X,[2,4,6])
%
% See also:
%   DISP(X) displays the matrix in the current screen format.
% -------------------------------------------------------------------------
function [] = dispn(X,N)
  [I,J] =   size(X);
     K  = length(N);
  if K == 1   
    N = N * ones(1,J);
  elseif K ~= J   
    disp('ERROR: length(N) must either be 1 or equal to the number of columns of X.')
    return
  end
  for i = 1:I
    string = '';
    for j = 1:J
      if X(i,j) >= 0
        string = [string,'   %.', num2str( N(j) ) ,'f'];
      else
        string = [string,'  %.' , num2str( N(j) ) ,'f'];        
      end
    end      
    fsprintf(string , X(i,:));
  end
end 
    
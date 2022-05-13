%
%  Ver 1_0:
%           - Recover translation directly without recovering depths
%             (See SymbVer)
%
function [T] = FindTrans_Ver1_0(m,n, Q)


% Preallocate variables
numInp  = size(Q,2);        % Number of rotation matrices in input
T       = zeros(3,numInp);  % Translation vector associated to each quaternion


for k = 1 : numInp
    % Stack rigid motion constraints into matrix-vector form M * z = b
    q = Q(:,k);
    
    Cf = MakeCoeff(m,n, q);
    
    [~,~,V] = svd(Cf);      
    
    % Store the results
    T(:,k) = V(:,end);
end



end




%%
function Cf = MakeCoeff(m,n, q)

w = q(1);
x = q(2);
y = q(3);
z = q(4);

mx = m(1,:).';
my = m(2,:).';
nx = n(1,:).';
ny = n(2,:).';

Cf = [ 2.*y.*z - 2.*w.*x + my.*w^2 - my.*x^2 - ny.*w^2 + my.*y^2 + ny.*x^2 - my.*z^2 + ny.*y^2 - ny.*z^2 + 2.*mx.*w.*z + 2.*mx.*x.*y - 2.*my.*ny.*w.*x + 2.*mx.*ny.*w.*y - 2.*mx.*ny.*x.*z - 2.*my.*ny.*y.*z, nx.*w^2 - 2.*x.*z - mx.*w^2 - mx.*x^2 - 2.*w.*y + mx.*y^2 - nx.*x^2 + mx.*z^2 - nx.*y^2 + nx.*z^2 + 2.*my.*w.*z - 2.*my.*x.*y + 2.*my.*nx.*w.*x - 2.*mx.*nx.*w.*y + 2.*mx.*nx.*x.*z + 2.*my.*nx.*y.*z, mx.*ny.*w^2 - my.*nx.*w^2 + mx.*ny.*x^2 + my.*nx.*x^2 - mx.*ny.*y^2 - my.*nx.*y^2 - mx.*ny.*z^2 + my.*nx.*z^2 + 2.*nx.*w.*x + 2.*ny.*w.*y + 2.*ny.*x.*z - 2.*nx.*y.*z - 2.*mx.*nx.*w.*z - 2.*mx.*nx.*x.*y - 2.*my.*ny.*w.*z + 2.*my.*ny.*x.*y];


end


























































































































































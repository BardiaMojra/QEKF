% 5pt example app
%
%
% This function takes coordinates of 5 feature points and returns all
% solutions hypotheis for the Essential matrix. 
%
%
function Eout = PolyEigWrapper(m1,m2)

[E] = peig5pt(m1, m2);


numSol = size(E,1)/3;
Eout = zeros(3,3,numSol);

for i = 1 : numSol
    
    Ei = E((i*3-2):(i*3), :);
    Ei = Ei ./ Ei(3,3);
    
    Eout(:,:,i) =  Ei;
end











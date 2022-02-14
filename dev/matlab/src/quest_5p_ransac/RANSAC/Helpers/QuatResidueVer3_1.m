%% Calculate the residue value |C x - c| for estimated quaternion solutions
%
%
function residu = QuatResidueVer3_1(m1, m2, qSol)

% numSol = size(qSol,2);

% Coefficinet matrix in the linearized system of multinomials (C * x = c)
C0 = CoefsVer3_1_1(m1,m2);

q1 = qSol(1,:);
q2 = qSol(2,:);
q3 = qSol(3,:);
q4 = qSol(4,:);

xVec = [q1.^4
        q1.^3.*q2
        q1.^2.*q2.^2
        q1.*q2.^3
        q2.^4
        q1.^3.*q3
        q1.^2.*q2.*q3
        q1.*q2.^2.*q3
        q2.^3.*q3
        q1.^2.*q3.^2
        q1.*q2.*q3.^2
        q2.^2.*q3.^2
        q1.*q3.^3
        q2.*q3.^3
        q3.^4
        q1.^3.*q4
        q1.^2.*q2.*q4
        q1.*q2.^2.*q4
        q2.^3.*q4
        q1.^2.*q3.*q4
        q1.*q2.*q3.*q4
        q2.^2.*q3.*q4
        q1.*q3.^2.*q4
        q2.*q3.^2.*q4
        q3.^3.*q4
        q1.^2.*q4.^2
        q1.*q2.*q4.^2
        q2.^2.*q4.^2
        q1.*q3.*q4.^2
        q2.*q3.*q4.^2
        q3.^2.*q4.^2
        q1.*q4.^3
        q2.*q4.^3
        q3.*q4.^3
        q4.^4];

% Residues
residuMat = C0 * xVec;

residu = sum( abs(residuMat), 1);




end


















































































































































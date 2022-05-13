% 
% Ver 2_0:
%           - Points are preconditioned to have the best results
%
%       NOTE:  THE RESULTS ARE NO DIFFERENT FROM THE ORIGINAL VERSION.
%
function E = sevenp_Ver2_0(x1,x2) 

% Normalise each set of points so that the origin 
% is at centroid and mean distance from origin is sqrt(2). 
% normalise2dpts also ensures the scale parameter is 1.
[x1, T1] = normalise2dpts(x1);
[x2, T2] = normalise2dpts(x2);


Q1 = x1';
Q2 = x2';

Q = [Q1(:,1).*Q2(:,1) , ...
     Q1(:,2).*Q2(:,1) , ...
     Q1(:,3).*Q2(:,1) , ...
     Q1(:,1).*Q2(:,2) , ...
     Q1(:,2).*Q2(:,2) , ...
     Q1(:,3).*Q2(:,2) , ...
     Q1(:,1).*Q2(:,3) , ...
     Q1(:,2).*Q2(:,3) , ...
     Q1(:,3).*Q2(:,3) ] ;

%EE = null( Q);
[U,S,V] = svd(Q,0);
EE = V(:,8:9);

F0 = reshape( EE(:,1),3,3);
F1 = reshape( EE(:,2),3,3);


% if 0 
%   F0 = [sym('f011') sym('f012') sym('f013') ; 
%   sym('f021') sym('f022') sym('f023') ; 
%   sym('f031') sym('f032') sym('f033') ; ]
% 
% 
%   F1 = [sym('f111') sym('f112') sym('f113') ; 
%   sym('f121') sym('f122') sym('f123') ; 
%   sym('f131') sym('f132') sym('f133') ; ]
% 
%   EQ= det( F0 + sym('x') *F1 ) ;
% 
%   maple('coeff', EQ, 'x', 3 )
%   maple('coeff', EQ, 'x', 2 )
%   maple('coeff', EQ, 'x', 1 )
%   maple('coeff', EQ, 'x', 0 )
% end


f011 = F0(1,1); 
f012 = F0(1,2); 
f013 = F0(1,3); 
f021 = F0(2,1); 
f022 = F0(2,2); 
f023 = F0(2,3); 
f031 = F0(3,1); 
f032 = F0(3,2); 
f033 = F0(3,3); 

f111 = F1(1,1); 
f112 = F1(1,2); 
f113 = F1(1,3); 
f121 = F1(2,1); 
f122 = F1(2,2); 
f123 = F1(2,3); 
f131 = F1(3,1); 
f132 = F1(3,2); 
f133 = F1(3,3); 


     a3= f111*f122*f133-f111*f123*f132-f121*f112*f133+f121*f113*f132+f131*f112*f123-f131*f113*f122;

     a2=f111*f022*f133+f111*f122*f033-f111*f023*f132-f111*f123*f032+f011*f122*f133-f011*f123*f132-f121*f012*f133-f121*f112*f033+f121*f013*f132+f121*f113*f032-f021*f112*f133+f021*f113*f132+f131*f012*f123+f131*f112*f023-f131*f013*f122-f131*f113*f022+f031*f112*f123-f031*f113*f122;

     a1=f111*f022*f033-f111*f023*f032+f011*f022*f133+f011*f122*f033-f011*f023*f132-f011*f123*f032-f121*f012*f033+f121*f013*f032-f021*f012*f133-f021*f112*f033+f021*f013*f132+f021*f113*f032+f131*f012*f023-f131*f013*f022+f031*f012*f123+f031*f112*f023-f031*f013*f122-f031*f113*f022;

     a0=f011*f022*f033-f011*f023*f032+f021*f013*f032-f031*f013*f022+f031*f012*f023-f021*f012*f033;

p = [ a3 a2 a1 a0];

r = roots( p );
if length(r) == 2
    r = [r; 0];
end

Fvec = [F0(:) F1(:)] *[ones(1,3); transpose(r)];

Fvec = Fvec./ ( ones(9,1)*sqrt(sum( Fvec.^2)));

% Keep real solutions
I = find(not(imag( Fvec(1,:) )));
Fvec = Fvec(:,I);


numSol = size(Fvec,2);
E = complex(zeros(3,3,numSol));
for i = 1 : numSol
    % Denormalise
    E(:,:,i) =  T1' * reshape(Fvec(:,i),3,3) * T2;
end






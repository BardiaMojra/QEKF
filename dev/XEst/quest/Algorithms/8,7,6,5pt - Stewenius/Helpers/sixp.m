function [Evec,errvec] = sixp( x1,x2) 
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
EE = V(:,7:9);

E0 = reshape( EE(:,1),3,3);
E1 = reshape( EE(:,2),3,3);
E2 = reshape( EE(:,3),3,3);

M = buildMsixp( E0,E1,E2);

if 0 
	addpath ~dnister/henrik/minimal_cases/matlab/common_tools/
	addpath e:/minimal_cases/matlab/common_tools/
	MONS =  build_MONS( 5 );
	I = find( and( MONS(1,:) + MONS(2,:) <= 3  ,  MONS( 3,:) < 3 ) ) ;
	MONS = MONS(:,I);
  
	I = find( not(MONS(3,:)));
  
	MONS = MONS(:,I);
end

MONS=[3 2 1 0 2 1 0 1 0 0
      0 1 2 3 0 1 2 0 1 0;];
M=M(:,[12 13 14 15 21 22 23 27 28 30]);

% %%Slänger determinantekvationen då denna är rent 9 i det planöra fallet.
%  M= M(1:9, :) ;  
  
 %Först provar vi utan att välja med hjälp av svd
 if 0 
    M = M(1:4,:) ;
 else
	 [U,S,V] = svd( M ) ;
    M = V(:,1:4)' ;
 end
 
 
 MONS = MONS(1:2,:) ;
 
 
 M = inv(M(1:4,1:4)) *M ;
 
 M(1:4,1:4 ) = eye(4); 
 
 NN = [ -M(1:3,5:end);
    1 0 0 0 0 0 ;
    0 1 0 0 0 0 ; 
    0 0 0 1 0 0 ; ];
 
 [V,D] = eig( NN ) ;
 
 l = V(4:5,:)./(([1;1]*V(end,:)));

 
 tr = inline( 'sum(diag(x))' );
 Evec =zeros(9,6);
 errvec = zeros(1,6);
 for i=1:6
   E = E0 + l(1,i)*E1 + l(2,i)*E2;
   E = E/norm(E(:));
   Evec(:,i) = E(:);
	errvec(i)=norm( 2*E*transpose(E)*E - tr( E*transpose(E))*E);
 end
 
 
 [errvec , I ] = sort( errvec );
 Evec = Evec(:,I);

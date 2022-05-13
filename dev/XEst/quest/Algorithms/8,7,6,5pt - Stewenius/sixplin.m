function E = sixplin( x1,x2) 
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


MONS=[3 2 1 0 2 1 0 1 0 0
      0 1 2 3 0 1 2 0 1 0;];
M=M(:,[12 13 14 15 21 22 23 27 28 30]);

[U,S,V] = svd( M );
V = V((end-2):(end-1),end)/V(end,end);

E = E0 + V(1)*E1 + V(2)*E2;

Evec = E(:) / norm(E(:));


numSol = size(Evec,2);
E = zeros(3,3,numSol);
for i = 1 : numSol
    E(:,:,i) =  reshape(Evec(:,i),3,3);
end


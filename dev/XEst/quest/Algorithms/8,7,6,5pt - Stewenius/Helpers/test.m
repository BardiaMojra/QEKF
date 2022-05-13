[Q,R] = qr( rand(3));
P1 = [eye(3) rand(3,1)];
P2 = [Q      rand(3,1)];
%%X = [rand(2,6) ;ones(2,6)];
X = [rand(3,6) ;ones(1,6)];
x1 = P1*X;
x2 = P2*X;
[Evec1 , errvec] = sixp( x1,x2);
[Evec2 , errvec2] = sixp_pizarro( x1,x2);
errvec
errvec2


E =  reshape( Evec2(:,1),3,3);
det( E )

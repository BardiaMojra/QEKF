function M = buildMsixp(F0,F1,F2)
%%Jag har F0 F1 F2


F = cell(3,3);
Ft = cell(3,3);
P = cell(3,3);
for row = 1 : 3
    for col = 1 : 3
        F{row,col} = [zeros(1,26) F1(row,col) F2(row,col) 0 F0(row,col)];
        Ft{col,row} = [zeros(1,26) F1(row,col) F2(row,col) 0 F0(row,col)];
        P{row,col} = zeros(1,30);
    end
end

P{1,1} = [zeros(1,29) 1];
P{2,2} = [zeros(1,29) 1];
P{3,3} = [zeros(1,28) 1 0];

% Ft = transpose(F);
temp = mult_mat(F,Ft);

tr = temp{1,1}+temp{2,2}+temp{3,3};


t1 =  mult_mat(temp, F) ;
t2 = mult_mat( 0.5*tr, F );
EQ = cell(3,3);
for row = 1 : 3
    for col = 1 : 3
        EQ{row,col} = t1{row,col}- t2{row, col};
    end
end
%EQ1 = 2*F*P*transpose(F)*P*F - tr( P*F*P*transpose( F))*F;


M = zeros(10,30);
for i = 1 : 9
    M(i,:) = EQ{i};
end
M = 2 * M;

%%Nu hade det ju varit trevligt att ha determinanten med! 

f11 = F{1,1};
f12 = F{1,2};
f13 = F{1,3};
f21 = F{2,1};
f22 = F{2,2};
f23 = F{2,3};
f31 = F{3,1};
f32 = F{3,2};
f33 = F{3,3};


% m = inline( 'mult_atom(mult_atom(x1,x2),x3)','x1','x2','x3');
% temp = m(f11,f22,f33)-m(f11,f23,f32)-m(f21,f12,f33)+m(f21,f13,f32)+m(f31,f12,f23)-m(f31,f13,f22);

temp = mult_atom(mult_atom(f11,f22),f33) - ...
    mult_atom(mult_atom(f11,f23),f32) - ...
    mult_atom(mult_atom(f21,f12),f33) + ...
    mult_atom(mult_atom(f21,f13),f32) + ...
    mult_atom(mult_atom(f31,f12),f23) - ...
    mult_atom(mult_atom(f31,f13),f22);

M(10,:) = temp;

function [v,depth] = Find_LinearVelocity(m, m_dot, omega)

    Identity_Matrix     = eye(3); 
    Num_Points          = size(m,2); 
    Num_Candidates      = size(omega,2);
    v                   = zeros(3,Num_Candidates);              % linear velocity candidates 
    u                   = zeros(Num_Points,Num_Candidates);     % depth candidates 
    u_dot               = zeros(Num_Points,Num_Candidates);     % depth derivative candidates 

    for k = 1 : Num_Candidates

        omega_skew = [0             -omega(3,k)     omega(2,k); 
                      omega(3,k)    0               -omega(1,k); 
                      -omega(2,k)   omega(1,k)      0           ]; 

        % build C matrix, size: 15x13
        C = zeros(3*Num_Points, 2*Num_Points+3); 
        
        for i = 1 : Num_Points 
            C((i-1)*3+1:i*3, 1:3) = Identity_Matrix; 
            C((i-1)*3+1:i*3, (i-1)*2+4:(i-1)*2+5) = [omega_skew*m(:,i)-m_dot(:,i), -m(:,i)]; 
        end
        
        % null space of C 
        [~, ~, V] = svd(C); 
    
        % first three rows of last column of V are v_x, v_y, v_z 
        v_temp = V(1:3, end); 
        depth_temp = V(4:13,end); 
        
        v(:,k) = v_temp; 
        depth(:,k) = depth_temp; 

    end 
    
    
    
end 



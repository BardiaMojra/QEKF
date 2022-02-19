function [v,omega] = PoCo(m,m_dot)
    
    index_m_mdot = randsample(length(m),6); 
        
    m6 = m(:, index_m_mdot); 
    m6_dot = m_dot(:, index_m_mdot); 
    
    m5 = m6(:, [1:5]); 
    m5_dot = m6_dot(:, [1:5]); 
    
    A = Find_A_Matrix(m5, m5_dot);          % size of A: 30x34, A is coefficient matrix 

    %% case 1 
    B1 = Find_B_Matrix(A,1);                             % size of B: 20x20, B is used for solving eigenvectors 
    [omega1, lamda1] = Find_AngularVelocity(B1,1);       % find angular velocities, 20 solution candidates 

    res1 = Residu(m6, m6_dot, omega1);                     % find the best result out of 20 solutions 
    [min_val1, min_idx1] = min( abs(res1) ); 

    [v1,depth1] = Find_LinearVelocity(m5, m5_dot, omega1(:,min_idx1));     % find linear velocity and depth 

    %% case 2 
    B2 = Find_B_Matrix(A,2);                             % size of B: 20x20, B is used for solving eigenvectors 
    [omega2, lamda2] = Find_AngularVelocity(B2,2);       % find angular velocities, 20 solution candidates 

    res2 = Residu(m6, m6_dot, omega2);                     % find the best result out of 20 solutions 
    [min_val2, min_idx2] = min( abs(res2) ); 

    [v2,depth2] = Find_LinearVelocity(m5, m5_dot, omega2(:,min_idx2));     % find linear velocity and depth 

    %% case 3 
    B3 = Find_B_Matrix(A,3);                             % size of B: 20x20, B is used for solving eigenvectors 
    [omega3, lamda3] = Find_AngularVelocity(B3,3);       % find angular velocities, 20 solution candidates 

    res3 = Residu(m6, m6_dot, omega3);                     % find the best result out of 20 solutions 
    [min_val3, min_idx3] = min( abs(res3) ); 

    [v3,depth3] = Find_LinearVelocity(m5, m5_dot, omega3(:,min_idx3));     % find linear velocity and depth 

    %% case 4 
    B4 = Find_B_Matrix(A,4);                             % size of B: 20x20, B is used for solving eigenvectors 
    [omega4, lamda4] = Find_AngularVelocity(B4,4);       % find angular velocities, 20 solution candidates 

    res4 = Residu(m6, m6_dot, omega4);                     % find the best result out of 20 solutions 
    [min_val4, min_idx4] = min( abs(res4) ); 

    [v4,depth4] = Find_LinearVelocity(m5, m5_dot, omega4(:,min_idx4));     % find linear velocity and depth 

    %% case 5 
    B5 = Find_B_Matrix(A,5);                             % size of B: 20x20, B is used for solving eigenvectors 
    [omega5, lamda5] = Find_AngularVelocity(B5,5);       % find angular velocities, 20 solution candidates 

    res5 = Residu(m6, m6_dot, omega5);                     % find the best result out of 20 solutions 
    [min_val5, min_idx5] = min( abs(res5) ); 

    [v5,depth5] = Find_LinearVelocity(m5, m5_dot, omega5(:,min_idx5));     % find linear velocity and depth 

    %% case 6 
    B6 = Find_B_Matrix(A,6);                             % size of B: 20x20, B is used for solving eigenvectors 
    [omega6, lamda6] = Find_AngularVelocity(B6,6);       % find angular velocities, 20 solution candidates 

    res6 = Residu(m6, m6_dot, omega6);                     % find the best result out of 20 solutions 
    [min_val6, min_idx6] = min( abs(res6) ); 

    [v6,depth6] = Find_LinearVelocity(m5, m5_dot, omega6(:,min_idx6));     % find linear velocity and depth 

    %% results 
    omega1_ans = omega1(:,min_idx1); 
    % v1 
    % 
    omega2_ans = omega2(:,min_idx2); 
    % v2
    % 
    omega3_ans = omega3(:,min_idx3); 
    % v3
    % 
    omega4_ans = omega4(:,min_idx4); 
    % v4
    % 
    omega5_ans = omega5(:,min_idx5); 
    % v5
    % 
    omega6_ans = omega6(:,min_idx6); 
    % v6

    [val, idx] = min([min_val1, min_val2, min_val3, min_val4, min_val5, min_val6]); 

    if idx == 1 
        v = v1; 
        omega = omega1_ans; 
    elseif idx == 2 
        v = v2; 
        omega = omega2_ans; 
    elseif idx == 3 
        v = v3; 
        omega = omega3_ans; 
    elseif idx == 4 
        v = v4; 
        omega = omega4_ans; 
    elseif idx == 5 
        v = v5; 
        omega = omega5_ans; 
    elseif idx == 6 
        v = v6; 
        omega = omega6_ans; 
    end 

end 



function residu = Residu(m, m_dot, omega)

    C0 = Find_C_Matrix(m, m_dot);     % size of C0: 10x20
    
    omega_x = omega(1,:); 
    omega_y = omega(2,:); 
    omega_z = omega(3,:); 
    
    xVec = [omega_x.^3;         % size of xVec: 20x20
        omega_y.^3; 
        omega_z.^3; 
        omega_x.^2.*omega_y; 
        omega_x.^2.*omega_z; 
        omega_y.^2.*omega_x; 
        omega_y.^2.*omega_z; 
        omega_z.^2.*omega_x; 
        omega_z.^2.*omega_y; 
        omega_x.*omega_y.*omega_z; 
        omega_x.^2; 
        omega_y.^2; 
        omega_z.^2; 
        omega_x.*omega_y; 
        omega_x.*omega_z; 
        omega_y.*omega_z; 
        omega_x; 
        omega_y; 
        omega_z; 
        ones(1,20) ]; 
    
    residuMat = C0 * xVec;      % size of residuMat: 10x20 
    
    residu = sum( abs(residuMat), 1 ); 
    

end 



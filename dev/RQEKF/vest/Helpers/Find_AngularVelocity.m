function [omega, lamda] = Find_AngularVelocity(B, number)
    
    switch number 
        
        case 1 
            [e_vector, e_value] = eig(B); % find eigenvectors 
            e_vector_real = real(e_vector); % real parts 
            e_vector_real_norm = e_vector_real ./ (e_vector_real(end, :)); % normalize each eigenvector 

            omega_x = e_vector_real_norm(end-3,:); 
            omega_y = e_vector_real_norm(end-2,:); 
            omega_z = e_vector_real_norm(end-1,:); 

            omega = [omega_x; 
                     omega_y; 
                     omega_z]; 

            lamda = diag(e_value); 
            
        case 2 
            [e_vector, e_value] = eig(B); 
            e_vector_real = real(e_vector); 
            e_vector_real_norm = e_vector_real ./ (e_vector_real(end, :)); % normalize each eigenvector 

            omega_x = e_vector_real_norm(end-3,:); 
            omega_y = e_vector_real_norm(end-2,:); 
            omega_z = e_vector_real_norm(end-1,:); 

            omega = [omega_x; 
                     omega_y; 
                     omega_z]; 

            lamda = diag(e_value);   
        
        case 3 
            [e_vector, e_value] = eig(B); 
            e_vector_real = real(e_vector); 
            e_vector_real_norm = e_vector_real ./ (e_vector_real(end, :)); % normalize each eigenvector 

            omega_x = e_vector_real_norm(end-2,:); 
            omega_y = e_vector_real_norm(end-3,:); 
            omega_z = e_vector_real_norm(end-1,:); 

            omega = [omega_x; 
                     omega_y; 
                     omega_z]; 

            lamda = diag(e_value);  
            
        case 4 
            [e_vector, e_value] = eig(B); 
            e_vector_real = real(e_vector); 
            e_vector_real_norm = e_vector_real ./ (e_vector_real(end, :)); % normalize each eigenvector 

            omega_x = e_vector_real_norm(end-2,:); 
            omega_y = e_vector_real_norm(end-3,:); 
            omega_z = e_vector_real_norm(end-1,:); 

            omega = [omega_x; 
                     omega_y; 
                     omega_z]; 

            lamda = diag(e_value); 
            
        case 5 
            [e_vector, e_value] = eig(B); 
            e_vector_real = real(e_vector); 
            e_vector_real_norm = e_vector_real ./ (e_vector_real(end, :)); % normalize each eigenvector 

            omega_x = e_vector_real_norm(end-2,:); 
            omega_y = e_vector_real_norm(end-1,:); 
            omega_z = e_vector_real_norm(end-3,:); 

            omega = [omega_x; 
                     omega_y; 
                     omega_z]; 

            lamda = diag(e_value); 
            
        case 6 
            [e_vector, e_value] = eig(B); 
            e_vector_real = real(e_vector); 
            e_vector_real_norm = e_vector_real ./ (e_vector_real(end, :)); % normalize each eigenvector 

            omega_x = e_vector_real_norm(end-2,:); 
            omega_y = e_vector_real_norm(end-1,:); 
            omega_z = e_vector_real_norm(end-3,:); 

            omega = [omega_x; 
                     omega_y; 
                     omega_z]; 

            lamda = diag(e_value); 


    end % end switch 
    
end % end function 



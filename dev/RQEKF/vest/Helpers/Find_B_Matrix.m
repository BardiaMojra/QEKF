function B = Find_B_Matrix(A, number)

    switch number 
        
        case 1 %% case 1: lamda = omega_y / omega_x 
            
            % A1 and A2 matrix
            index_A1 = [1 4 5 6 8 10 11 13 14 15 16 19 20 21 23 25 26 29 30 32];    % at least one omega_x 
            index_A2 = [2 3 7 9 12 17 18 22 24 27 28 31 33 34];                     % without omega_x 

            A1 = A(:, index_A1);        % size of A1: 30x20 
            A2 = A(:, index_A2);        % size of A2: 30x14 

            % Bbar matrix 
            Bbar = -pinv(A2)*A1;        % size of Bbar: 14x20 

            % B matrix 
            index_B_ones = [1 2;        % rows of B, which only have ones and zeros 
                            2 6; 
                            3 8; 
                            6 4; 
                            7 10; 
                            8 9; 
                            11 12; 
                            12 14; 
                            13 16; 
                            17 18]; 

            index_B_Bbar = [4 1;        % rows of B, which have part of Bbar rows 
                            5 4; 
                            9 3; 
                            10 5; 
                            14 6; 
                            15 9; 
                            16 8; 
                            18 10; 
                            19 12; 
                            20 13]; 

            B = zeros(20);      % size of B: 20x20 

            % fill ones and zeros into B 
            for i = 1:10
                B(index_B_ones(i,1), index_B_ones(i,2)) = 1; 
            end 

            % fill certain rows of Bbar into B 
            B(index_B_Bbar(:,1),:) = Bbar(index_B_Bbar(:,2),:); 
            
        
        case 2 %% case 2: lamda = omega_z / omega_x 
            
            % A1 and A2 matrix
            index_A1 = [1 4 5 6 8 10 11 13 14 15 16 19 20 21 23 25 26 29 30 32];    % at least one omega_x 
            index_A2 = [2 3 7 9 12 17 18 22 24 27 28 31 33 34];                     % without omega_x 

            A1 = A(:, index_A1);        % size of A1: 30x20 
            A2 = A(:, index_A2);        % size of A2: 30x14 

            % Bbar matrix 
            Bbar = -pinv(A2)*A1;        % size of Bbar: 14x20 

            % B matrix 
            index_B_ones = [1 3; 
                            2 8; 
                            3 7; 
                            6 9; 
                            7 5; 
                            8 10; 
                            11 13; 
                            12 16; 
                            13 15; 
                            17 19 ]; 

            index_B_Bbar = [4 3; 
                            5 2; 
                            9 5; 
                            10 4; 
                            14 8; 
                            15 7; 
                            16 9; 
                            18 12; 
                            19 11; 
                            20 14]; 

            B = zeros(20);      % size of B: 20x20 

            % fill ones and zeros into B 
            for i = 1:10
                B(index_B_ones(i,1), index_B_ones(i,2)) = 1; 
            end 

            % fill certain rows of Bbar into B 
            B(index_B_Bbar(:,1),:) = Bbar(index_B_Bbar(:,2),:); 
            
            
        case 3 %% case 3: lamda = omega_x / omega_y 
            
            % A1 and A2 matrix
            index_A1 = [2 4 6 7 9 10 12 13 14 15 17 19 21 22 24 25 27 29 31 33];    % at least one omega_y 
            index_A2 = [1 3 5 8 11 16 18 20 23 26 28 30 32 34];                     % without omega_y 

            A1 = A(:, index_A1);        % size of A1: 30x20 
            A2 = A(:, index_A2);        % size of A2: 30x14 

            % Bbar matrix 
            Bbar = -pinv(A2)*A1;        % size of Bbar: 14x20 

            % B matrix 
            index_B_ones = [1 3; 
                            3 6; 
                            4 9; 
                            6 2; 
                            7 10; 
                            9 8; 
                            11 13; 
                            13 12; 
                            14 16; 
                            17 18]; 

            index_B_Bbar = [2 1; 
                            5 4; 
                            8 3; 
                            10 5; 
                            12 6; 
                            15 9; 
                            16 8; 
                            18 10; 
                            19 12; 
                            20 13]; 

            B = zeros(20);      % size of B: 20x20 

            % fill ones and zeros into B 
            for i = 1:10
                B(index_B_ones(i,1), index_B_ones(i,2)) = 1; 
            end 

            % fill certain rows of Bbar into B 
            B(index_B_Bbar(:,1),:) = Bbar(index_B_Bbar(:,2),:); 
            

        case 4 %% case 4: lamda = omega_z / omega_y 
            
            % A1 and A2 matrix
            index_A1 = [2 4 6 7 9 10 12 13 14 15 17 19 21 22 24 25 27 29 31 33];    % at least one omega_y 
            index_A2 = [1 3 5 8 11 16 18 20 23 26 28 30 32 34];                     % without omega_y 

            A1 = A(:, index_A1);        % size of A1: 30x20 
            A2 = A(:, index_A2);        % size of A2: 30x14 

            % Bbar matrix 
            Bbar = -pinv(A2)*A1;        % size of Bbar: 14x20 

            % B matrix 
            index_B_ones = [1 4; 
                            3 9; 
                            4 7; 
                            6 8; 
                            7 5; 
                            9 10; 
                            11 14; 
                            13 16; 
                            14 15; 
                            17 19]; 

            index_B_Bbar = [2 3; 
                            5 2; 
                            8 5; 
                            10 4; 
                            12 8; 
                            15 7; 
                            16 9; 
                            18 12; 
                            19 11; 
                            20 14]; 

            B = zeros(20);      % size of B: 20x20 

            % fill ones and zeros into B 
            for i = 1:10
                B(index_B_ones(i,1), index_B_ones(i,2)) = 1; 
            end 

            % fill certain rows of Bbar into B 
            B(index_B_Bbar(:,1),:) = Bbar(index_B_Bbar(:,2),:); 
            
            
        case 5 %% case 5: lamda = omega_x / omega_z 
            
            % A1 and A2 matrix
            index_A1 = [3 5 7 8 9 11 12 13 14 15 18 20 22 23 24 25 28 30 31 34];    % at least one omega_z 
            index_A2 = [1 2 4 6 10 16 17 19 21 26 27 29 32 33];                     % without omega_z 

            A1 = A(:, index_A1);        % size of A1: 30x20 
            A2 = A(:, index_A2);        % size of A2: 30x14 

            % Bbar matrix 
            Bbar = -pinv(A2)*A1;        % size of Bbar: 14x20 

            % B matrix 
            index_B_ones = [1 4; 
                            4 6; 
                            5 10; 
                            6 2; 
                            7 9; 
                            10 8; 
                            11 14; 
                            14 12; 
                            15 16; 
                            17 18]; 

            index_B_Bbar = [2 1; 
                            3 4; 
                            8 3; 
                            9 5; 
                            12 6; 
                            13 9; 
                            16 8; 
                            18 10; 
                            19 12; 
                            20 13]; 

            B = zeros(20);      % size of B: 20x20 

            % fill ones and zeros into B 
            for i = 1:10
                B(index_B_ones(i,1), index_B_ones(i,2)) = 1; 
            end 

            % fill certain rows of Bbar into B 
            B(index_B_Bbar(:,1),:) = Bbar(index_B_Bbar(:,2),:); 
            
            
        case 6 %% case 6: lamda = omega_y / omega_z 
            
            % A1 and A2 matrix
            index_A1 = [3 5 7 8 9 11 12 13 14 15 18 20 22 23 24 25 28 30 31 34];    % at least one omega_z 
            index_A2 = [1 2 4 6 10 16 17 19 21 26 27 29 32 33];                     % without omega_z 

            A1 = A(:, index_A1);        % size of A1: 30x20 
            A2 = A(:, index_A2);        % size of A2: 30x14 

            % Bbar matrix 
            Bbar = -pinv(A2)*A1;        % size of Bbar: 14x20 

            % B matrix 
            index_B_ones = [1 5; 
                            4 10; 
                            5 7; 
                            6 8; 
                            7 3; 
                            10 9; 
                            11 15; 
                            14 16; 
                            15 13; 
                            17 19]; 

            index_B_Bbar = [2 3; 
                            3 2; 
                            8 5; 
                            9 4; 
                            12 8; 
                            13 7; 
                            16 9; 
                            18 12; 
                            19 11; 
                            20 14]; 

            B = zeros(20);      % size of B: 20x20 

            % fill ones and zeros into B 
            for i = 1:10
                B(index_B_ones(i,1), index_B_ones(i,2)) = 1; 
            end 

            % fill certain rows of Bbar into B 
            B(index_B_Bbar(:,1),:) = Bbar(index_B_Bbar(:,2),:); 
            
            
    end % end switch 
    
    
    
end % end function 



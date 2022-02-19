function [Rs, ts] = TransformEssentialsVer2_0(Es)
    

    temp = size(size(Es));
    numberSolutions = 1;
    if temp(1,2) == 3
        temp2 = size(Es);
        numberSolutions = temp2(1,3);
    end
    
    % Preallocate Rs, ts. (Needed for Mex generation)
    Rs = zeros(3,3,2*numberSolutions);
    ts = zeros(3,2,2*numberSolutions);
    
    if numberSolutions == 1
        
        Rs = zeros(3,3,2);
        ts = zeros(3,2,2);

        [U,~,V] = svd(Es);
        W = [0 -1 0; 1 0 0; 0 0 1];

        % Two possibilities for rotation:
        Rs(1:3,1:3) = U * W * V';
        Rs(1:3,4:6) = U * W' * V';

        % Two possibilities for translation:
        ts(1:3,1,1) = U(:,3) ./max(abs(U(:,3)));
        ts(1:3,2,1) = -U(:,3) ./max(abs(U(:,3)));
        ts(1:3,1,2) = U(:,3) ./max(abs(U(:,3)));
        ts(1:3,2,2) = -U(:,3) ./max(abs(U(:,3)));
        
        if( det(Rs(1:3,1:3)) < 0 )
            Rs(1:3,1:3,1) = -Rs(1:3,1:3);
        end
        if( det(Rs(1:3,4:6)) < 0 )
            Rs(1:3,1:3,2) = -Rs(1:3,4:6);
        end
        
    else
        
        Rs_temp = zeros(3,3,2*numberSolutions);
        ts_temp = zeros(3,2,2*numberSolutions);
        index = 0;
        
        for i = 1 : numberSolutions
            
            %Check if there is any Nan
            if ~isnan(Es(1,1,i))
                [U,~,V] = svd(Es(:,:,i));
                W = [0 -1 0; 1 0 0; 0 0 1];
                
                % Two possibilities for rotation and translation:
                index = index + 1;
                Rs_temp( :,:, index ) = U * W * V';
                if(det(Rs_temp( :,:, index )) < 0)
                    Rs_temp( :,:, index ) = -Rs_temp( :,:, index );
                end
                ts_temp(1:3,1,index) = U(:,3) ./max(abs(U(:,3)));
                ts_temp(1:3,2,index) = -U(:,3) ./max(abs(U(:,3)));
                
                index = index + 1;
                
                Rs_temp( :,:, index ) = U * W' * V';
                if(det(Rs_temp( :,:, index )) < 0)
                    Rs_temp( :,:, index ) = -Rs_temp( :,:, index );
                end
                ts_temp(1:3,1,index) = U(:,3) ./max(abs(U(:,3)));
                ts_temp(1:3,2,index) = -U(:,3) ./max(abs(U(:,3)));
                
            end
        end
        
        Rs = Rs_temp(:,:,1:index);
        ts = ts_temp(:,:,1:index);
    end
                
end

function [m,m_dot] = Find_M_and_Mdot(Image_Path,Image_1_index,Image_2_index, surfThresh)

    Image_Path_1 = strcat(Image_Path,Image_1_index);    
    Image_Path_2 = strcat(Image_Path,Image_2_index);    

    Image_1 = imread(Image_Path_1);     % image 1 
    Image_2 = imread(Image_Path_2);     % image 2 
    
%     SURF_point_1 = detectSURFFeatures(Image_1, 'MetricThreshold', surfThresh); 
%     SURF_point_2 = detectSURFFeatures(Image_2, 'MetricThreshold', surfThresh); 
%     
%     [f1,vp1] = extractFeatures(Image_1,SURF_point_1); 
%     [f2,vp2] = extractFeatures(Image_2,SURF_point_2); 
%     
%     [indexPairs, mmetric] = matchFeatures(f1, f2, ...
%                                             'MaxRatio',0.3, ...
%                                             'MatchThreshold',.1, ...
%                                             'Unique',true); 
%                                         
%     matchedPoints1 = vp1(indexPairs(:,1)); 
%     matchedPoints2 = vp2(indexPairs(:,2)); 
    
    % corner features 
    Corner_1 = detectHarrisFeatures(Image_1);   % corner features 
    Corner_2 = detectHarrisFeatures(Image_2);   % corner features 
    
    [features1,valid_points1] = extractFeatures(Image_1,Corner_1.selectStrongest(300)); 
    [features2,valid_points2] = extractFeatures(Image_2,Corner_2.selectStrongest(300)); 
    
    indexPairs = matchFeatures(features1,features2);    % matched-corner indexes 
    
    matchedPoints1 = valid_points1(indexPairs(:,1),:);  % matched corner in image1 
    matchedPoints2 = valid_points2(indexPairs(:,2),:);  % matched corner in image2 

    % number of corner points 
    num_points = length(indexPairs); 
    
    % corner points in current and next frame 
    uv_current = zeros(2, num_points);       % all corner points in current frame 
    uv_next = zeros(2, num_points);          % all corner points in next frame 
    
    for i = 1:num_points 

        pt_current = matchedPoints1.Location(i,:); 
        pt_next = matchedPoints2.Location(i,:); 

        uv_current(:,i) = pt_current; 
        uv_next(:,i) = pt_next; 

    end 
    
    uv_dot = uv_next - uv_current; 
    
%     figure(1)
%     imshow(Image_1); 
%     
%     figure(2) 
%     imshow(Image_2); 
%     
%     figure(3) 
%     quiver(uv_current(1,:), uv_current(2,:), uv_dot(1,:), uv_dot(2,:), 'r')
%     
%     figure(4)
%     imshow(Image_1); hold on; 
%     quiver(uv_current(1,:), uv_current(2,:), uv_dot(1,:), uv_dot(2,:), 'r'); 
        
    uv_current_1 = [uv_current; ones(1,num_points)]; % append ones at bottom 
    uv_next_1 = [uv_next; ones(1,num_points)]; % append ones at bottom 
    
    % camera calibration matrix 
    K_00 = [984     0       696; 
            0       981     256;
            0       0       1   ]; 
%     K_01 = [990 0 702; 0 988 246; 0 0 1]; 
%     K_02 = [960 0 696; 0 957 224; 0 0 1]; 
%     K_03 = [904 0 696; 0 902 224; 0 0 1]; 
    
    % corner points in image coordinate 
    xy_current = inv(K_00) * uv_current_1; 
    xy_next = inv(K_00) * uv_next_1; 
    
    % results: m and mdot 
    m = xy_current; 
    m_dot = xy_next - xy_current;     
    
%     figure(1)
%     quiver(m(1,:), m(2,:), m_dot(1,:), m_dot(2,:), 'r')
    
end 



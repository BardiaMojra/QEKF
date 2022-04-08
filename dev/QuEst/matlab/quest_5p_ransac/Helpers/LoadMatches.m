function matches = LoadMatches(i)
%LOADMATCHES loads matched keypoints from file.
%   This module is used in debugging to test and compare computational
%   integrity.

% load matched keypoints in pixels (padded with row of 1s)
path = ['./out/KITTI/feature_matches/matches_dat_p11p21_' num2str(i, '%02d') '.txt'];
p11p21 = readmatrix(path,'Delimiter',' ');
p1 = p11p21(:,1:2)';
p2 = p11p21(:,4:5)';

% load matched keypoints in unit frame lengths
path = ['./out/KITTI/feature_matches/matches_dat_m1m2_' num2str(i, '%02d') '.txt'];
m1m2 = readmatrix(path,'Delimiter',' ');
m1 = m1m2(:,1:3)';
m2 = m1m2(:,4:6)';

AA = sqrt(sum(m1.^2,1));
BB = sqrt(sum(m2.^2,1));
% unit norm coordinates
m1u = bsxfun(@rdivide, m1, AA); 
m2u = bsxfun(@rdivide, m2, BB);

matches.p1 = p1;
matches.p2 = p2;
matches.m1 = m1;
matches.m2 = m2;
matches.m1u = m1u;
matches.m2u = m2u;  
matches.numPts = length(m1(1,:)); 

end




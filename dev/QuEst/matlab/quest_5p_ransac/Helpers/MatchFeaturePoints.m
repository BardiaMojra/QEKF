function matches = MatchFeaturePoints(Ip,ppoints, In,npoints, maxPts, dataset, i)

% Extract feature points
[f1,vp1] = extractFeatures(Ip,ppoints);
[f2,vp2] = extractFeatures(In,npoints);   

% Match feature points
[indexPairs, mmetric] = matchFeatures(f1, f2, 'MaxRatio',0.7, ...
    'MatchThreshold',1, 'Unique',true);     
matchedPoints1 = vp1(indexPairs(:,1));
matchedPoints2 = vp2(indexPairs(:,2)); 

p1 = matchedPoints1.Location;
p2 = matchedPoints2.Location;     
% figure; im = showMatchedFeatures(Ip,In,p1,p2);
% outpath = ['./out/KITTI/feature_matches/' num2str(i, '%02d') '.png'];
% imwrite(im, outpath);
% Feature points
numMatch = size(p1, 1);  % Number of matched feature points
numPts = min(numMatch, maxPts);
p1 = p1(1:numPts, :);
p2 = p2(1:numPts, :);
p11 = double( [p1 ones(numPts, 1)].' );
p21 = double( [p2 ones(numPts, 1)].' );
% Point coordinates on image plane
m1 = dataset.K \ p11;
m2 = dataset.K \ p21 ;

% Unit norm coordinates
m1u = bsxfun(@rdivide, m1, sqrt(sum(m1.^2,1))); 
m2u = bsxfun(@rdivide, m2, sqrt(sum(m2.^2,1)));

% % Display images with matched feature points    
% imshow(In);
% hold on;
% plot(p1(:,1), p1(:,2), 'g+');
% plot(p2(:,1), p2(:,2), 'yo');    
% for j = 1 : numPts
%     plot([p1(j,1) p2(j, 1)], [p1(j,2) p2(j, 2)], 'g');
% end
% drawnow;    
% hold off;

matches.p1 = p1;
matches.p2 = p2;
matches.m1 = m1;
matches.m2 = m2;
matches.m1u = m1u;
matches.m2u = m2u;  
matches.numPts = numPts; 

end


function matches = MatchFeaturePoints(Ip, ...
                                      ppoints, ...
                                      In, ...
                                      npoints, ...
                                      maxPts, ...
                                      dataset, ...
                                      kfi,...
                                      mat_feat_disp_fig_en, ...
                                      mat_feat_sav_fig_en, ...
                                      toutDir, ...
                                      mat_feat_sav_pts_en)

  kf_tag = strcat("kf", num2str(kfi,"%05d"));
  
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
  % Feature points
  numMatch = size(p1, 1);  % Number of matched feature points
  numPts = min(numMatch, maxPts);
  p1 = p1(1:numPts, :);
  p2 = p2(1:numPts, :);
  % pad with a col of 1s and transpose
  p11 = double( [p1 ones(numPts, 1)].' );
  p21 = double( [p2 ones(numPts, 1)].' );
  % Point coordinates on image plane
  m1 = dataset.K \ p11;
  m2 = dataset.K \ p21 ;
  % Unit norm coordinates
  m1u = bsxfun(@rdivide, m1, sqrt(sum(m1.^2,1))); 
  m2u = bsxfun(@rdivide, m2, sqrt(sum(m2.^2,1)));
   
  if mat_feat_disp_fig_en || mat_feat_sav_fig_en
    fig = figure();
    imshow(In);
    title(kf_tag);
    hold on;
    plot(p1(:,1), p1(:,2), 'g+');
    plot(p2(:,1), p2(:,2), 'ro');    
    for j = 1 : numPts
        plot([p1(j,1) p2(j, 1)], [p1(j,2) p2(j, 2)], 'm');
    end
    drawnow;  
    lg  = legend(["$kf(i-1)$", "$kf(i)$"], "Interpreter", "latex"); 
    lg.Position(1:2) = [1 1];
    lg.FontSize = 12;  
    hold off;
    if mat_feat_sav_fig_en
      fname = strcat(toutDir,'fig_mat-feat_',kf_tag,'.png');
      saveas(fig, fname);
    end
    if mat_feat_disp_fig_en
      waitforbuttonpress;
    end 
    close(fig);
  end
    
  if mat_feat_sav_pts_en % save the matched data points in pixels (padded with row of 1s)
    dat = cat(2,p11',p21');
    fname = strcat(toutDir, 'pts_mat-feat_',kf_tag,'.txt');
    writematrix(dat,fname,'Delimiter',' ') 
  end
  matches.p1 = p1;
  matches.p2 = p2;
  matches.m1 = m1;
  matches.m2 = m2;
  matches.m1u = m1u;
  matches.m2u = m2u;  
  matches.numPts = numPts; 
  end


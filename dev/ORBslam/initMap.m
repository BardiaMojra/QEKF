function initMap(tkr, dat, pft, cfg, cam)
  
  % Map initialization loop
  while ~tkr.isMapInitd && tkr.idx_currFr < numel(dat.imds.Files)
    
    tkr.currI = readimage(dat.imds, tkr.idx_currFr);
    [tkr.currFts, tkr.currPts] = helperDetectAndExtractFeatures(tkr.currI, ...
      pft.scaleFactor, pft.numLevels, pft.numPoints);   
    tkr.idx_currFr = tkr.idx_currFr + 1;
  
    % Find putative feature matches
    tkr.idxPairs = matchFeatures(tkr.prevFts, tkr.currFts, 'Unique', true, ...
      'MaxRatio', 0.9, 'MatchThreshold', 40);
    preMatchedPoints  = tkr.prevPts(tkr.idxPairs(:,1),:);
    currMatchedPoints = tkr.currPts(tkr.idxPairs(:,2),:);
    
    if size(tkr.idxPairs, 1) < pft.minMatches
      continue
    end
    preMatchedPoints  = tkr.prevPts(tkr.idxPairs(:,1),:);
    currMatchedPoints = tkr.currPts(tkr.idxPairs(:,2),:);
    % Compute homography and evaluate reconstruction
    [tformH, scoreH, inliersIdxH] = helperComputeHomography(preMatchedPoints, currMatchedPoints);
    % Compute fundamental matrix and evaluate reconstruction
    [tformF, scoreF, inliersIdxF] = helperComputeFundamentalMatrix(preMatchedPoints, currMatchedPoints);
    % Select the model based on a heuristic
    ratio = scoreH/(scoreH + scoreF);

    if ratio > cfg.ratioThreshold
      inlierTformIdx = inliersIdxH;
      tform          = tformH;
    else
      inlierTformIdx = inliersIdxF;
      tform          = tformF;
    end
  
    % Computes the camera location up to scale. Use half of the 
    % points to reduce computation
    inlierPrePoints  = preMatchedPoints(inlierTformIdx);
    inliertkr.currPts = currMatchedPoints(inlierTformIdx);
    [relOrient, relLoc, validFraction] = relativeCameraPose(tform, cam.ins, ...
      inlierPrePoints(1:2:end), inliertkr.currPts(1:2:end));
    
    % If not enough inliers are found, move to the next frame
    if validFraction < 0.9 || numel(size(relOrient))==3
      continue
    end
    
    % Triangulate two views to obtain 3-D map points
    tkr.relPose = rigid3d(relOrient, relLoc);
    
    [isValid, tkr.xyzWorldPoints, inlierTriangulationIdx] = helperTriangulateTwoFrames(...
      rigid3d, tkr.relPose, inlierPrePoints, inliertkr.currPts, cam.ins, cfg.minParallax);
    if ~isValid
      continue
    end
    % Get the original index of features in the two key frames
    tkr.idxPairs = tkr.idxPairs(inlierTformIdx(inlierTriangulationIdx),:);
    tkr.isMapInitd = true;
    disp(['Map initialized with frame 1 and frame ', num2str(tkr.idx_currFr-1)])
  end % End of map initialization loop

end 
function check_initMap(tkr)
  if tkr.isMapInitd
    % Show matched features
    tkr.hfeature = showMatchedFeatures(tkr.firstI, tkr.currI, tkr.prevPts(tkr.idxPairs(:,1)), ...
      tkr.currPts(tkr.idxPairs(:, 2)), 'Montage');
  else
      error('Unable to initialize map.')
  end
end

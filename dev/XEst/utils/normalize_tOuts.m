function tOuts = normalize_tOuts(tOuts)
  for ts = 1:size(tOuts,3)
    for t = 1:size(tOuts,2)
      %disp(tOuts(:,t,ts));
      tOuts(:,t,ts) = normalizeVec(tOuts(:,t,ts));
      %disp("normed: ");
      %disp(tOuts(:,t,ts));
    end
  end
 

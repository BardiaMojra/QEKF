function normed = normalizeVec(vec)
  normThresh  = 1.5;
  assert(isequal(size(vec),[3,1])||isequal(size(vec),[4,1]), ...
    "[normalizeVec]-->> not a col vec")
  normed  = vec ./ sqrt(sum(vec.*vec)); % normalize v answer 
  if all(normed < 0) 
      normed = - normed; 
  end 
  if isequal(size(vec),[4,1]) && normed(1) < 0 
      normed = - normed; 
  end 
  
  %disp(vec);
  %disp(sqrt(sum(vec.*vec)));
  assert(sqrt(sum(vec.*vec))<normThresh,"[normalizeVec]->> vec not normalized properly!!"  );
  %disp("normed");
  %disp(normed);
  %;
  

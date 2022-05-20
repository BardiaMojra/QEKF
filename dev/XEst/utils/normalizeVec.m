function normed = normalizeVec(vector)
  normed  = vector/ vecnorm(vector); % normalize v answer 
  if normed(3) < 0 
      normed = - normed; 
  end 
  

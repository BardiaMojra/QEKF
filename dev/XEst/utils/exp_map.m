function Qwxyz = exp_map(W)
  if isequal(size(W), [3,1]) 
    norm_w = norm(W);
    if norm_w == 0
      Qwxyz = [1; 0; 0; 0];
    else
      qxyz = sin(norm_w/2) .* W/norm_w;
      qw = cos(norm_w/2);
      Qwxyz = [qw; qxyz(1); qxyz(2); qxyz(3)];
    end
  else
    disp(W);
    assert(false, "[exp_map]--> W size is not [3 1]! ");
  end
%end % Qxyzw = exp_map(W)
function Qs = check_quats(Qs) % --> in: 4xN
  %disp(Qs);
  %disp(size(Qs));
  Qs = normalize_all(Qs);
  assert(isequal(size(Qs,1),4), "[check_quats]-->> Qs has wrong dimensions!")
  for i = 1:size(Qs,2)
    Qs(:,i) = check_quat_mag(Qs(:,i));
  end
  
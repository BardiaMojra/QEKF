function Q = check_quat_mag(Q)
  assert(isequal(size(Q), [4,1]), "[check_quat_mag]-->> Q dims is NOT [4,1]!")
  %disp(Q);
  if Q(1,1) < 0
    Q = -Q;
  end
  
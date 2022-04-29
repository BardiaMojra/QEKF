%% Metric for nonzero translation error
function err = TransError(t1, t2)

% t1n = t1 / norm(t1);
t1Nrm = sqrt(sum(t1.^2, 1));
t1n = bsxfun(@rdivide, t1, t1Nrm); 

% t2n = t2 / norm(t2);
t2Nrm = sqrt(sum(t2.^2, 1));
t2n = bsxfun(@rdivide, t2, t2Nrm); 


dotErr = sum(bsxfun(@times, t1n, t2n),1);
err = (1/pi) *  acos(dotErr);
err = real(err);

end

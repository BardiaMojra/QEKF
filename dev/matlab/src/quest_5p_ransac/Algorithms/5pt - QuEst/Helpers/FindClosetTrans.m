function tOut = FindClosetTrans(tRef, tIn)

err = TransError(tRef, tIn);

[~,mIdx] = min(abs(err));

tOut = tIn(:,mIdx);

end








































































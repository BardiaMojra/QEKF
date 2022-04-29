function qOut = QuatConj(qBin)

qOut = bsxfun(@times, qBin, [1; -1; -1; -1]);

end
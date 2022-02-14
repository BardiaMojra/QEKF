%% Find groud truth at time "ftime" in the table [times, QarrTru, tarrTru]

function [qInterp, tInterp] = InterpPoseVer1_1(ftime,times, qTru, tTru)


tdiff = times - ftime;
temp = find(tdiff>=0);

idxP = temp(1);
idxN = idxP - 1;

tP = times(idxP);
tN = times(idxN);


qP = qTru(:,idxP);
qN = qTru(:,idxN);

trP = tTru(:,idxP);
trN = tTru(:,idxN);

a = (tP - ftime) / (tP - tN); % line interpolation

qInterp = a * qN + (1-a) * qP;
tInterp = a * trN + (1-a) * trP;









































































































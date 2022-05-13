% Running the benchmarking code on every sequence of KITTI, TUM, ICL, and
% NAIST
%
% (c) Juan-Pablo Ramirez-Paredes, Kaveh Fathian
%
close all
clear

% Folders which contain the required helper functions
addpath('Helpers')

%% KITTI
% http://www.cvlibs.net/datasets/kitti/eval_odometry.php.
benchtype = 'KITTI';
for seq = 0 : 10
    close all
    benchnum = seq;
    Benchmark_Ver1_5_1;
%     savename = [benchtype num2str(seq, '%02d') '.mat'];
%     save(savename);
end

%% TUM
% https://vision.in.tum.de/data/datasets/rgbd-dataset
benchtype = 'TUM';
snums = 1 : 10;
snums([3 9 10]) = []; % Problematic sequences: 3, 9, 10 (data length does not match)
for seq = snums
    close all
    benchnum = seq;
    Benchmark_Ver1_5_1;
%     savename = [benchtype num2str(seq, '%02d') '.mat'];
%     save(savename);
end

%% ICL
% https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html
benchtype = 'ICL';
for seq = 0 : 3
    close all
    benchnum = seq;
    Benchmark_Ver1_5_1;
%     savename = [benchtype num2str(seq, '%02d') '.mat'];
%     save(savename);
end

%% NAIST
% http://ypcex.naist.jp/trakmark/sequences/naist.package02/index.html#result
benchtype = 'NAIST';
for seq = [0 12 13]
    close all
    benchnum = seq;
    Benchmark_Ver1_5_1;
%     savename = [benchtype num2str(seq, '%02d') '.mat'];
%     save(savename);
end

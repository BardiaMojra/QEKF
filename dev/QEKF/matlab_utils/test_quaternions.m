



clear
close all


% config
% testDir = 'dead_reckoning_data/1/';
testDir = 'test_001_vicon_training_day/';
dataDir = '/home/smerx/git/QEKF/dev/data/';

rv_f    = strcat(dataDir, testDir, 'imu/rv.txt');
data_rv = importdata(rv_f);    
t = data_rv(:,1); 
% low res data 

%% Run QEKF
% * Author: Bardia Mojra
% * Data: 02/15/2022
% 

clc
close all 
clear 

addpath('utils');

dataDir = [pwd 'datasets'];
%%%%%%%%
% datasets
%  0  'dataset-iphone1_clean'
%  1  'bigC_06-Aug2021'
%  2  'kitti_imu_0926_0001'
%  3  'kitti_imu_0926_0002'
%  4  'kitti_imu_0926_0005'
%  5  'kitti_imu_0926_0018'
%  6  'kitti_imu_0926_0060'
%  7  'kitti_imu_0926_0084'
%  8  'kitti_imu_0926_0113'
%  9  'kitti_imu_0928_0001'
% 10  'Y2021M08D05_ZoomTwistJackal_BigC-off_ransac-off'
% 11  'Y2021M08D05_BoxWalkKuka_BigC-off_ransac-off_Q-Select-on_FP-Last6'
% 12  'Y2021M08D06_BoxWalkKuka_BigC-off_ransac-off_Q-Select-off_FP-HighLow6'
% 13  'Y2021M08D05_CircleAoundMetal_BigC-off_ransac-off'

% _TEST_MODE    = 'all'
TEST_ID      = 12; TEST_MODE = 'single';

if strcmp(TEST_MODE, 'all')
  
elseif strcmp(TEST_MODE, 'all')
  
end




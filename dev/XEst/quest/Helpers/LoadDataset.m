function [dataset, posp] = LoadDataset(dataroot, benchtype, benchnum, ...
  st_frame, end_frame)

dataset.dataroot = dataroot;
dataset.benchtype = benchtype;
dataset.benchnum = benchnum;


%% Path to the database files

if strcmp(benchtype, 'KITTI')
    
    imgpath = [dataroot '/KITTI/sequences/' num2str(benchnum, '%02d') ...
               '/image_0'];
    datapath = [dataroot '/KITTI/poses'];
    
elseif strcmp(benchtype, 'ICL')
    
    imgpath = [dataroot '/ICL/kt' num2str(benchnum) '/rgb'];
    datapath = [dataroot '/ICL/kt' num2str(benchnum)];
    
elseif strcmp(benchtype, 'NAIST')
    
    imgpath = [dataroot '/NAIST/naist' num2str(benchnum, '%03d')];
    datapath = [dataroot, '/NAIST'];
    
elseif strcmp(benchtype, 'TUM')   
    
    tum_names = {'rgbd_dataset_freiburg1_360', ...
    'rgbd_dataset_freiburg1_desk', ...
    'rgbd_dataset_freiburg1_desk2', ...
    'rgbd_dataset_freiburg1_floor', ...
    'rgbd_dataset_freiburg1_room', ...
    'rgbd_dataset_freiburg2_360_hemisphere', ...
    'rgbd_dataset_freiburg2_360_kidnap', ...
    'rgbd_dataset_freiburg2_desk', ...
    'rgbd_dataset_freiburg2_large_no_loop', ...
    'rgbd_dataset_freiburg2_large_with_loop', ...
    'rgbd_dataset_freiburg3_long_office_household'};
    imgpath = [dataroot '/TUM/' tum_names{benchnum} '/rgb'];
    datapath = [dataroot '/TUM/' tum_names{benchnum}];
    
else
    error('Undefined dataset.')
end

dataset.imgpath = imgpath;
dataset.datapath = datapath;


%% Image files
files  = dir(imgpath);
st_frame = st_frame+2; % dir('.') lists . and .. 
if isnan(end_frame)
  end_frame = length(files);
end 

if strcmp(benchtype, 'KITTI')
  fnames = cell(1,end_frame-3);
    for i = st_frame:end_frame
        fnames{i-2} = files(i).name;
    end
elseif strcmp(benchtype, 'NAIST')
    fnames = cell(1,end_frame-3);
    for i = st_frame:end_frame
        fnames{i-2} = files(i).name;
    end
elseif strcmp(benchtype, 'ICL')
    fnames = cell(1,end_frame-3);
    for i = st_frame-2:length(fnames)
        fnames{i} = [num2str(i-1) '.png'];
    end
elseif strcmp(benchtype, 'TUM')
    fnames = cell(1,end_frame-2);
    for i = st_frame:end_frame
        fnames{i-2} = files(i).name;
    end
end
dataset.fnames = fnames;
dataset.st_frame = st_frame;
dataset.end_frame = end_frame;

%% Calibration matrix
if strcmp(benchtype, 'KITTI')
    
    %Use textscan to load the calibration matrix
    calibfile = [dataroot 'KITTI/sequences/' num2str(benchnum, '%02d') ...
               '/calib.txt'];
    fileid = fopen(calibfile);
    C = textscan(fileid, '%s %f %f %f %f %f %f %f %f %f %f %f %f');
    fclose(fileid);
    K = [C{2}(1) C{3}(1) C{4}(1);
         C{6}(1) C{7}(1) C{8}(1);
         C{10}(1) C{11}(1) C{12}(1)];
    dp = zeros(1,3);
    tp = [0 0];
    
elseif strcmp(benchtype, 'ICL')
    
    K = [481.20	0       319.50;
         0      -480.00 239.50;
         0      0       1];
    dp = zeros(1,3);
    tp = [0 0];
    
elseif strcmp(benchtype, 'NAIST')
    
    K = [884.9574219 0           634.0410934;
         0           883.5555857 367.8930972;
         0           0             1        ];
    dp = [-0.2297238599,0.1703723682,-0.09885776185];
    tp = [-0.0002223016917,-0.0003623410498];
    
elseif strcmp(benchtype, 'TUM')
    
    if benchnum <= 5
        K = [517.3          0    318.6;
                 0      516.5    255.3;
                 0          0        1];
        dp = [0.2624 -0.9531 1.1633];
        tp = [-0.0054 0.0026];
    else
        K = [520.9          0    325.1;
                 0      521.0    249.7;
                 0          0        1];
        dp = [0.2312 -0.7849 0.9172];
        tp = [-0.0033 0.0001];
    end
    if benchnum == 11
        K = [535.4          0    320.1;
                 0      539.2    247.6;
                 0          0        1];
        dp = [0.0 0.0 0.0];
        tp = [0.0 0.0];
    end
    
else
    error('Undefined dataset.')
end

camParams = cameraParameters('IntrinsicMatrix', K', ...
                             'NumRadialDistortionCoefficients', 3, ...
                             'RadialDistortion', dp, ...
                             'TangentialDistortion', tp); 

dataset.camParams = camParams;
dataset.K = K;

%% Ground truth

% Read ground truth data
if strcmp(benchtype, 'KITTI')
    
    % The KITTI dataset has exactly one pose per image, so there is no need
    % to interpolate poses
    rawdata = load([datapath '/' num2str(benchnum, '%02d') '.txt']);
    tx = rawdata(:,4);
    ty = rawdata(:,8);
    tz = rawdata(:,12);
    Rmats = zeros(3,3,size(rawdata,1));
    for k = 1 : size(rawdata,1)
        Rmats(:,:,k) = [rawdata(k,1:3); rawdata(k,5:7); rawdata(k,9:11)];
    end
    qTru = R2Q(Rmats);
    tTru = [tx, ty, tz].';    
    
elseif strcmp(benchtype, 'ICL')
    
    rawdata = load([datapath '/traj' num2str(benchnum) '.gt.freiburg']);    
    tTru = rawdata(:,2:4)';
    qTru = [rawdata(:,8) rawdata(:,5:7)]';     
    
elseif strcmp(benchtype, 'NAIST')
    
    datafile = [datapath '/naist' num2str(benchnum, '%03d') '_camerapath.csv'];
    fileid = fopen(datafile);
    C = textscan(fileid, '%s %f %f %f %f %f %f %f %f %f %f %f %f', 'Delimiter', ',', 'CommentStyle', '#');
    fclose(fileid);
    tx = C{5};
    ty = C{9};
    tz = C{13};
    Rmats = zeros(3,3,length(C{1}));
    for k=1:length(C{1})
        Rmats(:,:,k) = [C{2}(k)  C{3}(k)  C{4}(k);
                        C{6}(k)  C{7}(k)  C{8}(k);
                        C{10}(k) C{11}(k) C{12}(k)];
    end
    qTru = R2Q(Rmats);
    tTru = [tx, ty, tz].';    
    
elseif strcmp(benchtype, 'TUM')
    
    % Using textscan instead of textread to deal with comment lines,
    % instead of editing every groundtruth.txt file
    datafile = [datapath '/groundtruth.txt'];
    fileid = fopen(datafile);
    C = textscan(fileid, '%f %f %f %f %f %f %f %f', 'CommentStyle', '#');
    fclose(fileid);
    times = C{1};
    tx = C{2};
    ty = C{3};
    tz = C{4};
    qx = C{5}; 
    qy = C{6}; 
    qz = C{7}; 
    qw = C{8};  % NOTE: In this dataset, instead of first, the last element is cosine of half rotation angle!
    qTru = [qw, qx, qy, qz].'; % Ground truth quaternions in world coord frame    
    tTru = [tx, ty, tz].';     % Ground truth translations in world coord frame
    dataset.times = times;
else
    error('Undefined dataset.')
end

% Make sure the first quaternion element is always positive
negIdx = qTru(1,:) < 0;    
qTru(:,negIdx) = - qTru(:,negIdx);

if strcmp(benchtype, 'TUM')
    % We need to interpolate pose for TUM dataset
    % Initialize with the first frame
    fname = fnames{1};
    ftime = str2double( fname(1:end-4) ); % Time at the current frame  
    [q1, t1] = InterpPoseVer1_1(ftime,times, qTru, tTru); % Interpolate data to find the pose of the current camera frame      
    
else
    % Initial pose
    q1 = qTru(:,1);
    t1 = tTru(:,1);
end

posp.t1 = t1;
posp.q1 = q1;

dataset.qTru = qTru;
dataset.tTru = tTru;























































































































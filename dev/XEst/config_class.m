classdef config_class < matlab.System & dat_class
  properties
    %% test config (constant)
    test_ID     = [];
    outDir      = [pwd '/out/'];
    % test config dependent 
    test_outDir
    %% data config (constant)
    datDir              = [pwd '/data/']; % data dir     
    st_frame          = 1; % start frame index
    end_frame       = nan;% end frame index

    
    %% state machine config
    cntr % key frame counter
    numMethods  % num of algs used for comparison
    Q_err % rot err for each method
    T_err % trans err for each method
    Q % recovered quaternions 
    T % recovered translations
    V
    W 
    V_err
    W_err

    %%QuEst config (constant)
%     QuEst_method               = 'QuEst';
%     QuEst_ransac_thresh   = 1e-6; % ransac, sampson dist thresh
%     QuEst_surf_thresh        = 200; % surf detector thresh
%     QuEst_maxPts               = 30; % max features used in pose est (fewer features, faster compute)
%     QuEst_minPts                = 8; % min feature required (6 to estimate a unique pose from RANSAC, or at least 8 for 8-pt algorithm)
    pose_algorithms      = {...
                                              'EightPt'; 
                                             'Nister'; 
                      %                        'Kneip'; 
                                             'Kukelova'; 
                      %                        'Stewenius'; 
                                             'QuEst'}; % algorithms to run 
    benchmarks      = {...
                                                'KITTI';
                                                'NAIST';
                                                'ICL';
                                                'TUM';
                                                           } % benchmarks ----> (disabled for now)
    
  end
  methods
    % Constructor
    function obj = config_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
      obj = init(obj);
    end
    function obj = init(obj)
      obj.test_outDir = [obj.outDir 'out_' obj.test_ID '/'];
      if not(isfolder(obj.test_outDir))
        disp('test_outDir does NOT exist: ');
        disp(obj.test_outDir);
        pause(5);
        mkdir(obj.test_outDir);
      end 

      obj.dats = dat_class.empty(length(obj.benchmarks), 0);
      for i = 1: obj.numDatasets
          obj.dats(i) = dat_init(obj.datDir, obj.benchmarks(i), obj.seq, obj.st_frame, ...
            obj.end_frame);  

      end

      obj.numMethods        = length(obj.algorithms);             
      obj.Q_err                     = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.T_err                      = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.Q                            = cell(obj.numKeyFrames,obj.numMethods); 
      obj.T                            = cell(obj.numKeyFrames,obj.numMethods);
      obj.V_err                     = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.W_err                    = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.V                            = cell(obj.numKeyFrames,obj.numMethods); 
      obj.W                           = cell(obj.numKeyFrames,obj.numMethods);

    
    end
    function res = get_res(obj)
      % remove NaN entries (corresponding to skipped frames)
      nanIdx = find(sum(isnan(obj.Q_err),2));
      obj.Q_err(nanIdx,:)  = [];
      obj.T_err(nanIdx,:) = [];
      % statistics of error for rotation and translation estimates
      rotErrM     = mean(obj.Q_err,1);           % Mean
      rotErrS     = std(obj.Q_err,1);            % Standard deviation
      rotErrMd    = median(obj.Q_err,1);         % Median
      rotErrQ1    = quantile(obj.Q_err,0.25, 1); % 25% quartile
      rotErrQ3    = quantile(obj.Q_err,0.75, 1); % 75% quartile
      tranErrM    = mean(obj.T_err,1);
      tranErrS    = std(obj.T_err,1);
      tranErrMd   = median(obj.T_err,1);
      tranErrQ1   = quantile(obj.T_err,0.25, 1);
      tranErrQ3   = quantile(obj.T_err,0.75, 1);

      % Table of results
      RowNames = {'Rot err mean';...
                  'Rot err std';...
                  'Rot err median'; 
                  'Rot err Q_1';...
                  'Rot err Q_3';...
                  'Tran err mean';...
                  'Tran err std';...
                  'Tran err median';...
                  'Tran err Q_1';...
                  'Tran err Q_3'};
      data      = [rotErrM;
                   rotErrS;
                   rotErrMd;
                   rotErrQ1;
                   rotErrQ3;
                   tranErrM;
                   tranErrS;
                   tranErrMd;
                   tranErrQ1;
                   tranErrQ3;];
      if obj.numMethods == 1
        res  = table(data(:,1), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 2
        res  = table(data(:,1), ...
                     data(:,2), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 3
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 4
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     data(:,4), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 5
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     data(:,4), ...
                     data(:,5), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms);    
      end

      if obj.res_prt_en
        disp(res);
      end 
      if obj.res_sav_en
        fname = strcat(obj.outDir,'res_',obj.test_ID,'_tab.csv');
        writetable(res,fname);
      end 
    end

  end
end
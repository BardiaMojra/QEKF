classdef config_class < matlab.System & dat_class
  properties
    %% features 
    test_single_bench_en     = true;
    
    %% test config (constant)
    test_ID     =  'quest_unit_test'; % test ID 
    outDir      = [pwd '/out/'];
    % test config dependent 
    test_outDir
    
    
    %% state machine config
    cntr % key frame counter
    numMethods  % num of algs used for comparison
    Q_errs % rot err for each method
    T_errs % trans err for each method
    Qs % recovered quaternions 
    Ts % recovered translations
    Vs
    Ws 
    V_errs
    W_errs

    %%QuEst config (constant)
    dats  % dataset handler array for multi-data mode 
%     dat  % dataset handler for single mode 
    
    pose_algorithms      = { ...
                                            'EightPt'; 
                                            'Nister'; 
                      %                        'Kneip'; 
                                            'Kukelova'; 
                      %                        'Stewenius'; 
                                            'QuEst'}; % algorithms to run 
 
    numBenchmarks;
%     benchmark =  'KITTI'; % benchmark name (in use, and in single mode)
    benchmarks      = { ...
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
      obj.numBenchmarks = length(obj.benchmarks);

      obj.dats = cell(obj.numBenchmarks,1);
      for i = 1: obj.numBenchmarks
        obj.dats{i} = dat_class( benchtype=obj.benchmarks(i));
          %           st_frame = 650, ...
          %          end_frame = 680 );  
      end

      obj.numMethods        = length(obj.pose_algorithms);             
      obj.Q_errs                     = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.T_errs                      = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.Qs                            = cell(obj.numKeyFrames,obj.numMethods); 
      obj.Ts                            = cell(obj.numKeyFrames,obj.numMethods);
      obj.V_errs                     = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.W_errs                    = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.Vs                            = cell(obj.numKeyFrames,obj.numMethods); 
      obj.Ws                           = cell(obj.numKeyFrames,obj.numMethods);
    end
    function res = get_res(obj)
      % remove NaN entries (corresponding to skipped frames)
      nanIdx = find(sum(isnan(obj.Q_errs),2));
      obj.Q_errs(nanIdx,:)  = [];
      obj.T_errs(nanIdx,:) = [];
      % statistics of error for rotation and translation estimates
      rotErrM     = mean(obj.Q_errs,1);           % Mean
      rotErrS     = std(obj.Q_errs,1);            % Standard deviation
      rotErrMd    = median(obj.Q_errs,1);         % Median
      rotErrQ1    = quantile(obj.Q_errs,0.25, 1); % 25% quartile
      rotErrQ3    = quantile(obj.Q_errs,0.75, 1); % 75% quartile
      tranErrM    = mean(obj.T_errs,1);
      tranErrS    = std(obj.T_errs,1);
      tranErrMd   = median(obj.T_errs,1);
      tranErrQ1   = quantile(obj.T_errs,0.25, 1);
      tranErrQ3   = quantile(obj.T_errs,0.75, 1);

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
                     'VariableNames', obj.pose_algorithms); 
      elseif obj.numMethods == 2
        res  = table(data(:,1), ...
                     data(:,2), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.pose_algorithms); 
      elseif obj.numMethods == 3
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.pose_algorithms); 
      elseif obj.numMethods == 4
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     data(:,4), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.pose_algorithms); 
      elseif obj.numMethods == 5
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     data(:,4), ...
                     data(:,5), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.pose_algorithms);    
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
classdef dlog_class < matlab.System %& config_class
  properties
    res_prt_en  = true;
    res_sav_en = true;

    % config
    test_ID
    test_outDir
    numMethods 
    numBenchmarks 
    pose_algorithms  
    benchmarks 
    numKeyFrames
    %% datalog arrays for each module
    % quest 
    Q_err_hist % rot err for each method
    T_err_hist % trans err for each method
    Q_hist % recovered quaternions 
    T_hist % recovered translations
    % vest 
    V_hist
    W_hist 
    V_err_hist
    W_err_hist
    % qekf
    
  end
  methods
    % Constructor
    function obj = dlog_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end
    function obj = load_cfg(obj, cfg) %,  extraprop)
      % load config from cfg
      obj.test_outDir             = cfg.test_outDir;
      obj.numMethods          = cfg.numMethods;
      obj.numBenchmarks   = cfg.numBenchmarks;
      obj.pose_algorithms   = cfg.pose_algorithms;
      obj.benchmarks           = cfg.benchmarks;
     
      obj.numKeyFrames = NaN(obj.numBenchmarks,1);
      for ds = obj.numBenchmarks
         obj.numKeyFrames(ds) =  cfg.dats{ds}.numKeyFrames; 
      end
      disp(obj.numKeyFrames);
      obj = init(obj);
    end
    function obj = init(obj)
      % quest
      obj.Q_err_hist       = NaN(obj.numKeyFrames,obj.numMethods, obj.numBenchmarks); 
      obj.T_err_hist        = NaN(obj.numKeyFrames,obj.numMethods, obj.numBenchmarks ); 
      obj.Q_hist              = cell(obj.numKeyFrames,obj.numMethods, obj.numBenchmarks); 
      obj.T_hist              = cell(obj.numKeyFrames,obj.numMethods, obj.numBenchmarks);
      % vest
      obj.V_err_hist       = NaN(obj.numKeyFrames,obj.numMethods, obj.numBenchmarks); 
      obj.W_err_hist      = NaN(obj.numKeyFrames,obj.numMethods, obj.numBenchmarks); 
      obj.V_hist             = cell(obj.numKeyFrames,obj.numMethods, obj.numBenchmarks); 
      obj.W_hist            = cell(obj.numKeyFrames,obj.numMethods, obj.numBenchmarks);
      % qekf

    end

    function res = get_res(obj)
      %% quest
      % remove NaN entries (corresponding to skipped frames)
      nanIdx = find(sum(isnan(obj.Q_err_hist),2));
      obj.Q_err_hist(nanIdx,:)  = [];
      obj.T_err_hist(nanIdx,:) = [];
      % statistics of error for rotation and translation estimates
      rotErrM     = mean(obj.Q_err_hist,1);           % Mean
      rotErrS     = std(obj.Q_err_hist,1);            % Standard deviation
      rotErrMd    = median(obj.Q_err_hist,1);         % Median
      rotErrQ1    = quantile(obj.Q_err_hist,0.25, 1); % 25% quartile
      rotErrQ3    = quantile(obj.Q_err_hist,0.75, 1); % 75% quartile
      tranErrM    = mean(obj.T_err_hist,1);
      tranErrS    = std(obj.T_err_hist,1);
      tranErrMd   = median(obj.T_err_hist,1);
      tranErrQ1   = quantile(obj.T_err_hist,0.25, 1);
      tranErrQ3   = quantile(obj.T_err_hist,0.75, 1);

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
        fname = strcat(obj.test_outDir,'res_',obj.test_ID,'_QuEst_table.csv');
        writetable(res,fname);
      end 
    end

  end
end
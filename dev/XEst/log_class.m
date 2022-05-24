classdef log_class < matlab.System
  properties
    % config (argin)
    benchtype  % single benchmark    
    keyFrames  % corresponding keyframes 
    algorithms  

    % private
    numKeyFrames
    numMethods 

    %% datalog arrays for each module
%     log_labels = { ...
    cntr_hist
    frame_hist
    % quest 
    T_hist % recovered translations
    Q_hist % recovered quaternions 
    % vest 
    V_hist
    W_hist 
    % qekf
    
    %% Log Errs
    % init in init() but always compute and fill at post-processing 
    T_errs % rot err for each method
    Q_errs % trans err for each method
    V_errs
    W_errs
    

    

  end
  methods  % constructor
    function obj = log_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end
    function load_cfg(obj, cfg)
      % load config from cfg
      obj.algorithms       = cfg.pose_algorithms;
      obj.init();
    end 

  end % methods % constructor 
  methods (Access = public) 
    function log_state(obj, cntr, frame_idx, QT_sols, V, W)
      % 
      obj.cntr_hist(cntr, 1)     = cntr;
      obj.frame_hist(cntr, 1)    = frame_idx;
      % quest
      for alg = 1:length(obj.algorithms)
        msg = sprintf("[log_class]--> obj.algorithms{%d}: %s  DOES NOT match , TQ_sols{1, %d}{1}: %s", ...
          alg, obj.algorithms{alg}, alg, QT_sols{1, alg}{1});
        assert(strcmp(obj.algorithms{alg}, QT_sols{1, alg}{1}), msg); 
        obj.Q_hist{cntr, alg}      =  QT_sols{3, alg};  
        obj.T_hist{cntr, alg}      =  QT_sols{2, alg};
        % vest
        obj.V_hist{cntr, alg}      = V;
        obj.W_hist{cntr, alg}     = W;
        % qekf
      
      end
    end % function log_state(obj, benchName, cntr, frame_idx, T, Q, V, W)
  end % methods (Access = public) % public functions
  methods (Access = private) % private functions
    function init(obj)
      %todo change cell arrays with matrices 

      obj.numMethods      = length(obj.algorithms);
      obj.numKeyFrames    = length(obj.keyFrames);
      % 
      obj.cntr_hist       = NaN(obj.numKeyFrames,1);
      obj.frame_hist      = NaN(obj.numKeyFrames,1);
      % quest
      obj.Q_errs          = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.T_errs          = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.Q_hist          = cell(obj.numKeyFrames,obj.numMethods); 
      obj.T_hist          = cell(obj.numKeyFrames,obj.numMethods);
      % vest
      obj.V_errs          = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.W_errs          = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.V_hist          = cell(obj.numKeyFrames,obj.numMethods); 
      obj.W_hist          = cell(obj.numKeyFrames,obj.numMethods);
      % qekf

    end % init(obj)
  end % methods (Access = private) % private functions
end
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
    idx_hist
    frame_hist
    % quest 
    T_hist % recovered translations
    Q_hist % recovered quaternions 
    T_err_hist % rot err for each method
    Q_err_hist % trans err for each method
    % vest 
    V_hist
    W_hist 
    V_err_hist
    W_err_hist
    % qekf

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
    function log_state(obj, idx, frame_idx, TQ_sols, V, W)
      % 
      obj.idx_hist(idx, 1)         = idx;
      obj.frame_hist(idx, 1)    = frame_idx;
      % quest
      for alg = length(obj.algorithms)
        msg = sprintf("[log_class]--> obj.algorithms{%d}: %s  DOES NOT match , TQ_sols{%d, 3}{1}: %s", ...
          alg, obj.algorithms{alg}, alg, TQ_sols{alg, 3}{1});
        assert(strcmp(obj.algorithms{alg}, TQ_sols{alg, 3}{1}), msg); 
          
        obj.T_hist{idx, alg}      =  TQ_sols{alg, 1};
        obj.Q_hist{idx, alg}      =  TQ_sols{alg, 2};
      end
      % vest
      obj.V_hist{idx, 1}      = V';
      obj.W_hist{idx, 1}     = W';
      % qekf
      
    end % function log_state(obj, benchName, idx, frame_idx, T, Q, V, W)
  end % methods (Access = public) % public functions
  methods (Access = private) % private functions
    function init(obj)
      %todo change cell arrays with matrices 

      obj.numMethods        = length(obj.algorithms);
      obj.numKeyFrames    = length(obj.keyFrames);
      % 
      obj.idx_hist            = NaN(obj.numKeyFrames,1);
      obj.frame_hist       = NaN(obj.numKeyFrames,1);
      % quest
      obj.Q_err_hist       = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.T_err_hist        = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.Q_hist              = cell(obj.numKeyFrames,obj.numMethods); 
      obj.T_hist              = cell(obj.numKeyFrames,obj.numMethods);
      % vest
      obj.V_err_hist       = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.W_err_hist      = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.V_hist             = cell(obj.numKeyFrames,obj.numMethods); 
      obj.W_hist            = cell(obj.numKeyFrames,obj.numMethods);
      % qekf

    end % init(obj)
  end % methods (Access = private) % private functions
end
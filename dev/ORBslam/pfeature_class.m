classdef pfeature_class < matlab.System 
  %% point feature class 
  properties % public vars
    %% features 
    mat_feat_sav_fig_en   = true;
    mat_feat_disp_fig_en  = false; % disp matched features between two keyframes
    mat_feat_sav_pts_en   = false; 
    %% cfg argin
    TID
    ttag
    toutDir
    benchmark
    maxPts  = 30
    %pos_algs
    %vel_algs  
    %% private vars

    mats  % current frame feature points 
    m_dot % m(i) - m(i-1) of matched feature points  
    % rpt constants 
    mod_name    = 'fdetect'
    rpt_note    = " "
  end
  methods % constructor
    
    function obj = pfeature_class(varargin) 
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end  
  
  end 
  methods (Access = public) 
  
    function load_cfg(obj, cfg)
      obj.TID            = cfg.TID;                   
      obj.ttag           = cfg.ttag;                   
      obj.toutDir        = cfg.toutDir;
      obj.benchmark      = cfg.benchmark;
      obj.init();
    end

    function get_pmats(obj, kfi, dat) 
      % get current frame features and match w prev frame    
      [dat.npoints,dat.In] = GetFeaturePoints(kfi, dat.dataset, dat.surfThresh);            
      dat.matches   = MatchFeaturePoints(dat.Ip, ...
                                         dat.ppoints, ...
                                         dat.In, ...
                                         dat.npoints, ...
                                         obj.maxPts, ...
                                         dat.dataset, ...
                                         kfi, ...
                                         obj.mat_feat_disp_fig_en, ... 
                                         obj.mat_feat_sav_fig_en, ...
                                         obj.toutDir, ...
                                         obj.mat_feat_sav_pts_en);
      [dat.relPose, dat.posp] = RelativeGroundTruth(kfi, dat.posp, dat.dataset);
    end 

   

  end % end of public access 
  methods (Access = private)
    
    function init(obj)
      
    end 
    
 
    function s = save(obj) %% Backup/restore functions
      s = save@matlab.System(obj);
      %s.myproperty = obj.myproperty;     
    end

    function load(obj,s,wasLocked)
      obj.myproperty = s.myproperty;       
      load@matlab.System(obj,s,wasLocked);
    end
    
  end % methods (Access = private)
end

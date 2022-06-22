classdef camera_class < matlab.System 
  properties
    %% features
    %% cfg (argin)
    TID
    outDir
    datDir
    st_frame
    end_frame
    btype
    toutDir
    ttag % TID+benchmark
    %% private
    

  end

  methods % constructor
    function obj = camera_class(varargin) 
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end 
  end % methods % constructor
  methods (Access = public) 
    
    function load_cfg(obj, cfg) 
      obj.TID           = cfg.TID;
      obj.outDir        = cfg.outDir;
      obj.datDir        = cfg.datDir;
      obj.btype         = cfg.btype;
      obj.toutDir       = cfg.toutDir;
      obj.ttag          = cfg.ttag ;
     
      obj.init();
      
    end


  end 
  methods  (Access = private)
    
    %%
    function load_TUM_RGBD_camera(obj)

    end 

    function load_camera_model(obj)
      if strcmp(obj.btype, "TUM_RGBD")
        obj.load_TUM_RGBD_camera();
      %elseif
      else
        assert(false, "[dat.load_dat]->> unknown dataset!");
      end
    end

    function init(obj)
      obj.load_camera_model();
      
    end
  end
end

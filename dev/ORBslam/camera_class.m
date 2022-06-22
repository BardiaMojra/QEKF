classdef camera_class < matlab.System 
  properties
    %% features

    %% config (private)
    focLen      = [535.4, 539.2]  % in units of pixels
    princPoint  = [320.1, 247.6] % in units of pixels
    imgSize     % in units of pixels
    %% cfg (argin)
    cfg
    TID
    outDir
    datDir
    st_frame
    end_frame
    btype
    toutDir
    ttag % TID+benchmark

    %% parameters
    ins % camera intrinsics
    %% rpt cfg (argout)
    mod_name    = 'camera'
    rpt_note    = ' '

  end

  methods % constructor
    function obj = camera_class(varargin) 
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end 
  end % methods % constructor
  methods (Access = public) 
    
    function load_cfg(obj, cfg) 
      cfg.cam           = obj;
      obj.cfg           = cfg;
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
      % Create a cameraIntrinsics object to store the camera intrinsic parameters.
      % The intrinsics for the dataset can be found at the following page:
      % https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
      % Note that the images in the dataset are already undistorted, hence there
      % is no need to specify the distortion coefficients.
      obj.imgSize = size(obj.cfg.dat.firstI,[1 2]); 
      obj.ins  = cameraIntrinsics(obj.focLen, obj.princPoint, obj.imgSize);
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

classdef dat_class < matlab.System 
  properties
    %% config (constant)
    datDir               = [pwd '/data/']; % data dir     
    st_frame          = 1; % start frame index
    end_frame       = nan;% end frame index
    benchtype       = 'KITTI'; % default
    benchnum       = 3; % aux config, used in KITTI     
    surfThresh      = 200; % SURF feature detection threshold

    % vars (private)
    dataset; % dataset obj
    keyFrames
    dat_posp; 
    ppoints_i; % init frame points 
    Ip_i; % init frame image 
    skipFrame       = 0; % num of frames skipped bwt two keyframes        
    numImag % total num of images
    numKeyFrames % num of keyframes

  end

  methods
   
    function obj = dat_class(varargin) % Constructor
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end 

    function obj = load_cfg(obj, cfg) %,  extraprop)
      % obj.datDir           =   cfg.datDir;          
      % obj.benchtype   =   cfg.benchtype;  % keep out for multiple datasets
      % obj.benchnum   =   cfg.benchnum;  
      obj.st_frame      =   cfg.st_frame;      
      obj.end_frame   =   cfg.end_frame;  
      obj.surfThresh  =    cfg.quest_surfThresh; 
      obj = obj.init();
    end
  end 

  methods  (Access = private)
    % In setup allocate any resources, which in this case is
    % opening the file.
    function obj = init(obj)
      %init 
      [obj.dataset, obj.dat_posp] = LoadDataset( ...
        obj.datDir, ...
        obj.benchtype, ...
        obj.benchnum, ...
        obj.st_frame, ...
        obj.end_frame); 

      if strcmp(obj.benchtype,'KITTI')
        obj.skipFrame = 1; 
      elseif strcmp(obj.benchtype,'NAIST')
        obj.skipFrame = 1; 
      elseif strcmp(obj.benchtype,'ICL')
        obj.skipFrame = 1; 
      elseif strcmp(obj.benchtype,'TUM')
        obj.skipFrame = 1;     
      end
 
      obj.numImag              = length(obj.dataset.fnames); 
      obj.keyFrames           = 2+obj.skipFrame:1+obj.skipFrame:obj.numImag; 
      obj.numKeyFrames   = length(obj.keyFrames);      
      
      [obj.ppoints_i, obj.Ip_i] = GetFeaturePoints(obj.st_frame,obj.dataset,...
        obj.surfThresh);
    end
  end
end

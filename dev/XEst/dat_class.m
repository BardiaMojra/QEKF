classdef dat_class < handle 
  %% DAT_CLASS Add summary here
  % Public, tunable properties
  properties
    %% data config (constant)
    datDir              = [pwd '/data/']; % data dir     
    st_frame          = 1; % start frame index
    end_frame       = nan;% end frame index
    benchtype       = 'KITTI'; % default
    seq                   = 3; % aux config, used in KITTI     

    % data config dependent (constant)
    dataset; % dataset obj
    posp; 
    ppoints_i; % init frame points 
    Ip_i; % init frame image 
    skipFrame       = 0; % num of frames skipped bwt two keyframes        
    numImag % total num of images
    keyFrames;
    numKeyFrames % num of keyframes

  end
  
  properties(Nontunable)% Public, non-tunable properties
  end
  properties(DiscreteState)
  end
  properties(Access = private) % Pre-computed constants
  end

  methods
    % Constructor
    function obj = dat_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end 
    function obj = dat_init(datDir, benchtype, benchnum, st_frame, end_frame)

      obj.datDir  =   datDir  ;
      obj.benchtype  =  benchtype  ;
      obj.benchnum  =  benchnum  ;
      obj.st_frame =  st_frame ;
      obj.end_frame =  end_frame ;
      
      %init 
      [obj.dataset, obj.posp] = LoadDataset(datDir, benchtype, benchnum, st_frame, end_frame); 

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

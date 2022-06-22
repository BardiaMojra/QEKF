classdef tracker_class < matlab.System 
  properties
    %% features
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
    firstI % first image
    idx_currFr % idx of curr image 
    currI % curr image 
    currFts % curr feats
    currPts % curr feat pt coords
    prevFts % prev feats 
    prevPts % prev feat pt coords
    idxPairs
    isMapInitd % is map initialized?
    relPose
    xyzWorldPoints
    tracks
    hfeature

    %% rpt cfg (argout)
    mod_name    = 'tracker'
    rpt_note    = ' '
  end
  methods % constructor
    function obj = tracker_class(varargin) 
      setProperties(obj,nargin,varargin{:});
    end 

  end % methods
  methods (Access = public) 
    
    function load_cfg(obj, cfg)
      cfg.tkr           = obj;
      obj.cfg           = cfg;
      obj.TID           = cfg.TID;
      obj.outDir        = cfg.outDir;
      obj.datDir        = cfg.datDir;
      obj.st_frame      = cfg.st_frame;
      obj.end_frame     = cfg.end_frame;
      obj.btype         = cfg.btype;
      obj.toutDir       = cfg.toutDir;
      obj.ttag          = cfg.ttag ;
     
      dat = obj.cfg.dat;
      obj.init(dat);
    end

    function init(obj, dat)
      obj.idx_currFr = 1;
      obj.firstI = readimage(dat.imds, obj.idx_currFr);
      obj.currI = obj.firstI;
      %imshow(obj.currI); % save init frame?
      
    end

  end 
  methods  (Access = private)

    %function init(obj)
    %end


    
  end % methods
end

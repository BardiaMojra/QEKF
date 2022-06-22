classdef pfeature_class < matlab.System 
  properties
    %% features 
    frm_feat_sav_fig_en   = true
    frm_feat_disp_fig_en  = false
    frm_feat_sav_pts_en   = false 
    mat_feat_sav_fig_en   = true
    mat_feat_disp_fig_en  = false
    mat_feat_sav_pts_en   = false 
    %% cfg (argin)
    cfg
    TID
    outDir
    datDir
    btype
    toutDir
    ttag % TID+benchmark
    %% parameters
    % descriptor config
    %desc_type       = 'orb'
    scaleFactor = 1.2
    numLevels   = 8
    numPoints   = 1000
    % matching config
    minMatches = 100
    
    firstI
    idx_firstFr
    %% rpt cfg (argout)
    mod_name    = 'pfeature'
    rpt_note    = " "
  end
  methods % constructor
    
    function obj = pfeature_class(varargin) 
      setProperties(obj,nargin,varargin{:});
    end  
  
  end 
  methods (Access = public) 
  

    function load_cfg(obj, cfg)
      cfg.pft           = obj;
      obj.cfg           = cfg;
      obj.TID           = cfg.TID;
      obj.outDir        = cfg.outDir;
      obj.datDir        = cfg.datDir;
      obj.btype         = cfg.btype;
      obj.toutDir       = cfg.toutDir;
      obj.ttag          = cfg.ttag;
     
      obj.init();
    end

    function init_initFrame(obj, tkr)
      [tkr.prevFts, tkr.prevPts] = helperDetectAndExtractFeatures( ...
        tkr.currI, obj.scaleFactor, obj.numLevels, obj.numPoints);
      
      tkr.idx_currFr  = tkr.idx_currFr + 1;
      tkr.firstI      = obj.cfg.dat.firstI; 
      tkr.isMapInitd  = false;
    end
   

  end % (Access = public) 
  methods (Access = private)
    
    function init(obj)
      obj.idx_firstFr = 1;
      obj.firstI = readimage(obj.cfg.dat.imds, obj.idx_firstFr);  
      %imshow(obj.firstI); % save init frame?
    end 

  end % methods (Access = private)
end

classdef dat_class < matlab.System 
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
    dsetPath 
    imds % img dset dir obj
    firstI
    idx_firstFr
    %% rpt cfg (argout)
    mod_name    = 'data'
    rpt_note    = ' '
  end
  methods % constructor
    function obj = dat_class(varargin) 
      setProperties(obj,nargin,varargin{:});
    end 

  end 
  methods (Access = public) 
    
    function load_cfg(obj, cfg) 
      cfg.dat           = obj;
      obj.cfg           = cfg;
      obj.TID           = cfg.TID;
      obj.outDir        = cfg.outDir;
      obj.datDir        = cfg.datDir;
      obj.st_frame      = cfg.st_frame;
      obj.end_frame     = cfg.end_frame;
      obj.btype         = cfg.btype;
      obj.toutDir       = cfg.toutDir;
      obj.ttag          = cfg.ttag;
     
      obj.load_benchmark();
      obj.init();
    end

  end 
  methods  (Access = private)
    
    %%
    function load_TUM_RGBD_dat(obj)
      baseDownloadURL = ['https://vision.in.tum.de/rgbd/dataset/' ...
        'freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz']; 
      obj.dsetPath      = fullfile(obj.datDir,obj.btype, filesep); 
      options           = weboptions('Timeout', Inf);
      tgzFileName       = strcat(obj.dsetPath, 'fr3_office.tgz');
      folderExists      = exist(obj.dsetPath, 'dir');
      if ~folderExists  
        mkdir(obj.dsetPath); 
        disp(['Downloading fr3_office.tgz (1.38 GB). ' ...
          'This download can take a few minutes.']) 
        websave(tgzFileName, baseDownloadURL, options); 
        disp('Extracting fr3_office.tgz (1.38 GB) ...') 
        untar(tgzFileName, obj.dsetPath); 
      end
    end 

    function load_benchmark(obj)
      if strcmp(obj.btype, "TUM_RGBD")
        obj.load_TUM_RGBD_dat();
      %elseif
      else
        assert(false, "[dat.load_dat]->> unknown dataset!");
      end
    end

    function init(obj)
      fname     = strcat(obj.dsetPath, ...
        'rgbd_dataset_freiburg3_long_office_household/rgb/');
      obj.imds  = imageDatastore(fname);
      obj.idx_firstFr   = 1;
      obj.firstI = readimage(obj.imds, obj.idx_firstFr);
    
    end

  end % methods
end

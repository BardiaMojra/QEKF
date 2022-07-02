classdef config_class < matlab.System
  properties
    %% features
    %test_single_bench_en    = true
    %sliding_ref_en          = true
    %% configs --->> write to other modules
    TID               = nan
    pos_alg           = 'default_5Pt'
    outDir            = [pwd '/out']
    datDir            %= ['/home/smerx/DATA'] % [pwd '/data']
    st_frame          = nan % start frame index
    end_frame         = nan % end frame index
    btype             = 'default'
    desc              = "test description."
    %% private
    toutDir
    ttag % TID+benchmark
    %%
  end
  methods
    function obj = config_class(varargin)
      setProperties(obj,nargin,varargin{:})
      addpath(genpath('./'));
      %addpath(genpath('/home/smerx/DATA'));
      init(obj);
    end

  end
  methods (Access = private)

    function init(obj)
      obj.ttag            = strcat(obj.TID,'_',obj.btype,'_',obj.pos_alg);

      %% setup outDir
      obj.toutDir         = strcat(obj.outDir,'/',obj.ttag,'/');
      if not(isfolder(obj.toutDir))
        disp("[config]->> test_outDir does NOT exist: ");
        disp(obj.toutDir);
        mkdir(obj.toutDir);
        disp("[config]->> directory has been created!");
      end

      %% sav desc file
      fname     = strcat(obj.toutDir,"description.txt");
      file      = fopen(fname,"wt");
      fprintf(file, obj.desc);
      fclose(file);

      %% setup datDir
      % Use |imageDatastore| to get a list of all image file names in a dir
      if strcmp(obj.btype, "default")
        obj.datDir = fullfile(toolboxdir('vision'), 'visiondata', 'structureFromMotion');
      elseif strcmp(obj.btype, "TUM_RGBD")
        obj.datDir = '/home/smerx/DATA/TUM_RGBD/rgbd_dataset_freiburg3_long_office_household/rgb';
      else
        assert(false, "[cfg.init]->> unknown dataset selection!");
      end


    end



  end % methods (Access = private)
end

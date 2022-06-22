classdef config_class < matlab.System 
  properties
    %% features
    %test_single_bench_en    = true
    %sliding_ref_en          = true
    %% configs --->> write to other modules
    TID               = nan
    outDir            = [pwd '/out']   
    datDir            = ['/home/smerx/DATA'] % [pwd '/data']   
    st_frame          = nan % start frame index
    end_frame         = nan % end frame index
    btype             = "TUM_RGBD"
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
      obj.ttag            = strcat(obj.TID,'_',obj.btype);
      obj.toutDir         = strcat(obj.outDir,'/',obj.ttag,'/');
      if not(isfolder(obj.toutDir))
        disp("[config]->> test_outDir does NOT exist: ");
        disp(obj.toutDir);
        mkdir(obj.toutDir);
        disp("[config]->> directory has been created!");
      end 

      % write desc to file 
      fname     = strcat(obj.toutDir,"description.txt");
      file      = fopen(fname,"wt");
      fprintf(file, obj.desc);
      fclose(file);
      
   
    end
  
   
      
  end % methods (Access = private)
end
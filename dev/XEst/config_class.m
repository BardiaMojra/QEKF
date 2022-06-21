classdef config_class < matlab.System 
  properties
    %% features
    test_single_bench_en    = true
    sliding_ref_en          = true
    %% configs --->> write to other modules
    TID               = 'XEst_dev_test'
    outDir            = [pwd '/out']   
    datDir            = ['/home/smerx/DATA'] % [pwd '/data']   
    st_frame          = 660 %nan % start frame index
    end_frame         = 680 % nan % end frame index
    del_T             = 0.1 % time period 
    surfThresh        = 150 % SURF feature detection threshold
    benchnum          = 3 % benchmark subset
    benchmark         = 'KITTI' % 10 frames/sec 
    %benchmark         = 'NAIST'
    %benchmark         = 'ICL'
    %benchmark         = 'TUM' 
    skipframe         = 1 %nan % default
    pos_algs          = { ...
                         'EightPt'; 
                         'Nister'; 
                         %'Kneip';  
                         'Kukelova'; 
                         %'Stewenius';
                         'QuEst'; 
                         'VEst'}; % algorithms to run % state machine vars
    vel_algs          = {'VEst'};
    %% cfgs <<--- read from a submod and write to other submods 
    kframes % read from dat_class obj 
    %% private
    toutDir
    ttag % TID+benchmark
    %dats  % dataset handler array for multi-data mode 
    dat
    pos_numAlgs
    vel_numAlgs
    %numBenchmarks    
  end
  methods  % constructor
    
    function obj = config_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
      addpath(genpath('./'));
      addpath(genpath('/home/smerx/DATA')); 
      addpath(genpath('/home/smerx/git/opengv/matlab')); % add opengv
      init(obj);
    end
  
  end 
  methods (Access = private)
  
    function init(obj)

      obj.ttag            = strcat(obj.TID,'_',obj.benchmark);
      obj.toutDir         = strcat(obj.outDir,'/',obj.ttag,'/');
      obj.pos_numAlgs     = length(obj.pos_algs);
      obj.vel_numAlgs     = length(obj.vel_algs);
      if not(isfolder(obj.toutDir))
        disp("[config]->> test_outDir does NOT exist: ");
        disp(obj.toutDir);
        %pause(5);
        mkdir(obj.toutDir);
        disp("[config]->> directory has been created!");
      end 
      obj.dat = dat_class(benchtype = obj.benchmark);
      obj.dat.load_cfg(obj);
    end
  
    function set_skipframe(obj)
      if isequal(obj.skipframe, nan)
        if strcmp(obj.benchmark,'KITTI')
          obj.skipframe = 1; 
        elseif strcmp(obj.benchmark,'NAIST')
          obj.skipframe = 1; 
        elseif strcmp(obj.benchmark,'ICL')
          obj.skipframe = 1; 
        elseif strcmp(obj.benchmark,'TUM')
          obj.skipframe = 1;     
        end
      end
    end
      
  end % methods (Access = private)
end
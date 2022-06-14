classdef config_class < matlab.System %& dat_class
  properties
    %% features
    test_single_bench_en    = true
    %% configs --->> write to other modules
    TID               = 'XEst_dev_test'
    outDir            = [pwd '/out']   
    datDir            = ['/home/smerx/DATA'] % [pwd '/data']   
    st_frame          = nan % start frame index
    end_frame         = 11 % nan % end frame index
    del_T             = 0.01 % time period 
    surfThresh        = 200 % SURF feature detection threshold
    benchnum          = 3 % benchmark subset
    benchmark         = 'KITTI'
    %benchmark         = 'NAIST'
    %benchmark         = 'ICL'
    %benchmark         = 'TUM' 
    pos_algs          = { ...
                         %'EightPt'; 
                         %'Nister'; 
                         %'Kneip';  % dep on opengv
                         %'Kukelova'; 
                         %'Stewenius';  % dep on opengv
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
    pos_numMethods  % num of algs used for comparison
    vel_numMethods  
    %numBenchmarks    
  end
  methods  % constructor
    
    function obj = config_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
      init(obj);
    end
  
  end 
  methods (Access = private)
  
    function init(obj)
      obj.ttag    = strcat(obj.TID,'_',obj.benchmark);
      obj.toutDir = strcat(obj.outDir,'/',obj.ttag,'/');
      if not(isfolder(obj.toutDir))
        disp('test_outDir does NOT exist: ');
        disp(obj.toutDir);
        pause(5);
        mkdir(obj.toutDir);
      end 
      obj.dat = dat_class(benchtype = obj.benchmark);
      obj.dat.load_cfg(obj);
    end
  
  end % methods (Access = private)
end
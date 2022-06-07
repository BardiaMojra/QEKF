classdef config_class < matlab.System %& dat_class
  properties
    %% features
    test_single_bench_en    = true
    %% configs --->> write to other modules
    test_ID           = 'XEst_dev_test'
    outDir            = [pwd '/out/']   
    datDir            = ['/home/smerx/DATA']
    %datDir            = [pwd '/data']   
    st_frame          = 650 % start frame index
    end_frame         = 680 % end frame index
    del_T             = 0.1 % time period 
    surfThresh        = 200 % SURF feature detection threshold
    benchnum          = 3 % benchmark subset
    benchmark         = 'KITTI'
    %benchmark         = 'NAIST'
    %benchmark         = 'ICL'
    %benchmark         = 'TUM' 
    pos_algs          = { ...
                         'EightPt'; 
                         'Nister'; 
                         %'Kneip';  % dep on opengv
                         'Kukelova'; 
                         %'Stewenius';  % dep on opengv
                         'QuEst'; 
                         'VEst'}; % algorithms to run % state machine vars
    vel_algs          = {'VEst'};
    %% cfgs <<--- read from a submod and write to other submods 
    kframes % read from dat_class obj 
    %% private
    test_outDir
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
      obj.test_outDir = [obj.outDir 'out_' obj.test_ID '_' obj.benchmark '/'];
      if not(isfolder(obj.test_outDir))
        disp('test_outDir does NOT exist: ');
        disp(obj.test_outDir);
        pause(5);
        mkdir(obj.test_outDir);
      end 
      obj.dat = dat_class(benchtype = obj.benchmark);
      obj.dat.load_cfg(obj);
      %obj.init(); % move all this there 
      %obj.numBenchmarks = length(obj.benchmarks);
      %obj.dats = cell(obj.numBenchmarks,1); % create corresponding dat_class objs
      %for i = 1: obj.numBenchmarks
      %  obj.dats{i} = dat_class(benchtype = obj.benchmarks(i));
      %  obj.dats{i}.load_cfg(obj); 
      %end
    end
  
  end % methods (Access = private)
end
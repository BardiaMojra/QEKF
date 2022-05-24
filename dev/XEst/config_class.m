classdef config_class < matlab.System %& dat_class
  properties
    %
    test_single_bench_en    = true;
    test_ID                 =  'quest_unit_test'; % test ID 
    outDir                  = [pwd '/out/'];      
    st_frame   = 1 % start frame index
    end_frame  = nan % end frame index
    % test config dependent 
    test_outDir
    quest_surfThresh     = 200; % SURF feature detection threshold
    pose_algorithms      = { ...
                            'EightPt'; 
                            'Nister'; 
                            %'Kneip';  % dep on opengv
                            'Kukelova'; 
                            %'Stewenius';  % dep on opengv
                            'QuEst'}; % algorithms to run 
    %% benchmarks
    benchmark       = 'KITTI'
    %benchmark       = 'NAIST'
    %benchmark       = 'ICL'
    %benchmark       = 'TUM' 
    % state machine vars
    %dats  % dataset handler array for multi-data mode 
    dat
    numMethods  % num of algs used for comparison
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
      %obj.numBenchmarks = length(obj.benchmarks);
      %obj.dats = cell(obj.numBenchmarks,1); % create corresponding dat_class objs
      %for i = 1: obj.numBenchmarks
      %  obj.dats{i} = dat_class(benchtype = obj.benchmarks(i));
      %  obj.dats{i}.load_cfg(obj); 
      %end
    end
  end % methods (Access = private)
end
classdef dat_class < matlab.System 
  properties
    %% config (constant)
    datDir          = [pwd '/data']; % data dir     
    st_frame        = nan; % start frame index
    end_frame       = nan;% end frame index
    benchtype       = 'KITTI'; % default
    benchnum        = 3; % aux config, used in KITTI     
    surfThresh      = 200; % SURF feature detection threshold
    % vars (init internally)
    dataset % dataset obj
    %imgpath
    %datapath
    kframes
    posp_i % init frame ground truth pose (given)
    ppoints_i; % init frame points 
    Ip_i; % init frame image 
    skipframe % num of frames skipped bwt two keyframes        
    %numImag % total num of images
    num_kframes % num of keyframes
    %% run-time variables 
    t % frame translation 
    q % frame quaternion orientation 
    posp % previous frame ground truth pose (per frame, given)
    ppoints % previous frame feature points 
    Ip % previous image
    npoints % current frame feature points 
    In % current image 
    matches % matches_class obj
    relPose % relative ground truth transform (bewteen frames)    
  end

  methods % constructor
    function obj = dat_class(varargin) 
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end 
  end % methods % constructor
  methods (Access = public) 
    
    function load_cfg(obj, cfg) 
      obj.datDir        = cfg.datDir;          
      obj.benchtype     = cfg.benchmark;  
      obj.benchnum      = cfg.benchnum;  
      obj.st_frame      = cfg.st_frame;      
      obj.end_frame     = cfg.end_frame;  
      obj.surfThresh    = cfg.surfThresh; 
      obj.skipframe     = cfg.skipframe;
      obj.init();
      cfg.st_frame      = obj.st_frame; % write back to cfg obj after dat obj init
      cfg.end_frame     = obj.end_frame;
      cfg.kframes       = obj.kframes; 
      
      %disp(cfg.kframes);
      %pause(5);
    end

    function [rgt_T, rgt_Q] = get_kframe_rgtruths(obj) % get ground truth relative translation
      t1      = obj.posp_i.t1;
      q1      = obj.posp_i.q1;
      rgt_T   = nan(length(obj.kframes), 3);  
      rgt_Q   = nan(length(obj.kframes), 4);      
      for i = 1: length(obj.kframes)
        [tr,qr,t2,q2] = get_relGT(obj.kframes(i), obj.benchtype, obj.dataset.tTru, ...
          obj.dataset.qTru, t1, q1);
        rgt_T(i,:) = tr';
        rgt_Q(i,:) = qr';        
        t1 = t2; q1 = q2;
      end
      %disp(rgt_T);
      %disp(rgt_Q);
    end 

  end 
  methods  (Access = private)
    
    function init(obj)
      %init 
      [obj.dataset, obj.posp_i] = LoadDataset(obj.datDir, obj.benchtype, ...
                                              obj.benchnum, obj.st_frame, ...
                                              obj.end_frame); 
      obj.st_frame      = obj.dataset.st_frame;
      obj.end_frame     = obj.dataset.end_frame;
      obj.kframes       = obj.st_frame+obj.skipframe:1+obj.skipframe:obj.end_frame; 
      obj.num_kframes   = length(obj.kframes);            
      [obj.ppoints_i, obj.Ip_i]   = GetFeaturePoints(obj.st_frame, ...
        obj.dataset, obj.surfThresh);
      obj.posp        = obj.posp_i;
      obj.ppoints     = obj.ppoints_i; 
      obj.Ip          = obj.Ip_i;
    end
  end
end

classdef quest_class < matlab.System
  %% public vars
  properties 
    %% configs (passed in via cfg obj or set manually)
    %% features (private)
    %     features_sav_en   = true;
    matches_sav_en    = true;
    matches_disp_en   = false; % disp matched features between two keyframes
    sliding_ref_en    = false; % disable sliding_ref mode when running in real-time
    res_prt_en        = true; % print res in terminal  
    res_sav_en        = true; % save res table 

   
    ranThresh       = 1e-6;% RANSAC Sampson dist threshold (for outliers)
    surfThresh      = 200; % SURF feature detection threshold
    maxPts              = 30; % max num of features used in pose est (fewer points, faster compute)
    minPts              = 8; % max num of features required (6 to est a unique pose with RANSAC)
    algorithms      = {...
                       'EightPt'; 
                       'Nister'; 
%                        'Kneip'; 
                       'Kukelova'; 
%                        'Stewenius'; 
                       'QuEst'}; % algorithms to run 
    benchmarks      = {'KITTI';
                       'NAIST';
                       'ICL';
                       'TUM';
                       } % benchmarks ----> (disabled for now)
    % run-time variables 
    t % frame translation 
    q % frame quaternion orientation 
    posp % previous frame ground truth pose (per frame, given)
    numMethods  % num of algs used for comparison
    ppoints % previous frame feature points 
    Ip % previous image
    npoints % current frame feature points 
    In % current image 
    matches % matches_class obj
    relPose % relative ground truth transform (bewteen frames)    
  end

  methods
    %% constructor
    function obj = quest_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end    
  end
  
    %% public functions
%     function obj = load_cfg(obj, ...
%                             skipFrame, ...
%                             ransac_thresh, ...
%                             surf_thresh, ...
%                             maxPts, ...
%                             minPts)
      %% common 
%       obj.test_ID       = test_ID;
%       obj.outDir        = outDir;
%       obj.method        = method;
%       obj.st_frame      = st_frame;
%       obj.end_frame     = end_frame;
      % QuEst
%       obj.dataroot      = datDir;
%       obj.benchtype     = benchtype;
%       obj.benchnum      = seq;
%       obj.skipFrame     = skipFrame;
%       obj.ranThresh     = ransac_thresh;
%       obj.surfThresh    = surf_thresh;
%       obj.maxPts        = maxPts;
%       obj.minPts        = minPts;
      %% init
%       obj = obj.init();
%     end

  methods (Access = public)
    function [obj,t,q] = get_pose(obj, i, dat, cfg)
%       obj.cntr  = cfg.cntr + 1; % Counter          
      % get current frame features and match w prev frame    
      [obj.npoints,obj.In] = GetFeaturePoints(i,dat,obj.surfThresh);            
      obj.matches   = MatchFeaturePoints(obj.Ip,obj.ppoints,obj.In,obj.npoints,...
        obj.maxPts,dat,i,obj.matches_disp_en,obj.matches_sav_en, cfg.test_outDir);
      % relative ground truth transformation and previous frame gt pose
      [obj.relPose,obj.posp] = RelativeGroundTruth(i,obj.posp,obj.dataset);
      % skip frame if not enough matches found 
      if (obj.matches.numPts < obj.minPts) % || (obj.matches.status~=0)         
        obj.Ip      = obj.In; % use current frame as the next previous frame
        obj.ppoints = obj.npoints;         
        disp('Not enough matched feature points. Frame skipped!');
        obj.Frame_skipped_flag = true;
      else
        % recover pose and calc its err by comparing w ground truth     
        for mthd = 1:obj.numMethods   
          if strcmp(obj.algorithms{mthd},'EightPt') % Eight Point alg
            EOut          = eightp_Ver2_0(obj.matches.m1,obj.matches.m2);
            [ROut,tOut]   = TransformEssentialsVer2_0(EOut);          
            q             = R2Q(ROut);
          elseif strcmp(obj.algorithms{mthd},'Nister') % Five Point alg (Nister)
            EOut          = opengv('fivept_nister',obj.matches.m2u,obj.matches.m1u);
            [ROut,tOut]   = TransformEssentialsVer2_0(EOut);          
            q             = R2Q(ROut);
          elseif strcmp(obj.algorithms{mthd},'Li') % Five Point alg (Li + Hatley)
            EOut          = Ematrix5pt_v2(obj.matches.m2u(:,1:5),obj.matches.m1u(:,1:5));
            [ROut,tOut]   = TransformEssentialsVer2_0(EOut);
            q             = R2Q(ROut);
          elseif strcmp(obj.algorithms{mthd}, 'Kneip') % Five Point alg (Kneip)
            ROut          = opengv('fivept_kneip',1:5,obj.matches.m2u,obj.matches.m1u);  
            if ~isempty(ROut) % If a solution is returned
              q            = R2Q(ROut);
              [q,matchIdx] = FindClosetQVer2_2(obj.relPose.qr,q); % ...to gtruth 
              obj.rotErr(obj.cntr,mthd) = QuatError(obj.relPose.qr,q); % est err
            end
            % Kneip does not return a translation estimate, so continue to
            % the next iteration.
            continue;
          elseif strcmp(obj.algorithms{mthd},'Kukelova') % Five Point algorithm (Polynomial eigenvalue)
            EOut          = PolyEigWrapper(obj.matches.m1,obj.matches.m2);                
            [ROut,tOut]   = TransformEssentialsVer2_0(EOut);          
            q             = R2Q(ROut);
          elseif strcmp(obj.algorithms{mthd},'Stewenius') % Five Point algorithm (Stewenius)
            EOut          = opengv('fivept_stewenius',obj.matches.m2u,obj.matches.m1u); % Same results as fivep.m                
            [ROut,tOut]   = TransformEssentialsVer2_0(EOut);          
            q             = R2Q(ROut);
          elseif strcmp(obj.algorithms{mthd},'QuEst') % Five Point algorithm (QuEst)
            sol           = QuEst_Ver1_1(obj.matches.m1,obj.matches.m2);                
            q             = sol.Q;
            tOut          = sol.T;
          else
            error('Undefined algorithm.')
          end
          
          % find the closest transform to ground truth    
          [q,matchIdx]    = FindClosetQVer2_2(obj.relPose.qr,q);
          t               = FindClosetTrans(obj.relPose.tr,[tOut(:,matchIdx),...
            -tOut(:,matchIdx)]);   

          % calc est error
          obj.rotErr(obj.cntr,mthd)  = QuatError(obj.relPose.qr, q);
          obj.tranErr(obj.cntr,mthd) = TransError(obj.relPose.tr, t);            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
        if obj.matches_disp_en 
          fig = imshow(obj.Ip);
          hold on;
          p1 = obj.matches.p1;
          p2 = obj.matches.p2;
          numPts = size(p1,2);
          plot(p1(:,1), p1(:,2), 'g+');
          plot(p2(:,1), p2(:,2), 'yo');    
          for j = 1 : numPts
            if any(j == inliers)
              plot([p1(j,1) p2(j,1)], [p1(j,2) p2(j,2)], 'g'); % Detected inlier
            else
              plot([p1(j,1) p2(j,1)], [p1(j,2) p2(j,2)], 'r'); % Detected outlier
            end
          end
          drawnow;    
          hold off;
          if obj.matches_sav_en 
            fname = strcat(obj.test_outDir,'fig_matches_',obj.name,'.png');
            imwrite(fig,fname);
          end
        end 
        if obj.sliding_ref_en % store current frame as next state's prev frame 
          obj.Ip = obj.In;
          obj.ppoints = obj.npoints;                              
        end
        if mod(obj.cntr,10) == 0 % Print iteration number
          disp(['Iteration ' num2str(obj.cntr) ' of ' num2str(obj.numKeyFrames)]);
        end  
      end 
    end
    

    %% Backup/restore functions
    function s = save(obj)
      % Set properties in structure s to values in object obj
      % Set public properties and states
      s = save@matlab.System(obj);
      % Set private and protected properties
      %s.myproperty = obj.myproperty;     
    end
    function load(obj,s,wasLocked)
      % Set properties in object obj to values in structure s
      % Set private and protected properties
      obj.myproperty = s.myproperty;       
      % Set public properties and states
      load@matlab.System(obj,s,wasLocked);
    end
  end
  methods(Access = protected)
   
  end
end

classdef quest_class 
  % untitled2 Add summary here
  %
  % NOTE: When renaming the class name untitled2, the file name
  % and constructor name must be updated to use the class name.
  %
  % This template includes most, but not all, possible properties,
  % attributes, and methods that you can implement for a System object.

  %% public vars
  properties 
    %% configs (passed in via cfg obj or set manually)
    test_ID % unified name for test run 
    outDir % out dir     
    method 
    st_frame    = 1;% start frame index
    end_frame   = nan;% end frame index
    %% features (private)
%     features_sav_en   = true;
    matches_sav_en    = true;
    matches_disp_en   = false; % disp matched features between two keyframes
    sliding_ref_en    = false; % disable sliding_ref mode when running in real-time
    res_prt_en        = true; % print res in terminal  
    res_sav_en        = true; % save res table 
    %% quest configs
    dataroot    = [pwd '/data/']; % data path
    benchtype       = "KITTI"; % benchmark type (e.g. KITTI or TUM) 
    benchnum        = 3; % data sequence aux to KITTI   
    skipFrame       = 0; % num of frames skiped between two keyframes
    ranThresh   = 1e-6;% RANSAC Sampson dist threshold (for outliers)
    surfThresh     = 200; % SURF feature detection threshold
    maxPts          = 30; % max num of features used in pose est (fewer points, faster compute)
    minPts          = 8; % max num of features required (6 to est a unique pose with RANSAC)
    algorithms      = {...
%                        'EightPt'; 
%                        'Nister'; 
%                        'Kneip'; 
%                        'Kukelova'; 
%                        'Stewenius'; 
                       'QuEst'}; % algorithms to run ----> (disabled for now)
    benchmarks      = {'KITTI';
%                        'NAIST';
%                        'ICL';
%                        'TUM';
                       } % benchmarks ----> (disabled for now)
    % run-time variables 
    dataset % quest dataset obj

    posp % previous frame ground truth pose (per frame, given)
    numImag % total num of images
    keyFrames % keyframe indecies 
    numKeyFrames % num of keyframes
    numMethods  % num of algs used for comparison
    rotErr % rot err for each method
    tranErr % trans err for each method
    Q % recovered quaternions 
    T % recovered translations
    cntr % key frame counter
    ppoints_i % initial frame feature points 
    Ip_i % initial image
    ppoints % previous frame feature points 
    Ip % previous image
    npoints % current frame feature points 
    In % current image 
    matches % matches_class obj
    relPose % relative ground truth transform (bewteen frames)    
  end
  %% public consts 
%   properties(Nontunable) 
%   end
  %% private vars
%   properties(DiscreteState) 
%   end
  %% private consts
%   properties(Access = private) 
%   end
  methods
    %% constructor
    function obj = quest_class(namedArgs)
      arguments
        namedArgs.test_ID = "xxx";
        namedArgs.outDir = "xxx";
        namedArgs.method = "QuEst";
      end 
      obj.test_ID = namedArgs.test_ID;
      obj.outDir  = namedArgs.outDir;
      obj.method  = namedArgs.method;
    end    
  
  
    %% public functions
    function obj = load_cfg(obj, test_ID, ...
                            outDir, ...
                            method, ...
                            st_frame, ...
                            end_frame, ...
                            datDir, ...
                            benchtype, ...
                            seq, ...
                            skipFrame, ...
                            ransac_thresh, ...
                            surf_thresh, ...
                            maxPts, ...
                            minPts)
      %% common 
      obj.test_ID       = test_ID;
      obj.outDir        = outDir;
      obj.method        = method;
      obj.st_frame      = st_frame;
      obj.end_frame     = end_frame;
      % QuEst
      obj.dataroot      = datDir;
      obj.benchtype     = benchtype;
      obj.benchnum      = seq;
      obj.skipFrame     = skipFrame;
      obj.ranThresh     = ransac_thresh;
      obj.surfThresh    = surf_thresh;
      obj.maxPts        = maxPts;
      obj.minPts        = minPts;
      %% init
      obj = obj.init();
    end
  end
  methods(Access = public)
 function [obj,t,q] = get_pose(obj,i)
      obj.cntr  = obj.cntr + 1; % Counter          
      % get current frame features and match w prev frame    
      [obj.npoints,obj.In] = GetFeaturePoints(i,obj.dataset,obj.surfThresh);            
      obj.matches   = MatchFeaturePoints(obj.Ip,obj.ppoints,obj.In,obj.npoints,...
        obj.maxPts,obj.dataset,i,obj.matches_disp_en,obj.matches_sav_en, ...
        obj.outDir);
      % relative ground truth transformation and previous frame gt pose
      [obj.relPose,obj.posp] = RelativeGroundTruth(i,obj.posp,obj.dataset);
      % skip frame if not enough matches found 
      if (obj.matches.numPts < obj.minPts) % || (obj.matches.status~=0)         
        obj.Ip      = obj.In; % use current frame as the next previous frame
        obj.ppoints = obj.npoints;         
        disp('Not enough matched feature points. Frame skipped!');
      else
        % recover pose and calc its err by comparing w ground truth     
        for mthd = 1:obj.numMethods   
          if strcmp(obj.algorithms{mthd},'QuEst_RANSAC_v0102') % QuEst w RANSAC
            [M, inliers]  = QuEst_RANSAC_Ver1_2(obj.matches.m1,obj.matches.m2,...
              obj.ranThresh);   
            q             = M.Q;
            tOut          = M.t;
          elseif strcmp(obj.algorithms{mthd},'QuEst_v0708') 
            kp1           = obj.matches.m1(:,1:5);
            kp2           = obj.matches.m2(:,1:5);
            q             = QuEst_5Pt_Ver7_8(kp1,kp2,obj.cntr);
            tOut          = FindTransDepth_Ver1_0(kp1,kp2,q);
          elseif strcmp(obj.algorithms{mthd},'EightPt') % Eight Point alg
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
          t               = FindClosetTrans(obj.relPose.tr,[tOut,-tOut]);   

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
            fname = strcat(obj.outDir,'fig_matches_',obj.name,'.png');
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
    function res = get_res(obj)
      % remove NaN entries (corresponding to skipped frames)
      nanIdx = find(sum(isnan(obj.rotErr),2));
      obj.rotErr(nanIdx,:)  = [];
      obj.tranErr(nanIdx,:) = [];
      % statistics of error for rotation and translation estimates
      rotErrM     = mean(obj.rotErr,1);           % Mean
      rotErrS     = std(obj.rotErr,1);            % Standard deviation
      rotErrMd    = median(obj.rotErr,1);         % Median
      rotErrQ1    = quantile(obj.rotErr,0.25, 1); % 25% quartile
      rotErrQ3    = quantile(obj.rotErr,0.75, 1); % 75% quartile
      tranErrM    = mean(obj.tranErr,1);
      tranErrS    = std(obj.tranErr,1);
      tranErrMd   = median(obj.tranErr,1);
      tranErrQ1   = quantile(obj.tranErr,0.25, 1);
      tranErrQ3   = quantile(obj.tranErr,0.75, 1);
      % Table of results
      RowNames = {'Rot err mean';...
                  'Rot err std';...
                  'Rot err median'; 
                  'Rot err Q_1';...
                  'Rot err Q_3';...
                  'Tran err mean';...
                  'Tran err std';...
                  'Tran err median';...
                  'Tran err Q_1';...
                  'Tran err Q_3'};
      data      = [rotErrM;
                   rotErrS;
                   rotErrMd;
                   rotErrQ1;
                   rotErrQ3;
                   tranErrM;
                   tranErrS;
                   tranErrMd;
                   tranErrQ1;
                   tranErrQ3;];
      res  = table(data(:,1), ...
                   'RowNames', RowNames, ...
                   'VariableNames', obj.algorithms); 
      if obj.res_prt_en
        disp(res);
      end 
      if obj.res_sav_en
        fname = strcat(obj.outDir,'res_',obj.test_ID,'_tab.csv');
        writetable(res,fname);
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
    function obj = init(obj)
      [obj.dataset,obj.posp] = LoadDataset(obj.dataroot,obj.benchtype,...
        obj.benchnum,obj.st_frame,obj.end_frame);
      if strcmp(obj.benchtype,'KITTI')
        obj.skipFrame = 1; 
      elseif strcmp(obj.benchtype,'NAIST')
        obj.skipFrame = 1; 
      elseif strcmp(obj.benchtype,'ICL')
        obj.skipFrame = 1; 
      elseif strcmp(obj.benchtype,'TUM')
        obj.skipFrame = 1;     
      end
      obj.numImag       = length(obj.dataset.fnames); 
      obj.keyFrames     = 2+obj.skipFrame:1+obj.skipFrame:obj.numImag; 
      obj.numKeyFrames  = length(obj.keyFrames); 
      obj.numMethods    = length(obj.algorithms); 
      obj.rotErr        = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.tranErr       = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.Q             = cell(obj.numKeyFrames,obj.numMethods); 
      obj.T             = cell(obj.numKeyFrames,obj.numMethods);

      [obj.ppoints_i, obj.Ip_i] = GetFeaturePoints(obj.st_frame,obj.dataset,...
        obj.surfThresh);
      obj.cntr          = 0;
      obj.ppoints       = obj.ppoints_i;
      obj.Ip            = obj.Ip_i;
    end  
  end
end

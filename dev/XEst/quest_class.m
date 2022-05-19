classdef quest_class < matlab.System % & config_class
  properties % public vars
    % configs (passed in via cfg obj or set manually)
    % features (private constants)
    features_sav_en   = true;
    matches_sav_en    = true;
    matches_disp_en   = false; % disp matched features between two keyframes
    sliding_ref_en    = false; % disable sliding_ref mode when running in real-time
    % configs (private constants)
    ranThresh       = 1e-6;% RANSAC Sampson dist threshold (for outliers)
    surfThresh      = 200; % SURF feature detection threshold
    maxPts              = 30; % max num of features used in pose est (fewer points, faster compute)
    minPts              = 8; % max num of features required (6 to est a unique pose with RANSAC)
    %% overwritten by cfg
    test_ID
    test_outDir
    algorithms      = {...
                                     'EightPt'; 
                                     'Nister'; 
                                     %'Kneip'; 
                                     'Kukelova'; 
                                     %'Stewenius'; 
                                     'QuEst'}; % algorithms to run ----> (disabled for now)

    benchmarks        = {'KITTI';
                                      'NAIST';
                                      'ICL';
                                      'TUM';  } % benchmarks ----> (disabled for now)
    %% runtime vars (private)
    pose_est_buff  % buffer that holds all pose est from diff methods
    numBenchmarks 
    numMethods  % num of algs used for comparison 
  end

  methods % constructor
    function obj = quest_class(varargin)
        setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end    
  end
 
  methods (Access = public)  % public functions
    function load_cfg(obj, cfg) %,  extraprop)
      obj.test_ID                   =  cfg.test_ID;                   
      obj.test_outDir            =  cfg.test_outDir;              
      obj.algorithms             =  cfg.pose_algorithms;             
      obj.benchmarks          =  cfg.benchmarks;          
      obj.surfThresh            =  cfg.quest_surfThresh;       
      obj.init();
    end

    function [TQ_sols, Q_quest] = get_pose(obj, kframe_idx, dat, cfg)
      Q_quest = nan;
      % get current frame features and match w prev frame    
      [dat.npoints,dat.In] = GetFeaturePoints(kframe_idx, dat.dataset, dat.surfThresh);            
      dat.matches   = MatchFeaturePoints(dat.Ip, dat.ppoints, dat.In, dat.npoints, ...
                                                                    obj.maxPts, dat.dataset, kframe_idx, ...
                                                                    obj.matches_disp_en, ... 
                                                                    obj.matches_sav_en, cfg.test_outDir);
      % relative ground truth transformation and previous frame gt pose
      [dat.relPose,dat.posp] = RelativeGroundTruth(kframe_idx, dat.posp, dat.dataset);
      
      % skip frame if not enough matches found 
      if (dat.matches.numPts < obj.minPts) % || (obj.matches.status~=0)         
        dat.Ip      = dat.In; % use current frame as the next previous frame
        dat.ppoints = dat.npoints;         
        disp('Not enough matched feature points. Frame skipped!');
        %dat.Frame_skipped_flag = true;
      else
        
        % recover pose and calc its err by comparing w ground truth     
        for mthd = 1: obj.numMethods   
          if strcmp(obj.algorithms{mthd},'EightPt') % Eight Point alg
            EOut          = eightp_Ver2_0(dat.matches.m1,dat.matches.m2);
            [ROut, tOut]   = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(obj.algorithms{mthd},'Nister') % Five Point alg (Nister)
            EOut          = opengv('fivept_nister',dat.matches.m2u,dat.matches.m1u);
            [ROut, tOut]   = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(obj.algorithms{mthd},'Li') % Five Point alg (Li + Hatley)
            EOut          = Ematrix5pt_v2(dat.matches.m2u(:,1:5),dat.matches.m1u(:,1:5));
            [ROut, tOut]   = TransformEssentialsVer2_0(EOut);
            Q             = R2Q(ROut);
          elseif strcmp(obj.algorithms{mthd}, 'Kneip') % Five Point alg (Kneip)
            ROut          = opengv('fivept_kneip',1:5,dat.matches.m2u,dat.matches.m1u);  
            if ~isempty(ROut) % If a solution is returned
              Q            = R2Q(ROut);
              [Q,matchIdx] = FindClosetQVer2_2(dat.relPose.qr, Q); % ...to gtruth 
              %dat.rotErr(dat.cntr,mthd) = QuatError(dat.relPose.qr, q); % est err
            end
            % Kneip does not return a translation estimate, so continue to
            % the next iteration.
            continue;
          elseif strcmp(obj.algorithms{mthd},'Kukelova') % Five Point algorithm (Polynomial eigenvalue)
            EOut          = PolyEigWrapper(dat.matches.m1,dat.matches.m2);                
            [ROut, tOut]   = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(obj.algorithms{mthd},'Stewenius') % Five Point algorithm (Stewenius)
            EOut          = opengv('fivept_stewenius',dat.matches.m2u,dat.matches.m1u); % Same results as fivep.m                
            [ROut, tOut]   = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(obj.algorithms{mthd},'QuEst') % Five Point algorithm (QuEst)
            sol           = QuEst_Ver1_1(dat.matches.m1, dat.matches.m2);                
            Q             = sol.Q;
            tOut          = sol.T;
            
            [Q, matchIdx]    = FindClosetQVer2_2(dat.relPose.qr, Q);
            T                = FindClosetTrans(dat.relPose.tr, [tOut(:,matchIdx), -tOut(:,matchIdx)]);  
            Q_quest = Q;
            disp('Q_quest'); disp(Q_quest); 
          else
            error('Undefined algorithm.')
          end
          
          % find the closest transform to ground truth    
          %[Q, matchIdx]    = FindClosetQVer2_2(dat.relPose.qr, Q);
          %T                = FindClosetTrans(dat.relPose.tr, [tOut(:,matchIdx), -tOut(:,matchIdx)]);   
          %T = -tOut;
         
          obj.save_method_est(Q, T, mthd);
          % calc est error
          %dat.tranErr(dat.cntr,mthd) = TransError(dat.relPose.tr, t);            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
        if obj.matches_disp_en 
          fig = imshow(dat.Ip);
          hold on;
          p1 = dat.matches.p1;
          p2 = dat.matches.p2;
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
            fname = strcat(obj.test_outDir, 'fig_matches_', obj.test_ID,'.png');
            imwrite(fig,fname);
          end
        end 
        if obj.sliding_ref_en % store current frame as next state's prev frame 
          dat.Ip = dat.In;
          dat.ppoints = dat.npoints;                              
        end
        %if mod(frame_idx,10) == 0 % Print iteration number
        %  disp(['Iteration ' num2str(frame_idx) ' of ' num2str(dat.numKeyFrames)]);
        %end  
      end 

      TQ_sols = obj.pose_est_buff;
    end %[T, Q] = get_pose(obj, i, dat, cfg)

    function res = get_res(obj)
      %% quest
      % remove NaN entries (corresponding to skipped frames)
      nanIdx = find(sum(isnan(obj.Q_err_hist),2));
      obj.Q_err_hist(nanIdx,:)  = [];
      obj.T_err_hist(nanIdx,:) = [];
      % statistics of error for rotation and translation estimates
      rotErrM     = mean(obj.Q_err_hist,1);           % Mean
      rotErrS     = std(obj.Q_err_hist,1);            % Standard deviation
      rotErrMd    = median(obj.Q_err_hist,1);         % Median
      rotErrQ1    = quantile(obj.Q_err_hist,0.25, 1); % 25% quartile
      rotErrQ3    = quantile(obj.Q_err_hist,0.75, 1); % 75% quartile
      tranErrM    = mean(obj.T_err_hist,1);
      tranErrS    = std(obj.T_err_hist,1);
      tranErrMd   = median(obj.T_err_hist,1);
      tranErrQ1   = quantile(obj.T_err_hist,0.25, 1);
      tranErrQ3   = quantile(obj.T_err_hist,0.75, 1);

      RowNames  = {'Rot err mean';...
                                'Rot err std';...
                                'Rot err median'; 
                                'Rot err Q_1';...
                                'Rot err Q_3';...
                                'Tran err mean';...
                                'Tran err std';...
                                'Tran err median';...
                                'Tran err Q_1';...
                                'Tran err Q_3'};
      data  = [rotErrM;
                   rotErrS;
                   rotErrMd;
                   rotErrQ1;
                   rotErrQ3;
                   tranErrM;
                   tranErrS;
                   tranErrMd;
                   tranErrQ1;
                   tranErrQ3;];

      if obj.numMethods == 1
        res  = table(data(:,1), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 2
        res  = table(data(:,1), ...
                     data(:,2), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 3
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 4
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     data(:,4), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 5
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     data(:,4), ...
                     data(:,5), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms);    
      end

      if obj.res_prt_en
        disp(res);
      end 
      if obj.res_sav_en
        fname = strcat(obj.test_outDir, 'res_', obj.test_ID, '_QuEst_table.csv');
        writetable(res, fname);
      end 
    end % end of get_res()

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
  methods (Access = private)
    function init(obj)
      obj.numBenchmarks    = length(obj.benchmarks);
      obj.numMethods          = length(obj.algorithms);
      obj.pose_est_buff        = cell(obj.numMethods, 3);
    end 

    function save_method_est(obj, Q, T, mthd)
      obj.pose_est_buff{mthd, 1}     =    Q;
      obj.pose_est_buff{mthd, 2}     =    T;
      obj.pose_est_buff{mthd, 3}     =    obj.algorithms(mthd);
    end

    function normed = normalize(vector)
      normed  = vector/ norm(vector); % normalize v answer 
        if(normed(3)<0) 
            normed = - normed; 
        end 
    end
  end % methods (Access = private)
end

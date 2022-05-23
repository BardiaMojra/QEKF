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
                                     %'Kneip';  % dep on opengv
                                     'Kukelova'; 
                                     %'Stewenius';  % dep on opengv
                                     'QuEst'}; % algorithms to run ----> (disabled for now)

    benchmarks        = {'KITTI';
                                      'NAIST';
                                      'ICL';
                                      'TUM';  } % benchmarks ----> (disabled for now)
    %% runtime vars (private)
    pose_est_buff  % buffer that holds all pose est from diff methods
    numBenchmarks 
    numMethods  % num of algs used for comparison 
    %% private constants 
    RowNames  = {'Rot err mean     ';
                              'Rot err std         ';
                              'Rot err median  '; 
                              'Rot err Q_1        ';
                              'Rot err Q_3        ';
                              'Tran err mean    ';
                              'Tran err std        ';
                              'Tran err median ';
                              'Tran err Q_1       ';
                              'Tran err Q_3       '};
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

    function [TQ_sols] = get_pose(obj, kframe_idx, dat, cfg)
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
      else
        % recover pose and calc its err by comparing w ground truth     
        for alg = 1: length(obj.algorithms)
          method = obj.algorithms{alg};
          if strcmp(method, 'EightPt') % Eight Point alg
            EOut          = eightp_Ver2_0(dat.matches.m1,dat.matches.m2);
            [ROut, tOut]   = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(method, 'Nister') % Five Point alg (Nister)
            EOut          = opengv('fivept_nister',dat.matches.m2u,dat.matches.m1u);
            [ROut, tOut]   = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(method, 'Li') % Five Point alg (Li + Hatley)
            EOut          = Ematrix5pt_v2(dat.matches.m2u(:,1:5),dat.matches.m1u(:,1:5));
            [ROut, tOut]   = TransformEssentialsVer2_0(EOut);
            Q             = R2Q(ROut);
          elseif strcmp(method,  'Kneip') % Five Point alg (Kneip)
            ROut          = opengv('fivept_kneip',1:5,dat.matches.m2u,dat.matches.m1u);  
            if ~isempty(ROut) % If a solution is returned
              Q            = R2Q(ROut);
              [Q, matchIdx] = FindClosetQVer2_2(dat.relPose.qr, Q); % ...to gtruth 
              %dat.rotErr(dat.cntr,mthd) = QuatError(dat.relPose.qr, q); % est err
            end
            % Kneip does not return a translation estimate, so continue to
            % the next iteration.
            continue;
          elseif strcmp(method, 'Kukelova') % Five Point algorithm (Polynomial eigenvalue)
            EOut          = PolyEigWrapper(dat.matches.m1,dat.matches.m2);                
            [ROut, tOut]   = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(method, 'Stewenius') % Five Point algorithm (Stewenius)
            EOut          = opengv('fivept_stewenius',dat.matches.m2u,dat.matches.m1u); % Same results as fivep.m                
            [ROut, tOut]   = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(method, 'QuEst') % Five Point algorithm (QuEst)
            sol           = QuEst_Ver1_1(dat.matches.m1, dat.matches.m2);                
            Q              = obj.normalize_all(sol.Q);
            tOut          = sol.T;
          else
            error('Undefined algorithm.')
          end
          
          % find the closest transform to ground truth    
          [Q, matchIdx]    = FindClosetQVer2_2(dat.relPose.qr, Q);
          T                         = FindClosetTrans(dat.relPose.tr, [tOut(:,matchIdx), -tOut(:,matchIdx)]);   
          obj.save_method_est(Q, T, alg);
          
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if obj.matches_disp_en  % todo: move 
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

    function res = get_res(obj, cfg, dlog)
      res = cell( dlog.numBenchmarks, 2);
      for b = 1:length(obj.benchmarks) % for each test set (benchmark)
        msg = sprintf("obj.benchmarks(%d): %s  DOES NOT match dlog.logs{%d}.benchtype: %s", ...
          b, obj.benchmarks{b}, b, dlog.logs{b}.benchtype);
        assert(strcmp(obj.benchmarks{b}, dlog.logs{b}.benchtype), msg);       
        
        % calc per frame err for 
        dat = cfg.dats{b};
        log = dlog.logs{b};
        res{b, 1}   =   dlog.logs{b}.benchtype;
        res{b, 2}   =   obj.get_log_res(log, dat);  % returns a table object
        
        if dlog.res_prt_en
          disp(res{b, 1}); disp(res{b, 2});
        end 
        
        if dlog.res_sav_en
          btag = [ '_' res{b, 1} '_' ];
          fname = strcat(obj.test_outDir, 'res_', obj.test_ID, btag, '_QuEst_table.csv');
          writetable(res{b, 2}, fname);
        end 
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
      obj.pose_est_buff        = cell(3, obj.numMethods);
    end 
    
    function res_table = get_log_res(obj, log, dat) % get per benchmark log errs 
      
      benchtype = dat.dataset.benchtype;
      qTru       = dat.dataset.qTru;
      tTru        = dat.dataset.tTru;
      cntr = 0;
      q1 = dat.posp_i.q1;
      t1  = dat.posp_i.t1;
      
      for f = dat.keyFrames
        cntr = cntr + 1;
        % get current pose
        if strcmp(benchtype, 'KITTI') || strcmp(benchtype, 'ICL') || strcmp(benchtype, 'NAIST')
          q2 = qTru(:, f);
          t2  =  tTru(:, f); 

        elseif strcmp(benchtype, 'TUM')  
          fname = dataset.fnames{f};
          ftime = str2double( fname(1:end-4) ); % Time at the current frame       
          [q2, t2] = InterpPoseVer1_1(ftime, dataset.times, qTru, tTru); % Interpolate data to find the pose of the current camera frame  
        else
          error('Undefined dataset.')
        end
      
        % compute incrementation pose wrt prev frame a.k.a. relative pose 
        if strcmp(benchtype, 'KITTI')  || strcmp(benchtype, 'ICL')  || strcmp(benchtype, 'TUM')
          % Relative rotation between two frames (^2R_1 : rotation of frame 1 given in frame 2)
          qr = QuatMult(QuatConj(q2), q1); 
          % Relative trans. vec. in current coord. frame (^2t_21 : relative trans given in frame 2)
          tr  = Q2R(QuatConj(q2)) * (t2 - t1); 
        elseif strcmp(benchtype, 'NAIST') 
          % In this dataset the absolute pose is given in the camera coordinate frame, i.e.,  
          % c^R_w, c^t_w.(This is opposite of what is claimed in their website, unfortunately!)
          % Relative rotation between two frames (^2R_1 : rotation of frame 1 given in frame 2)
          qr = QuatMult(q2,QuatConj(q1)); 
          % Relative trans. vec. in current coord. frame (^2t_21 : relative trans given in frame 2)
          tr = t2 - Q2R(q2)*Q2R(QuatConj(q1)) * t1;  
        end
         
        % calc and save errs per method
        for alg = 1:length(log.algorithms)
          q                                     = log.Q_hist{cntr, alg};
          t                                      = log.T_hist{cntr, alg};
          log.Q_errs(cntr, alg)     = QuatError(qr, q);
          log.T_errs(cntr, alg)      = TransError(tr, t);
        end % for alg = length(log.algorithms)

        q1 = q2;   t1 = t2;  % store frame pose for the next keyFrame 
      end % for f = kframes

      % statistics of error for rotation and translation estimates
      rotErrM     = mean(log.Q_errs,1);           % Mean
      rotErrS     = std(log.Q_errs,1);            % Standard deviation
      rotErrMd    = median(log.Q_errs,1);         % Median
      rotErrQ1    = quantile(log.Q_errs,0.25, 1); % 25% quartile
      rotErrQ3    = quantile(log.Q_errs,0.75, 1); % 75% quartile
      
      tranErrM    = mean(log.T_errs,1);
      tranErrS    = std(log.T_errs,1);
      tranErrMd   = median(log.T_errs,1);
      tranErrQ1   = quantile(log.T_errs,0.25, 1);
      tranErrQ3   = quantile(log.T_errs,0.75, 1);
      
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

      res_table  = obj.get_res_table(data);
    end % function get_log_errs(log, dat) 

    function res_table = get_res_table(obj, data)
      if obj.numMethods == 1
        res_table  = table(data(:,1), ...
                                      'RowNames', obj.RowNames, ...
                                      'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 2
        res_table  = table(data(:,1), ...
                                       data(:,2), ...
                                       'RowNames', obj.RowNames, ...
                                       'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 3
        res_table  = table(data(:,1), ...
                                      data(:,2), ...
                                      data(:,3), ...
                                      'RowNames', obj.RowNames, ...
                                      'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 4
        res_table  = table(data(:,1), ...
                                      data(:,2), ...
                                      data(:,3), ...
                                      data(:,4), ...
                                      'RowNames', obj.RowNames, ...
                                      'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 5
        res_table  = table(data(:,1), ...
                                      data(:,2), ...
                                      data(:,3), ...
                                      data(:,4), ...
                                      data(:,5), ...
                                      'RowNames', obj.RowNames, ...
                                      'VariableNames', obj.algorithms);    
      end
    end %  function res_table = get_res_table(obj, data)

    function save_method_est(obj, Q, T, alg) % save res from all methods per frame
      obj.pose_est_buff{1, alg}     =    obj.algorithms(alg);
      obj.pose_est_buff{2, alg}     =    Q;
      obj.pose_est_buff{3, alg}     =    T;
    end

    function normed = normalize(~, vector)
      normed  = vector/ vecnorm(vector); % normalize v answer 
      if normed(3) < 0 
          normed = - normed; 
      end 
    end

    function normedv = normalize_all(obj, vectors)
      normedv  = zeros(size(vectors));
      for v = 1: size(vectors, 2)
        normedv(:, v) = obj.normalize(vectors(:, v));
      end
    end
  
  end % methods (Access = private)
end

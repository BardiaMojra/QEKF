classdef quest_class < matlab.System 
  properties % public vars
    % features (private constant flags)
    res_tab_sav_en        = true;
    res_tab_prt_en        = false;
    all_feat_sav_en       = false;
    all_feat_disp_en      = false;
    masked_feat_sav_en    = false;
    masked_feat_disp_en   = false;
    sliding_ref_en        % disable sliding_ref mode when running in real-time
    % configs (private constants)
    ranThresh         = 1e-6 % RANSAC Sampson dist threshold (for outliers)
    surfThresh        = 200 % SURF feature detection threshold
    maxPts            = 8 % max num of features used in pose est (fewer points, faster compute)
    minPts            = 8 % max num of features required (6 to est a unique pose with RANSAC)
    normThresh        = 1.5
    % argin from cfg
    TID
    ttag
    toutDir
    benchmark
    vel_algs
    pos_algs
    del_T
    % runtime vars (private)
    TQVW_sols  % buffer that holds all pose est from diff methods
    res
    pos_numMethods  % num of algs used for comparison 
    vel_numMethods
    % private constants 
    T_RowNames  = {'T err mean';
                   'T err std';
                   'T err med'; 
                   'T err Q1';
                   'T err Q3';};
    Q_RowNames  = {'Q err mean';
                   'Q err std';
                   'Q err med'; 
                   'Q err Q1';
                   'Q err Q3';};
    % rpt constants 
    mod_name      = "QuEst+"
    rpt_note      = "This module contains multipl"
  end
  methods % constructor
    
    function obj = quest_class(varargin)
        setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end    
  
  end
  methods (Access = public)  % public functions
    
    function load_cfg(obj, cfg) %,  extraprop)
      obj.TID               =  cfg.TID;
      obj.ttag              =  cfg.ttag;
      obj.toutDir           =  cfg.toutDir;              
      obj.pos_algs          =  cfg.pos_algs;
      obj.vel_algs          =  cfg.vel_algs;
      obj.del_T             =  cfg.del_T;
      obj.benchmark         =  cfg.benchmark;          
      obj.surfThresh        =  cfg.surfThresh;
      obj.sliding_ref_en    =  cfg.sliding_ref_en;
      obj.init();
    end

    function TQVW_sols = get_pose(obj, kfi, dat)
      if (dat.matches.numPts < obj.minPts) % || (obj.matches.status~=0)
        dat.Ip      = dat.In; % use current frame as the next previous frame
        dat.ppoints = dat.npoints;         
        disp('Not enough matched feature points. Frame skipped!');
      else
        % recover pose and calc its err by comparing w ground truth     
        for a = 1:length(obj.pos_algs)
          alg = obj.pos_algs{a};
          if strcmp(alg, 'EightPt') % Eight Point alg
            EOut          = eightp_Ver2_0(dat.matches.m1,dat.matches.m2);
            [ROut, tOut]  = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(alg, 'Nister') % Five Point alg (Nister)
            EOut          = opengv('fivept_nister',dat.matches.m2u,dat.matches.m1u);
            [ROut, tOut]  = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(alg, 'Li') % Five Point alg (Li + Hatley)
            EOut          = Ematrix5pt_v2(dat.matches.m2u(:,1:5),dat.matches.m1u(:,1:5));
            [ROut, tOut]  = TransformEssentialsVer2_0(EOut);
            Q             = R2Q(ROut);
          elseif strcmp(alg,  'Kneip') % Five Point alg (Kneip)
            ROut          = opengv('fivept_kneip',1:5,dat.matches.m2u,dat.matches.m1u);  
            if ~isempty(ROut) % If a solution is returned
              Q             = R2Q(ROut);
              [Q, matchIdx] = FindClosetQVer2_2(dat.relPose.qr,Q); % to gtruth 
              %dat.rotErr(dat.cntr,mthd) = QuatError(dat.relPose.qr, q); % est err
            end
            % Kneip does not return a translation estimate, so continue to
            % the next iteration.
            continue;
          elseif strcmp(alg, 'Kukelova') % Five Point algorithm (Polynomial eigenvalue)
            EOut          = PolyEigWrapper(dat.matches.m1,dat.matches.m2);                
            [ROut, tOut]  = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(alg, 'Stewenius') % Five Point algorithm (Stewenius)
            EOut          = opengv('fivept_stewenius',dat.matches.m2u,dat.matches.m1u); % Same results as fivep.m                
            [ROut, tOut]  = TransformEssentialsVer2_0(EOut);          
            Q             = R2Q(ROut);
          elseif strcmp(alg, 'QuEst') % Five Point algorithm (QuEst)
            sol           = QuEst_Ver1_1(dat.matches.m1, dat.matches.m2);                
            Q             = normalize_all(sol.Q);
            tOut          = sol.T;
          elseif strcmp(alg, 'VEst') 
            % pass, VEst is computed by a separate class
          else
            error('Undefined algorithm.')
          end
          Q = check_quats(Q);
          tOut = normalize_tOuts(tOut);
          % find the closest transform to ground truth    
          [Q, matchIdx]    = FindClosetQVer2_2(dat.relPose.qr, Q);
          T    = FindClosetTrans(dat.relPose.tr, [tOut(:,matchIdx), -tOut(:,matchIdx)]);   
          assert(sqrt(sum(T.*T))<obj.normThresh,"[quest.run]->> T not normalized!");
          assert(sqrt(sum(Q.*Q))<obj.normThresh,"[quest.run]->> Q not normalized!");
          obj.save_method_sols(a, T, Q);
        end
        if obj.masked_feat_disp_en  
          imshow(dat.Ip);
          hold on;
          p1 = dat.matches.p1;
          p2 = dat.matches.p2;
          numPts = size(p1,2);
          plot(p1(:,1), p1(:,2), 'g+');
          plot(p2(:,1), p2(:,2), 'yo');    
          for j = 1 : numPts
            %if any(j == inliers)
            plot([p1(j,1) p2(j,1)], [p1(j,2) p2(j,2)], 'g'); % Detected inlier
            %else
            %plot([p1(j,1) p2(j,1)], [p1(j,2) p2(j,2)], 'r'); % Detected outlier
            %end
          end
          drawnow;    
          hold off;
          if obj.masked_feat_sav_en
            fname = strcat(obj.toutDir, "fig_mask-mat-feat_", ...
                             num2str(kfi, '%04d'), '.png');
            print(fname, '-dpng');
          end
        end % if obj.masked_feat_disp_en  

        if obj.sliding_ref_en % store current frame as next state's prev frame 
          dat.Ip = dat.In;
          dat.ppoints = dat.npoints;                              
        end
      end 
      TQVW_sols = obj.TQVW_sols;
    end % function TQVW_sols = get_pose(obj, kframe_idx, dat)

    function res = get_res(obj, cfg, dlog)
      obj.res{1}   = dlog.log.btype;
      obj.res{2}   = obj.get_res_tab(dlog.log, cfg.dat); % returns a table object
      if obj.res_tab_prt_en
        disp(strcat(obj.mod_name, ' module:')); disp(obj.rpt_note);
        disp(obj.res{1}); disp(obj.res{2});
      end 
      if obj.res_tab_sav_en
        fname = strcat(obj.toutDir,'res_',obj.mod_name,'_tab.csv');
        writetable(obj.res{2}, fname);
      end 
      res = obj.res;
    end      
    
    function s = save(obj) % Backup/restore functions
      s = save@matlab.System(obj);
      %s.myproperty = obj.myproperty; % Set private and protected properties
    end

    function load(obj,s,wasLocked)
      obj.myproperty = s.myproperty;    
      load@matlab.System(obj,s,wasLocked); % Set public properties and states
    end
  
  end
  methods (Access = private)
    
    function init(obj)
      obj.pos_numMethods      = length(obj.pos_algs);
      obj.vel_numMethods      = length(obj.vel_algs);
      obj.TQVW_sols           = cell(5, obj.pos_numMethods); % alg,T,Q,V,W
      obj.res                 = cell(3, 0); % btype, res_tab, log_fig
    end 
    
    function res_table = get_res_tab(obj, log, dat) % get per benchmark log errs 
      btype   = dat.dataset.benchtype;
      qTru    = dat.dataset.qTru;
      tTru    = dat.dataset.tTru;
      cntr    = 0;
      q1      = dat.posp_i.q1;
      t1      = dat.posp_i.t1;
      for f = dat.kframes
        cntr = cntr + 1;
        [tr,qr,t2,q2] = get_relGT(f, btype, tTru, qTru, t1, q1);
        for a = 1:length(log.pos_algs) % calc errs per pose alg
          q  = log.Q_hist(cntr,((a-1)*log.d_Q)+1:((a-1)*log.d_Q)+log.d_Q)';
          t  = log.T_hist(cntr,((a-1)*log.d_T)+1:((a-1)*log.d_T)+log.d_T)';
          log.Q_errs(cntr, a)   = QuatError(qr, q);
          log.T_errs(cntr, a)   = TransError(tr, t);
        end 
        q1 = q2; % store frame pose for the next keyFrame 
        t1 = t2; 
      end % for f = dat.kframes
      T_stats     = obj.get_stats(log.T_errs);
      Q_stats     = obj.get_stats(log.Q_errs);
      T_res_tab   = obj.get_res_table(T_stats, obj.T_RowNames);
      Q_res_tab   = obj.get_res_table(Q_stats, obj.Q_RowNames);
      res_table   = vertcat(T_res_tab, Q_res_tab);
    end % get_res_tab(log, dat) 

    function res_table = get_res_table(obj, data, RowNames)
      if obj.pos_numMethods == 1
        res_table  = table(data(:,1), ...
                          'RowNames', RowNames, ...
                          'VariableNames', obj.pos_algs); 
      elseif obj.pos_numMethods == 2
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.pos_algs); 
      elseif obj.pos_numMethods == 3
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                          'RowNames', RowNames, ...
                          'VariableNames', obj.pos_algs); 
      elseif obj.pos_numMethods == 4
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                           data(:,4), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.pos_algs); 
      elseif obj.pos_numMethods == 5
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                           data(:,4), ...
                           data(:,5), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.pos_algs);    
      end
    end %  function res_table = get_res_table(obj, data)

    function stats = get_stats(~, errs) 
      stats       = zeros(5, size(errs,2));
      stats(1, :) = mean(errs,1);  % Mean
      stats(2, :) = std(errs,1);  % Standard deviation
      stats(3, :) = median(errs,1); % Median
      stats(4, :) = quantile(errs, 0.25, 1); % 25% quartile
      stats(5, :) = quantile(errs, 0.75, 1); % 75% quartile
    end

    function save_method_sols(obj, alg, T, Q) % save res from all methods per frame
      obj.TQVW_sols{1, alg}     =    obj.pos_algs(alg);
      obj.TQVW_sols{2, alg}     =    T;
      obj.TQVW_sols{3, alg}     =    Q;
    end

  end % methods (Access = private)
end

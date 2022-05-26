classdef qekf_handler_class < matlab.System 
  properties % public vars
    % features 
    
    % cfg argin    
    test_ID
    test_outDir
    benchmark
    pose_algorithms
    % local vars
    numMethods
    trackers
    st_sols
    
    %% other private constants 
    metNames  = {'mean';
                 'std';
                 'median'; 
                 'Q_1';
                 'Q_3';};
    tabNames = {'GT-X T err';
                'GT-X Q err';
                'GT-X V err';
                'Z-XH T L1';
                'Z-XH Q L1';
                'Z-XH V L1';
                'Z-XH T L2';
                'Z-XH Q L2';
                'Z-XH V L2';};
  end

  methods % constructor
    function obj = qekf_handler_class(varargin) 
      setProperties(obj, nargin, varargin{:}) % init obj w name-value args
    end  
  end % methods % constructor
  
  methods (Access = public) 
    function load_cfg(obj, cfg)
      obj.test_ID           = cfg.test_ID;                   
      obj.test_outDir       = cfg.test_outDir;       
      obj.benchmark         = cfg.benchmark;
      obj.pose_algorithms   = cfg.pose_algorithms;
      obj.numMethods        = length(obj.pose_algorithms);
      obj.trackers          = cell(obj.numMethods, 0);
      obj.st_sols           = cell(7,obj.numMethods); % alg,Z,U,X,Y,P,K 
      for alg = 1:obj.numMethods
        obj.trackers{alg} = qekf_class( alg_idx = alg );
        obj.trackers{alg}.load_cfg(cfg);
      end
    end

    function st_sols = run_filter(obj, TQVW_sols)
      for alg = 1:obj.numMethods
        st_sol = obj.trackers{alg}.run_qekf(TQVW_sols, alg);
        for s = 1:length(st_sol)
          obj.st_sols{s, alg} = st_sol{s};
        end
      end 
      st_sols = obj.st_sols;
    end 

    function supTab = get_res(obj, cfg, dlog)
      benchName  = dlog.log.benchtype;
      res_tabs   = obj.get_log_res(dlog.log, cfg.dat);  % returns a table object
     
      if dlog.res_prt_en
        disp("QEKF module:"); 
        disp(benchName); 
        for tab = 1:length(res_tabs)
          disp(res_tabs{tab});
        end
      end 
      
      if dlog.res_sav_en
        supTab = res_tabs{1};
        for tab = 1:length(res_tabs)-1
          supTab = [supTab; res_tabs{tab+1}];
        end
        %disp(supTab);
        btag = [ '_' benchName '_' ];
        fname = strcat(obj.test_outDir, 'res_', obj.test_ID, btag, '_QEKF_table.csv');
        writetable(supTab, fname);
      end 
    end % end of get_res()      


    function res_table = get_log_res(obj, log, dat) % get per benchmark log errs 
      benchtype   = dat.dataset.benchtype;
      qTru        = dat.dataset.qTru;
      tTru        = dat.dataset.tTru;
      cntr        = 0;
      q1          = dat.posp_i.q1;
      t1          = dat.posp_i.t1;
      
      for f = dat.keyFrames
        cntr = cntr + 1;
        % get current pose
        if strcmp(benchtype, 'KITTI') || strcmp(benchtype, 'ICL') || strcmp(benchtype, 'NAIST')
          q2 = qTru(:, f);
          t2 = tTru(:, f); 
        elseif strcmp(benchtype, 'TUM')  
          fname = dat.dataset.fnames{f};
          disp(fname);
          ftime = str2double( fname(1:end-4) ); % Time at the current frame       
          [q2, t2] = InterpPoseVer1_1(ftime, dat.dataset.times, qTru, tTru); % Interpolate data to find the pose of the current camera frame  
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
          %Z_hist % z_TVQw 
          %U_hist % u_Wrpy
          X = log.X_hist{cntr, alg}; % x_TVQw
          Y = log.Y_hist{cntr, alg}; % y_TVQw
          % state pose and vel vs GT 
          x_t = X(1:3,1);
          x_q = X(7:10,1); % Qxyzw
          x_v = X(4:6,1); 
          % state residual
          y_t = Y(1:3,1);
          y_q = Y(7:10,1); % Qxyzw
          y_v = Y(4:6,1);
          % get errs 
          log.x_t_errs(cntr, alg)     = TransError(tr, x_t);
          log.x_q_errs(cntr, alg)     = QuatError(qr,  x_q);
          log.x_v_errs(cntr, alg)     = TransError(tr, x_v);
          log.y_t_L1(cntr, alg)       = sum(abs(y_t));
          log.y_q_L1(cntr, alg)       = sum(abs(y_q));
          log.y_v_L1(cntr, alg)       = sum(abs(y_v));
          log.y_t_L2(cntr, alg)       = sum(y_t.^2);
          log.y_q_L2(cntr, alg)       = sum(y_q.^2);
          log.y_v_L2(cntr, alg)       = sum(y_v.^2);
        end % for alg = length(log.algorithms)
        q1 = q2;    % store frame pose for the next keyFrame 
        t1 = t2; 
      end % for f = kframes

      stats_tabs = cell(9,0);
      stats_tabs{1} = obj.get_stats_table(log.x_t_errs, obj.tabNames{1});
      stats_tabs{2} = obj.get_stats_table(log.x_q_errs, obj.tabNames{2});
      stats_tabs{3} = obj.get_stats_table(log.x_v_errs, obj.tabNames{3});
      stats_tabs{4} = obj.get_stats_table(log.y_t_L1,   obj.tabNames{4});
      stats_tabs{5} = obj.get_stats_table(log.y_q_L1,   obj.tabNames{5});
      stats_tabs{6} = obj.get_stats_table(log.y_v_L1,   obj.tabNames{6});
      stats_tabs{7} = obj.get_stats_table(log.y_t_L2,   obj.tabNames{7});
      stats_tabs{8} = obj.get_stats_table(log.y_q_L2,   obj.tabNames{8});
      stats_tabs{9} = obj.get_stats_table(log.y_v_L2,   obj.tabNames{9});
      res_table  = stats_tabs;
    end % function get_log_errs(log, dat) 


    function stats_tab = get_stats_table(obj, errs, tabName) 
      stats       = zeros(5, size(errs,2));
      stats(1, :) = mean(errs,1);  % Mean
      stats(2, :) = std(errs,1);  % Standard deviation
      stats(3, :) = median(errs,1); % Median
      stats(4, :) = quantile(errs, 0.25, 1); % 25% quartile
      stats(5, :) = quantile(errs, 0.75, 1); % 75% quartile
      rNames      = obj.get_RowNames(obj.metNames, tabName);
      stats_tab   = obj.get_res_table(stats, rNames);
    end

    function RowNames = get_RowNames(~, metNames, tabName)
      MNn = length(metNames);
      RowNames = cell(MNn,0);
      for MN = 1:MNn
        RowNames{MN} = strcat(tabName, " ", metNames{MN});
      end
      RowNames = string(RowNames);
      %disp(RowNames);
    end


    function res_table = get_res_table(obj, data, rNames)
      if obj.numMethods == 1
        res_table  = table(data(:,1), ...
                          'RowNames', rNames, ...
                          'VariableNames', obj.pose_algorithms); 
      elseif obj.numMethods == 2
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           'RowNames', rNames, ...
                           'VariableNames', obj.pose_algorithms); 
      elseif obj.numMethods == 3
        res_table   = table(data(:,1), ...
                            data(:,2), ...
                            data(:,3), ...
                            'RowNames', rNames, ...
                            'VariableNames', obj.pose_algorithms); 
      elseif obj.numMethods == 4
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                           data(:,4), ...
                           'RowNames', rNames, ...
                           'VariableNames', obj.pose_algorithms); 
      elseif obj.numMethods == 5
        res_table   = table(data(:,1), ...
                            data(:,2), ...
                            data(:,3), ...
                            data(:,4), ...
                            data(:,5), ...
                            'RowNames', rNames{:}, ...
                            'VariableNames', obj.pose_algorithms);    
      end
    end %  function res_table = get_res_table(obj, data)
  end % methods (Access = public) 
end

classdef qekf_handler_class < matlab.System 
  properties % public vars
    % features 
    res_tab_sav_en        = true;
    res_tab_prt_en        = false;
    
    % cfg argin    
    TID
    ttag
    toutDir
    benchmark
    pos_algs
    vel_algs
    % local vars
    pos_numMethods
    vel_numMethods
    trackers
    st_sols
    res
    del_T
    %% other private constants 
    metNames  = {'mean';
                 'std';
                 'med'; 
                 'Q1';
                 'Q3';};
    tabNames = {'GT-X T err';
                'GT-X Q err';
                'GT-X V err';
                'Z-X T L1';
                'Z-X Q L1';
                'Z-X V L1';
                'Z-X T L2';
                'Z-X Q L2';
                'Z-X V L2';};
    % rpt constants 
    mod_name      = "QEKF"
    rpt_note      = " "
  end
  methods % constructor
    
    function obj = qekf_handler_class(varargin) 
      setProperties(obj, nargin, varargin{:}) % init obj w name-value args
    end  

  end % methods % constructor
  methods (Access = public) 
    
    function load_cfg(obj, cfg)
      obj.TID               = cfg.TID;   
      obj.ttag              = cfg.ttag;
      obj.toutDir           = cfg.toutDir;       
      obj.benchmark         = cfg.benchmark;
      obj.pos_algs          = cfg.pos_algs;
      obj.pos_numMethods    = length(obj.pos_algs);
      obj.vel_algs          = cfg.vel_algs;
      obj.vel_numMethods    = length(obj.vel_algs);
      obj.trackers          = cell(obj.pos_numMethods, 0);
      obj.st_sols           = cell(7,obj.pos_numMethods); % alg,Z,U,X,Y,P,K 
      obj.res               = cell(3,0); % benchmark, res_tab
      obj.del_T             = cfg.del_T;
      for alg = 1:obj.pos_numMethods
        obj.trackers{alg} = qekf_class( alg_idx = alg );
        obj.trackers{alg}.load_cfg(cfg);
      end
    end

    function st_sols = run_filter(obj, TQVW_sols)
      for a = 1:obj.pos_numMethods
        st_sol = obj.trackers{a}.run_qekf(TQVW_sols, a);
        for s = 1:length(st_sol)
          obj.st_sols{s, a} = st_sol{s};
        end
      end 
      st_sols = obj.st_sols;
    end 

    function res = get_res(obj, cfg, dlog)
      obj.res{1}   = dlog.log.benchtype;
      obj.res{2}   = obj.get_res_tab(dlog.log, cfg.dat); % returns a table object
      if obj.res_tab_prt_en
        disp(strcat(obj.mod_name, ' module:')); disp(obj.rpt_note);
        disp(obj.res{1}); disp(obj.res{1});
      end 
      if obj.res_tab_sav_en
        fname = strcat(obj.toutDir,'res_', ...
                       ...%obj.ttag, "_", ...
                       obj.mod_name,'_tab.csv');
        writetable(obj.res{2}, fname);
      end 
      res = obj.res;
    end % get_res()      

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
        for a = 1:length(log.pos_algs) % calc and save errs per method 
          Xcols   = get_cols(a, log.d_X); % --->> get var cols
          Ycols   = get_cols(a, log.d_Y);
          Z_hist % z_TVQw 
          %U_hist % u_Wrpy
          X = log.X_hist(cntr, Xcols)'; % x_TVQw
          Y = log.Y_hist(cntr, Ycols)'; % y_TVQw
          % state pose and vel vs GT 
          x_t = X(1:3,1);
          x_v = X(4:6,1);  
          x_q = X(7:10,1); % Qxyzw
          % state residual
          y_t = Y(1:3,1);
          y_v = Y(4:6,1);
          y_q = Y(7:10,1); % Qxyzw
          % get errs 
          log.x_t_errs(cntr, a)     = TransError(tr, x_t);
          log.x_v_errs(cntr, a)     = TransError(tr, x_v);
          log.x_q_errs(cntr, a)     = QuatError(qr,  x_q);
          log.y_t_L1(cntr, a)       = sum(abs(y_t));
          log.y_v_L1(cntr, a)       = sum(abs(y_v));
          log.y_q_L1(cntr, a)       = sum(abs(y_q));
          log.y_t_L2(cntr, a)       = sum(y_t.^2);
          log.y_v_L2(cntr, a)       = sum(y_v.^2);
          log.y_q_L2(cntr, a)       = sum(y_q.^2);
        end % for alg = length(log.algorithms)
        q1 = q2;    % store frame pose for the next keyFrame 
        t1 = t2; 
      end % for f = kframes
      stats_tabs    = cell(9,0);
      stats_tabs{1} = obj.get_stats_table(log.x_t_errs, obj.tabNames{1});
      stats_tabs{2} = obj.get_stats_table(log.x_q_errs, obj.tabNames{2});
      stats_tabs{3} = obj.get_stats_table(log.x_v_errs, obj.tabNames{3});
      stats_tabs{4} = obj.get_stats_table(log.y_t_L1,   obj.tabNames{4});
      stats_tabs{5} = obj.get_stats_table(log.y_q_L1,   obj.tabNames{5});
      stats_tabs{6} = obj.get_stats_table(log.y_v_L1,   obj.tabNames{6});
      stats_tabs{7} = obj.get_stats_table(log.y_t_L2,   obj.tabNames{7});
      stats_tabs{8} = obj.get_stats_table(log.y_q_L2,   obj.tabNames{8});
      stats_tabs{9} = obj.get_stats_table(log.y_v_L2,   obj.tabNames{9});
      res_table     = vertcat(stats_tabs{:});
    end % function get_res_tab(log, dat) 

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
      if obj.pos_numMethods == 1
        res_table  = table(data(:,1), ...
                          'RowNames', rNames, ...
                          'VariableNames', obj.pos_algs); 
      elseif obj.pos_numMethods == 2
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           'RowNames', rNames, ...
                           'VariableNames', obj.pos_algs); 
      elseif obj.pos_numMethods == 3
        res_table   = table(data(:,1), ...
                            data(:,2), ...
                            data(:,3), ...
                            'RowNames', rNames, ...
                            'VariableNames', obj.pos_algs); 
      elseif obj.pos_numMethods == 4
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                           data(:,4), ...
                           'RowNames', rNames, ...
                           'VariableNames', obj.pos_algs); 
      elseif obj.pos_numMethods == 5
        res_table   = table(data(:,1), ...
                            data(:,2), ...
                            data(:,3), ...
                            data(:,4), ...
                            data(:,5), ...
                            'RowNames', rNames, ...
                            'VariableNames', obj.pos_algs);    
      end
    end %  function res_table = get_res_table(obj, data)
  end % methods (Access = public) 
end

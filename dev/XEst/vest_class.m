classdef vest_class < matlab.System 
  properties % public vars
    %% features 
    res_tab_sav_en        = true;
    res_tab_prt_en        = false;
    prt_exp_map_w_Q_en   =  false;
    %% cfg argin
    TID
    ttag
    toutDir
    benchmark
    %benchmarks
    pos_algs
    vel_algs  = {'VEst'} 
    del_T % time period
    %% private vars
    res
    vel_numMethods
    pos_numMethods 
    %numBenchmarks
    m  % current frame feature points 
    m_dot % m(i) - m(i-1) of matched feature points  
    %% private constants 
    exp_map_threshold    =  0.04;
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
    mod_name        = 'VEst+'
    rpt_note    = "Since VEst outputs V and W, we compute the integral " + ...
                  "of the two and compute the error with respect to the " + ...
                  "ground truth for each frame.";
  end
  methods % constructor
    
    function obj = vest_class(varargin) 
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end  
  
  end 
  methods (Access = public) 
  
    function load_cfg(obj, cfg)
      obj.TID            = cfg.TID;                   
      obj.ttag           = cfg.ttag;                   
      obj.toutDir        = cfg.toutDir;
      obj.benchmark      = cfg.benchmark;
      %obj.benchmarks     = cfg.benchmarks;
      obj.pos_algs       = cfg.pos_algs;
      obj.vel_algs       = cfg.vel_algs;
      obj.del_T          = cfg.del_T;
      obj.init();
    end

    function TQVW_sols = get_vel(obj, matches, TQVW_sols) % --->> run
      [obj.m, obj.m_dot] = obj.prep_matches(matches);
      [v, w]  = PoCo(obj.m, obj.m_dot); % --->> run VEst
      %% log Vest T and Q as a pose est method
      TQVW_sols{2, end}  = normalizeVec(v .* obj.del_T); % VEst_T
      TQVW_sols{3, end}  = check_quats(exp_map(w)); % VEst_Q
      for alg = 1:length(obj.pos_algs) % log VEst V W for all pose algs
        assert(strcmp(obj.pos_algs{alg}, TQVW_sols{1, alg}{1}), ... 
          "[log_class.log_state()]--> alg mismatch"); 
        TQVW_sols{4, alg} = normalizeVec(v); % VEst_V
        TQVW_sols{5, alg} = normalizeVec(w); % VEst_W
      end 
    end 

    function res = get_res(obj, cfg, dlog)
      obj.res{1}   = dlog.log.benchtype;
      obj.res{2}   = obj.get_res_tab(dlog.log, cfg.dat); % returns a table object
      if obj.res_tab_prt_en
        disp(strcat(obj.mod_name, ' module:')); disp(obj.rpt_note);
        disp(obj.res{1}); disp(obj.res{2});
      end 
      if obj.res_tab_sav_en
        fname = strcat(obj.toutDir, 'res_', ...
                       ...%obj.ttag, "_", ...
                       obj.mod_name, '_tab.csv');
        writetable(obj.res{2}, fname);
      end 
      res = obj.res;
    end % get_res()      

    function  [v_err, w_err] = get_err(obj, idx, v, w)
      v = obj.normalize(v);
      w = obj.normalize(w);
      v_err = 1/pi * acos(v' * obj.v_gt(idx,:));
      w_err = sqrt(   (w - obj.w_gt(idx))^2   );
    end 
  
    function dist = cmp_exp_map_w_Q(obj, W, Q)
      Q_VEst = exp_map(W);   
      dist = obj.phi03_dist(Q, Q_VEst); 
      if dist > obj.exp_map_threshold  && obj.prt_exp_map_w_Q_en % 
        disp('exp map of W NOT AN EXACT match!');
        msg = sprintf('Phi03 dist: %f ', dist);
        disp(msg);
      end
    end

  end % end of public access 
  methods (Access = private)
    
    function init(obj)
      %obj.numBenchmarks         = length(obj.benchmarks);
      obj.pos_numMethods        = length(obj.pos_algs);
      obj.vel_numMethods        = length(obj.vel_algs);
      obj.res                   = cell(3,0); % benchmark, res_tab, res_fig
    end 

    function stats = get_stats(~, errs) 
      stats      = zeros(5, size(errs,2));
      stats(1,:) = mean(errs,1);  % Mean
      stats(2,:) = std(errs,1);  % Standard deviation
      stats(3,:) = median(errs,1); % Median
      stats(4,:) = quantile(errs, 0.25, 1); % 25% quartile
      stats(5,:) = quantile(errs, 0.75, 1); % 75% quartile
    end
    
    function res_table = get_res_tab(obj, log, dat) % get per benchmark log errs 
      btype   = dat.dataset.benchtype;
      qTru    = dat.dataset.qTru;
      tTru    = dat.dataset.tTru;
      cntr    = 0;
      q1      = dat.posp_i.q1;
      t1      = dat.posp_i.t1;
      for f = dat.kframes
        cntr          = cntr + 1;
        [tr,qr,t2,q2] = get_relGT(f, btype, tTru, qTru, t1, q1);
        w             = log.W_hist(cntr,1:log.d_W)';
        v             = log.V_hist(cntr,1:log.d_V)';
        assert(isequal(size(w),[3,1]),"[vest.get_log_res]--->> w is not [3,1]!");
        log.VEst_Q_errs(cntr, 1)   = obj.cmp_exp_map_w_Q(w, qr);          
        log.VEst_T_errs(cntr, 1)   = TransError(tr, v);
        q1 = q2; % store frame pose for the next keyFrame 
        t1 = t2; 
      end 
      VEst_T_stats     = obj.get_stats(log.VEst_T_errs);
      VEst_Q_stats     = obj.get_stats(log.VEst_Q_errs);
      VEst_T_res_tab   = obj.get_res_table(VEst_T_stats, obj.T_RowNames);
      VEst_Q_res_tab   = obj.get_res_table(VEst_Q_stats, obj.Q_RowNames);
      res_table        = vertcat(VEst_T_res_tab, VEst_Q_res_tab);
    end % function get_res_tab(log, dat) 

    function dist = phi03_dist(~, qr, q)
      dist = (1/pi) * acos(abs( sum(qr .* q)));
    end 

    function [m, m_dot] = prep_matches(~, matches)
      m       = matches.m2; 
      m_dot   = matches.m2 - matches.m1;
    end 

    function normed = normalize(~, vector)
      normed  = vector/ norm(vector); % normalize v answer 
      if(normed(3)<0) 
        normed = - normed; 
      end 
    end
  
    function res_table = get_res_table(obj, data, RowNames)
      if obj.vel_numMethods == 1
        res_table  = table(data(:,1), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.vel_algs); 
      elseif obj.vel_numMethods == 2
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.vel_algs); 
      elseif obj.vel_numMethods == 3
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.vel_algs); 
      elseif obj.vel_numMethods == 4
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                           data(:,4), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.vel_algs); 
      elseif obj.vel_numMethods == 5
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                           data(:,4), ...
                           data(:,5), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.vel_algs);       
      end
    end %  function res_table = get_res_table(obj, data)

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
    
  end % methods (Access = private)
end

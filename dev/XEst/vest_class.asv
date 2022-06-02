classdef vest_class < matlab.System 
  properties % public vars
    % features 
    prt_exp_map_w_Q_en   =  false;
    exp_map_threshold    =  0.04;
    
    % cfg argin
    test_ID
    test_outDir
    benchmark
    %benchmarks
    algorithms
    del_T % time period
    %% private 
    numMethods
    %numBenchmarks
    % run-time variables 
    m  % current frame feature points 
    m_dot % m(i) - m(i-1) of matched feature points  
    
    %% private constants 
    W_RowNames  = {'exp(W) err mean';
                   'exp(W) err std';
                   'exp(W) err med'; 
                   'exp(W) err Q1';
                   'exp(W) err Q3';
                  };
    Q_RowNames  = {'Q err mean';
                   'Q err std';
                   'Q err med'; 
                   'Q err Q1';
                   'Q err Q3';
                  };

   
  end
  methods % constructor
    function obj = vest_class(varargin) 
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end  
  end 
  methods (Access = public) 
    function load_cfg(obj, cfg)
      obj.test_ID        = cfg.test_ID;                   
      obj.test_outDir    = cfg.test_outDir;
      obj.benchmark      = cfg.benchmark;
      %obj.benchmarks     = cfg.benchmarks;
      obj.algorithms     = cfg.pose_algorithms;
      obj.del_T          = cfg.del_T;
      obj.init();
    end

    function TQVW_sols = get_vel(obj, matches, TQVW_sols)
      [obj.m, obj.m_dot] = obj.prep_matches(matches);
      [v, w]  = PoCo(obj.m, obj.m_dot); % call algorithm 
      for alg = 1:obj.numMethods
        assert(strcmp(TQVW_sols{1,alg}, obj.algorithms{alg}), ...
          '[vest.get_vel()]--> pose_alg mismatch!!')
        if strcmp(obj.algorithms{alg}, 'VEst_Q') % --->> VEst_Q
          TQVW_sols{2, alg}  = v .* obj.del_T; % T
          %disp(TQVW_sols{2, alg});
          TQVW_sols{3, alg}  = exp_map(w); % Q
        end
        TQVW_sols{4, alg} = v; % V
        TQVW_sols{5, alg} = w; % W
      end 
      %disp(TQVW_sols);
    end 

    function res = get_res(obj, cfg, dlog)
      res = cell( 1, 2);
      dat = cfg.dat;
      log = dlog.log;
      res{1, 1}   = dlog.log.benchtype;
      res{1, 2}   = obj.get_log_res(log, dat);  % returns a table object
      if dlog.res_prt_en
        disp('VEst module:');
        msg = "Here, we compare VEst_Q (exp_map(W)) of each frame with " + ...
          "the Q estimate of each method for the same frame. ";
        disp(msg);
        disp(res{1, 1});
        disp(res{1, 2});
      end 
      if dlog.res_sav_en
        btag = ['_' res{1, 1} '_'];
        fname = strcat(obj.test_outDir, 'res_', obj.test_ID, btag, '_VEst_table.csv');
        writetable(res{1, 2}, fname);
      end 
    end % function res = get_res(obj, cfg, dlog)

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
      %obj.numBenchmarks    = length(obj.benchmarks);
      obj.numMethods           = length(obj.algorithms);
    end 

    function stats = get_stats(~, errs) 
      stats     =  zeros(5, size(errs,2));
      stats(1, :) = mean(errs,1);  % Mean
      stats(2, :) = std(errs,1);  % Standard deviation
      stats(3, :) = median(errs,1); % Median
      stats(4, :) = quantile(errs, 0.25, 1); % 25% quartile
      stats(5, :) = quantile(errs, 0.75, 1); % 75% quartile
    end
    
    function res_table = get_log_res(obj, log, dat) % get per benchmark log errs 
      btype   = dat.dataset.benchtype;
      qTru    = dat.dataset.qTru;
      tTru    = dat.dataset.tTru;
      cntr    = 0;
      q1      = dat.posp_i.q1;
      t1      = dat.posp_i.t1;
      for f = dat.keyFrames
        cntr = cntr + 1;
        for alg = 1:length(log.algorithms) % calc and save errs per method
          [tr,qr,t2,q2] = get_relGT(f, btype, tTru, qTru, t1, q1);
          w    = log.W_hist{cntr, alg};
          v    = log.V_hist{cntr, alg};

          
          log.V_errs(cntr, alg)   = TransError(tr, t);
          if isequal(size(w), [3, 1]) 
            
            log.W_errs(cntr, alg)     = obj.cmp_exp_map_w_Q(w, qr);
            log.Q_errs(cntr, alg)   = TransError(tr, t);
            end
          else % other pose_algs
            log.W_errs(cntr, alg)     = obj.cmp_exp_map_w_Q(w, qr);
            % add v_errs
          end
        end % for alg = length(log.algorithms)
        q1 = q2; % store frame pose for the next keyFrame 
        t1 = t2; 
      end % for f = kframes
      T_stats = obj.get_stats(log.T_errs);
      Q_stats = obj.get_stats(log.Q_errs);
      data    = [T_stats; Q_stats];
      disp(data);
      Q_stats       = obj.get_stats(log.Q_errs);
      %W_stats       = obj.get_stats(log.W_errs);
      Q_res_table   = obj.get_res_table(Q_stats, obj.Q_RowNames);
      %W_res_table   = obj.get_res_table(W_stats, obj.W_RowNames);
      %res_table     = vertcat(Q_res_table, W_res_table);
      res_table      = Q_res_table;
    end % function get_log_errs(log, dat) 

    function dist = phi03_dist(~, qr, q)
      dist = (1/pi) * acos(abs( sum(qr .* q)));
    end 

    function [m, m_dot] = prep_matches(~, matches)
      m         = matches.m2; 
      m_dot = matches.m2 - matches.m1;
    end % end of prep_matches

    function normed = normalize(~, vector)
      normed  = vector/ norm(vector); % normalize v answer 
        if(normed(3)<0) 
            normed = - normed; 
        end 
    end
    


    function res_table = get_res_table(obj, data, RowNames)
      if obj.numMethods == 1
        res_table  = table(data(:,1), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 2
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 3
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 4
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                           data(:,4), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 5
        res_table  = table(data(:,1), ...
                           data(:,2), ...
                           data(:,3), ...
                           data(:,4), ...
                           data(:,5), ...
                           'RowNames', RowNames, ...
                           'VariableNames', obj.algorithms);       
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

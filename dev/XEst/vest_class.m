classdef vest_class < matlab.System 
  properties % public vars
    % features 
    prt_exp_map_w_Q_en   =  false;
    exp_map_threshold        =  0.04;
    
    % cfg argin
    test_ID
    test_outDir
    benchmark
    %benchmarks
    algorithms
    
    %% private 
    numMethods
    %numBenchmarks
    % run-time variables 
    m  % current frame feature points 
    m_dot % m(i) - m(i-1) of matched feature points  
    
    %% private constants 
    RowNames  = {'VEst - Rot err mean';
                 'VEst - Rot err std';
                 'VEst - Rot err median'; 
                 'VEst - Rot err Q_1';
                 'VEst - Rot err Q_3';
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
      obj.init();
    end
    function TQVW_sols = get_vel(obj, matches, TQVW_sols)
      [obj.m, obj.m_dot] = obj.prep_matches(matches);
      [v, w]  = PoCo(obj.m, obj.m_dot); % call algorithm 
      for alg = 1:obj.numMethods
        assert(strcmp(TQVW_sols{1,alg}, obj.algorithms{alg}), ...
          '[vest.get_vel()]--> pose_alg mismatch!!')
        TQVW_sols{4, alg} = v;
        TQVW_sols{5, alg} = w;
      end
    end 

    function  [v_err, w_err] = get_err(obj, idx, v, w)
      v = obj.normalize(v);
      w = obj.normalize(w);
      v_err = 1/pi * acos(v' * obj.v_gt(idx,:));
      w_err = sqrt(   (w - obj.w_gt(idx))^2   );
    end 
  
    function dist = cmp_exp_map_w_Q(obj, W, Q)
      Q_VEst = obj.exp_map(W);   
      dist = obj.phi03_dist(Q, Q_VEst); 
      if dist > obj.exp_map_threshold  && obj.prt_exp_map_w_Q_en % 
        disp('exp map of W NOT AN EXACT match!');
        msg = sprintf('Phi03 dist: %f ', dist);
        disp(msg);
      end
    end

    function res = get_res(obj, cfg, dlog)
      res = cell( 1, 2);
      dat = cfg.dat;
      log = dlog.log;
      res{1, 1}   = dlog.log.benchtype;
      res{1, 2}   = obj.get_log_res(log, dat);  % returns a table object

      if dlog.res_prt_en
        disp(res{1, 1}); disp(res{1, 2});
      end 
      
      if dlog.res_sav_en
        btag = [ '_' res{1, 1} '_' ];
        fname = strcat(obj.test_outDir, 'res_', obj.test_ID, btag, '_VEst_table.csv');
        writetable(res{1, 2}, fname);
      end 

    end % function res = get_res(obj, cfg, dlog)
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
      cntr = 0;
      for f = dat.keyFrames
        cntr = cntr + 1;
        for alg = 1:length(log.algorithms) % calc and save errs per method
          q    = log.Q_hist{cntr, alg};
          w    = log.W_hist{cntr, alg};
          if isequal(size(w), [3,1]) 
            log.W_errs(cntr, alg)     = obj.cmp_exp_map_w_Q(w, q);
          end
        end % for alg = length(log.algorithms)
      end % for f = kframes
      W_stats = obj.get_stats(log.W_errs);
      res_table  = obj.get_res_table(W_stats);
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
    
    function Qwxyz = exp_map(~, x)
      %disp(class(x));
      %disp(x);
      if isequal(size(x), [3,1]) 
        norm_x = norm(x);
        if norm_x == 0
          Qwxyz = [1; 0; 0; 0];
        else
          qxyz = sin(norm_x/2) .* x/norm_x;
          qw = cos(norm_x/2);
          Qwxyz = [qw; qxyz(1); qxyz(2); qxyz(3)];
        end
      else
        disp(x);
        assert(false, "[VEst.exp_map]--> x size is not [3 1]! ");
      end
    end % Qxyzw = exp_map(x)

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

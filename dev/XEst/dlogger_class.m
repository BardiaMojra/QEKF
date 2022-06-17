classdef dlogger_class < matlab.System 
  properties
    %% features
    %log_prt_en        = false;
    %log_sav_en        = true;
    %log_csv_sav_en    = true;
    % config (argin)
    TID
    ttag
    toutDir
    benchmark
    pos_algs
    vel_algs
    pos_numMethods 
    vel_numMethods 
    %dat
    %
    log
    pfeat_logs % pfeat mod
    pos_logs
    vel_logs
    qekf_logs
  end
  methods  % constructor

    function obj = dlogger_class(varargin)
      setProperties(obj,nargin,varargin{:})
    end
  
  end 
  methods (Access = public) 

    function load_cfg(obj, cfg)
      obj.TID               = cfg.TID;
      obj.ttag              = cfg.ttag;
      obj.toutDir           = cfg.toutDir;
      obj.benchmark         = cfg.benchmark;  
      obj.pos_numMethods    = cfg.pos_numMethods;
      obj.pos_algs          = cfg.pos_algs;  
      obj.vel_numMethods    = cfg.vel_numMethods;
      obj.vel_algs          = cfg.vel_algs;
      obj.log               = log_class(btype     =  cfg.dat.benchtype, ... 
                                        kframes   =  cfg.dat.kframes, ...
                                        pos_algs  =  cfg.pos_algs, ...
                                        vel_algs  =  cfg.vel_algs );
      obj.log.load_cfg(cfg); 
      obj.log.dat               = cfg.dat;
    end
    
    function log_state(obj, cntr, frame_idx, TQVW_sols, state_sols)
      obj.log.log_state(cntr, frame_idx, TQVW_sols, state_sols);
    end

    function get_logs(obj)
      %obj.pfeat_logs = obj.log.get_qekf_logs();
      obj.pos_logs = obj.log.get_pos_logs();
      obj.vel_logs = obj.log.get_vel_logs();
      obj.qekf_logs = obj.log.get_qekf_logs();
    end 
    
    function log_plts = get_log_plts(obj)
      log_plts = cell(3,2);
      log_plts{1,1} = "pos";
      log_plts{1,2} = obj.get_pos_plts();
      log_plts{2,1} = "vel";
      log_plts{2,2} = obj.get_vel_plts();
      log_plts{3,1} = "qekf";
      log_plts{3,2} = obj.get_qekf_plts();
    end 

  end % methods (Access = public) 
  methods (Access = private) 


    function get_pos_plts(obj)
      
      for a = 1:obj.pos_numAlgs
        assert(isequal(obj.pos_logs{1,a}, obj.pos_algs{a}), ...
          "pos alg not matching pos log name!")
        obj.plot_pos_log(obj.pos_logs{2,a}, a);
      end 
    end 

    function plot_pos_log(obj, log, a)
            
      subplot(7,1,1); hold on; subtitle('$T_{x}$',"Interpreter",'latex', ... % Tx
        'fontsize',obj.fig_txt_size); grid on;
      plot(idx, log(:,3), "Color",obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
      subplot(7,1,2); hold on; subtitle('$T_{y}$',"Interpreter",'latex', ... % Ty
        'fontsize',obj.fig_txt_size); grid on;
      plot(idx, log(:,4), "Color",obj.plt_lclrs(a), "Marker", obj.plt_mrkrs(a));
      subplot(7,1,3); hold on; subtitle('$T_{z}$',"Interpreter",'latex', ... % Tz
        'fontsize',obj.fig_txt_size); grid on;
      plot(idx, log(:,5), "Color",obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));        
      subplot(7,1,4); hold on; subtitle('$Q_{w}$',"Interpreter",'latex', ... % Qw
        'fontsize',obj.fig_txt_size); grid on;
      plot(idx, log(:,6), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
      subplot(7,1,5); hold on; subtitle('$Q_{x}$',"Interpreter",'latex', ... % Qx
        'fontsize',obj.fig_txt_size); grid on;
      plot(idx, log(:,7), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
      subplot(7,1,6); hold on; subtitle('$Q_{y}$',"Interpreter",'latex', ... % Qy
        'fontsize',obj.fig_txt_size); grid on; 
      plot(idx, log(:,8), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));  
      subplot(7,1,7); hold on; subtitle('$Q_{z}$',"Interpreter",'latex', ... % Qz
        'fontsize',obj.fig_txt_size); grid on;
      plot(idx, log(:,9), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
      hold off
      lg          = legend([obj.pos_algs; "Groundtruth"]); 
      lg.Units    = obj.leg_units;
      lg.Position = obj.leg_pos;
      lg.FontSize = obj.fig_txt_size-4;
      if obj.plt_questp_sav_en
        figname = strcat(obj.toutDir,"plt_QuEst+_logs.png");
        saveas(fig, figname);
      end
      if obj.plt_questp_shw_en
        waitforbuttonpress;
      end
      close(fig);
    end % function figname = plt_quest_logs(obj, log, rgt_T, rgt_Q)
    end 

    function plot_something()
      idx       = log.cntr_hist;
      T         = log.T_hist;
      Q         = log.Q_hist;
      numKF     = log.numKF;
      numAlgs   = log.pos_numAlgs;
      Tx    = zeros(numKF, numAlgs + 1); % Xs+GT
      Ty    = zeros(numKF, numAlgs + 1);
      Tz    = zeros(numKF, numAlgs + 1);
      Qw    = zeros(numKF, numAlgs + 1);
      Qx    = zeros(numKF, numAlgs + 1);
      Qy    = zeros(numKF, numAlgs + 1);
      Qz    = zeros(numKF, numAlgs + 1);
      for a = 1:obj.pos_numAlgs
        Tcols   = get_cols(a, log.d_T); % --->> get var cols
        Qcols   = get_cols(a, log.d_Q);
        Tx(:,a) = T(:, Tcols(1)); % --->> load to plt cols
        Ty(:,a) = T(:, Tcols(2));
        Tz(:,a) = T(:, Tcols(3));    
        Qw(:,a) = Q(:, Qcols(1));
        Qx(:,a) = Q(:, Qcols(2));
        Qy(:,a) = Q(:, Qcols(3));
        Qz(:,a) = Q(:, Qcols(4));
      end
      Tx(:, end) = rgt_T(:,1); % --->> load ground truth to last col
      Ty(:, end) = rgt_T(:,2);
      Tz(:, end) = rgt_T(:,3);    
      Qw(:, end) = rgt_Q(:,1);
      Qx(:, end) = rgt_Q(:,2);
      Qy(:, end) = rgt_Q(:,3);
      Qz(:, end) = rgt_Q(:,4);
      fig = figure(); % 7 subplots Txyz Qwxyz
      sgtitle("QuEst+ Pose Estimate Logs","Interpreter",'latex');
      fig.Units    = obj.fig_units;
      fig.Position = obj.fig_pos;
      hold on
      for a = 1:obj.pos_numAlgs+1 % +gt

        subplot(7,1,1); hold on; subtitle('$T_{x}$',"Interpreter",'latex', ... % Tx
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Tx(:,a), "Color",obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
        subplot(7,1,2); hold on; subtitle('$T_{y}$',"Interpreter",'latex', ... % Ty
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Ty(:,a), "Color",obj.plt_lclrs(a), "Marker", obj.plt_mrkrs(a));
        subplot(7,1,3); hold on; subtitle('$T_{z}$',"Interpreter",'latex', ... % Tz
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Tz(:,a), "Color",obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));        
        subplot(7,1,4); hold on; subtitle('$Q_{w}$',"Interpreter",'latex', ... % Qw
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Qw(:,a), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
        subplot(7,1,5); hold on; subtitle('$Q_{x}$',"Interpreter",'latex', ... % Qx
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Qx(:,a), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
        subplot(7,1,6); hold on; subtitle('$Q_{y}$',"Interpreter",'latex', ... % Qy
          'fontsize',obj.fig_txt_size); grid on; 
        plot(idx, Qy(:,a), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));  
        subplot(7,1,7); hold on; subtitle('$Q_{z}$',"Interpreter",'latex', ... % Qz
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Qz(:,a), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
      end
      hold off
      lg          = legend([obj.pos_algs; "Groundtruth"]); 
      lg.Units    = obj.leg_units;
      lg.Position = obj.leg_pos;
      lg.FontSize = obj.fig_txt_size-4;
      if obj.plt_questp_sav_en
        figname = strcat(obj.toutDir,"plt_QuEst+_logs.png");
        saveas(fig, figname);
      end
      if obj.plt_questp_shw_en
        waitforbuttonpress;
      end
      close(fig);
    end % function figname = plt_quest_logs(obj, log, rgt_T, rgt_Q)
  end 
end

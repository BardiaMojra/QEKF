classdef report_class < matlab.System 
  
  properties % public vars
    % features 
    rpt_shw_en          = true
    plt_questp_shw_en   = false
    plt_questp_sav_en   = true
    plt_vestp_shw_en    = true
    plt_vestp_sav_en    = true
    plt_qekf_shw_en     = true
    plt_qekf_sav_en     = true
    %% config
    rpt_title         = 'XEst Report'
    rpt_subtitle      = 'Real-time vision-based navigation module'
    rpt_author        = 'Bardia Mojra'
    text_font_size    = '12pt'
    num_font_size     = '10pt'
    font_style        = 'Times'
    num_format        = "%1.4f"
    rpt_outFormat     = 'pdf'
    fig_txt_size      = 12;
    fig_units         = "inches"
    fig_pos           = [0 0 7 10]
    leg_units         = "inches"
    leg_pos           = [6 9 .8 .8]
    % cfg (argin)
    test_ID
    toutDir
    benchmark
    pos_algs
    vel_algs
    % private vars
    pos_numAlgs
    vel_numAlgs
    rpt
    %numBenchmarks
    % private constants 
    plt_lclrs = ["#A2142F", "#77AC30", "#0072BD", "#7E2F8E", ...
                 "#EDB120", "#4DBEEE", "#D95319", "#77AC30"] % unique per alg
    plt_mrkrs = ["o", "+", "*", ".", ...
                 "x", "s", "d", "^", ...
                 "v", ">", "<", "h"]
  
  end
  methods % constructor
    
    function obj = report_class(varargin) % init obj w name-value args
      setProperties(obj,nargin,varargin{:}) 
    end  
  
  end 
  methods (Access = public) 
  
    function load_cfg(obj, cfg)
      obj.test_ID        = cfg.test_ID;                   
      obj.toutDir        = cfg.test_outDir;
      obj.benchmark      = cfg.benchmark;
      obj.pos_algs       = cfg.pos_algs;
      obj.vel_algs       = cfg.vel_algs;
      obj.init();
    end

    function gen_report(obj, quest, vest, qekf)
      import mlreportgen.report.* 
      import mlreportgen.dom.* 
      setDefaultNumberFormat(obj.num_format);

      mlreportgen.dom.FontSize(obj.text_font_size);
      mlreportgen.dom.FontFamily(obj.font_style);

      obj.rpt     = Report(strcat(obj.toutDir,'rpt_',obj.test_ID,'_report'), ...
                           obj.rpt_outFormat);
      %open(obj.rpt);
      append(obj.rpt, TitlePage('Title', obj.rpt_title, ...
                                'Author', obj.rpt_author ));
      append(obj.rpt, TableOfContents);
      

      sec = obj.get_sec(quest);
      append(obj.rpt, sec);
      sec = obj.get_sec(vest);
      append(obj.rpt, sec);
      sec = obj.get_sec(qekf);
      append(obj.rpt, sec);
      close(obj.rpt)

      if obj.rpt_shw_en 
        rptview(obj.rpt); 
      end
    end

    function gen_plots(obj, dat, dlog, quest, vest, qekf)
      log               = dlog.log;
      %idx               = log.cntr_hist;
      %kf                = log.kf_hist;
      [rgt_T, rgt_Q]    = dat.get_kframe_rgtruths();
      
      T     = log.T_hist;
      Q     = log.Q_hist;
      V     = log.V_hist;
      W     = log.W_hist;
 
      quest.res{3} = obj.plt_log_QuEstp(log, rgt_T, rgt_Q);
      %vest_figname  = obj.plt_log_VEstp(log, rgt_T, rgt_Q);
      %qekf.res{3} = obj.plt_log_QEKF(log, rgt_T, rgt_Q);
      vest.res{3} = nan;
      qekf.res{3} = nan;
      
      
      %V_hist     = log.V_hist;
      %W_hist     = log.W_hist;
      %Z_hist     = log.Z_hist;
      %U_hist     = log.U_hist;
      %X_hist     = log.X_hist;
      %Y_hist     = log.Y_hist;

      disp("gen plots!");
    end 

  end % end of public access 
  methods (Access = private)
    
    function init(obj)
      obj.pos_numAlgs    = length(obj.pos_algs);
      obj.vel_numAlgs    = length(obj.vel_algs);
    end

    function sec = get_sec(obj, mod)
      import mlreportgen.report.* 
      import mlreportgen.dom.* 
      sec = Chapter(); % heading 
      append(sec, Chapter(Title = mod.mod_name, ...
                          Content = mod.rpt_note));%, ...
                          %FontSize = obj.text_font_size, ...
                          %FontFamilyName = obj.font_style)
      %if isstring(mod.res{3})
      %  % shrink
      %  append(sec, Chapter(Title = mod.res{1}, ...
      %                    Content = Image(mod.res{3})));
      %                    %FontSize = obj.text_font_size, ...
      %                    %FontFamilyName = obj.font_style);
      %end
      
      tab = Table(mod.res{2}); % res_tab
      tab.Style = [tab.Style 
                  {NumberFormat(obj.num_format),...
                   Width("95%"),...
                   Border("solid"),...
                   ColSep("solid"),...
                   RowSep("solid"),...
                   mlreportgen.dom.FontSize(obj.num_font_size),...
                   mlreportgen.dom.FontFamily(obj.font_style)
                   }];
      headerStyle = { BackgroundColor("LightBlue"), ...
                Bold(true) };
      firstRow = tab.Children(1);
      firstRow.Style = [{HAlign("center")},headerStyle];
      tab.TableEntriesHAlign = "center";
      tab.TableEntriesVAlign = "middle";      
      append(sec, tab);
    end 

    function figname = plt_log_QuEstp(obj, log, rgt_T, rgt_Q)
      idx   = log.cntr_hist;
      T     = log.T_hist;
      Q     = log.Q_hist;
      Tx    = nan(log.numKF, log.pos_numAlgs);
      Ty    = nan(log.numKF, log.pos_numAlgs);
      Tz    = nan(log.numKF, log.pos_numAlgs);
      Qw    = nan(log.numKF, log.pos_numAlgs);
      Qx    = nan(log.numKF, log.pos_numAlgs);
      Qy    = nan(log.numKF, log.pos_numAlgs);
      Qz    = nan(log.numKF, log.pos_numAlgs);
      for a = 1:obj.pos_numAlgs
        Tcols = get_cols(a, log.d_T); % --->> get var cols
        Qcols = get_cols(a, log.d_Q);
        Tx(:,a) = T(:, Tcols(1)); % --->> load to plt cols
        Ty(:,a) = T(:, Tcols(2));
        Tz(:,a) = T(:, Tcols(3));    
        Qw(:,a) = Q(:, Qcols(1));
        Qx(:,a) = Q(:, Qcols(1));
        Qy(:,a) = Q(:, Qcols(2));
        Qz(:,a) = Q(:, Qcols(3));
      end
      Tx = horzcat(Tx, rgt_T(:,1)); % --->> concat ground truth
      Ty = horzcat(Ty, rgt_T(:,2));
      Tz = horzcat(Tz, rgt_T(:,3));
      Qw = horzcat(Qw, rgt_Q(:,1));
      Qx = horzcat(Qx, rgt_Q(:,1));
      Qy = horzcat(Qy, rgt_Q(:,2));
      Qz = horzcat(Qz, rgt_Q(:,3));
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
      lg  = legend([obj.pos_algs; "Groundtruth"]); 
      lg.Units    = obj.leg_units;
      lg.Position = obj.leg_pos;
      lg.FontSize = obj.fig_txt_size-4;
      if obj.plt_questp_sav_en
        figname = strcat(obj.toutDir,"fig_pose_logs.png");
        saveas(fig, figname);
      end
      if obj.plt_questp_shw_en
        waitforbuttonpress;
      end
      close(fig);
    end 

    function fname = plt_log_qekf(obj, log)
      V     = log.V_hist;
      W     = log.W_hist;

      idx   = log.cntr_hist;
      T     = log.T_hist;
      Q     = log.Q_hist;

      Tx    = nan(log.numKF, log.pos_numAlgs);
      Ty    = nan(log.numKF, log.pos_numAlgs);
      Tz    = nan(log.numKF, log.pos_numAlgs);
      Qw    = nan(log.numKF, log.pos_numAlgs);
      Qx    = nan(log.numKF, log.pos_numAlgs);
      Qy    = nan(log.numKF, log.pos_numAlgs);
      Qz    = nan(log.numKF, log.pos_numAlgs);
      for a = 1:obj.pos_numAlgs
        Tcols = get_cols(a, log.d_T); % --->> get var cols
        Qcols = get_cols(a, log.d_Q);
        Tx(:,a) = T(:, Tcols(1)); % --->> load to plt cols
        Ty(:,a) = T(:, Tcols(2));
        Tz(:,a) = T(:, Tcols(3));    
        Qw(:,a) = Q(:, Qcols(1));
        Qx(:,a) = Q(:, Qcols(1));
        Qy(:,a) = Q(:, Qcols(2));
        Qz(:,a) = Q(:, Qcols(3));
      end
      Tx = horzcat(Tx, rgt_T(:,1)); % --->> concat ground truth
      Ty = horzcat(Ty, rgt_T(:,2));
      Tz = horzcat(Tz, rgt_T(:,3));
      Qw = horzcat(Qw, rgt_Q(:,1));
      Qx = horzcat(Qx, rgt_Q(:,1));
      Qy = horzcat(Qy, rgt_Q(:,2));
      Qz = horzcat(Qz, rgt_Q(:,3));
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
      lg  = legend([obj.pos_algs; "Groundtruth"]); 
      lg.Units    = obj.leg_units;
      lg.Position = obj.leg_pos;
      lg.FontSize = obj.fig_txt_size-4;
      if obj.plt_questp_sav_en
        fname = strcat(obj.toutDir,'fig_pose_logs.png');
        saveas(fig, fname);
      end
      if obj.plt_questp_shw_en
        waitforbuttonpress;
      end
      close(fig);
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
  end % methods (Access = private)
end

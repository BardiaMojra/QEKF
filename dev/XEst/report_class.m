classdef report_class < matlab.System 
  
  properties % public vars
    % features 
    rpt_shw_en          = false
    plt_questp_shw_en   = false
    plt_questp_sav_en   = true
    plt_vestp_shw_en    = false
    plt_vestp_sav_en    = true
    plt_qekf_shw_en     = false
    plt_qekf_sav_en     = true
    %% config
    rpt_title         = 'XEst Report'
    rpt_subtitle      = 'Real-time vision-based navigation module'
    rpt_author        = 'Bardia Mojra'
    rpt_outFormat     = 'pdf'
    font_style        = 'Times'
    text_font_size    = '12pt'
    tab_font_size     = '10pt'
    num_format        = "%1.4f"
    fig_txt_size      = 12;
    fig_units         = "inches"
    fig_pos           = [0 0 7 10]
    leg_units         = "inches"
    leg_pos           = [6 9 .8 .8]
    % cfg (argin)
    TID
    ttag
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
      obj.TID            = cfg.TID;
      obj.ttag           = cfg.ttag;
      obj.toutDir        = cfg.toutDir;
      obj.benchmark      = cfg.benchmark;
      obj.pos_algs       = cfg.pos_algs;
      obj.vel_algs       = cfg.vel_algs;
      obj.init();
    end

    function gen_report(obj, quest, vest, qekf)
      import mlreportgen.report.* 
      import mlreportgen.dom.* 
      setDefaultNumberFormat(obj.num_format);
      obj.rpt       = Report(strcat(obj.toutDir,'rpt_',obj.TID,'_report'), ...
                             obj.rpt_outFormat);
      tp            = TitlePage();
      p             = Paragraph(obj.rpt_title);
      p.Style       = {HAlign("center"), ...
                       FontFamily(obj.font_style),...
                       FontSize("32pt"),...
                       OuterMargin('0.0in','0in','0in','12pt'), ...
                       WhiteSpace('preserve')};
      tp.Title      = p;
      p             = Paragraph(obj.rpt_subtitle); 
      p.Style       = {FontSize("20pt"), ...
                       HAlign("center"), ...
                       FontFamily(obj.font_style),...
                       OuterMargin('0.0in','0in','0in','24pt'), ...
                       WhiteSpace('preserve')};
      tp.Subtitle   = p;
      p             = Paragraph(obj.rpt_author); 
      p.Style       = {FontSize("18pt"), ...
                       HAlign("center"), ...
                       FontFamily(obj.font_style),...
                       OuterMargin('0.0in','0in','0in','6pt'), ...
                       WhiteSpace('preserve')};
      tp.Author     = p;
      p             = Paragraph("The Works"); 
      p.Style       = {FontSize("18pt"), ...
                       HAlign("center"), ...
                       FontFamily(obj.font_style),...
                       OuterMargin('0.0in','0in','0in','6pt'), ...
                       WhiteSpace('preserve')};
      tp.Publisher  = p;
      p             = Paragraph(date()); 
      p.Style       = {FontSize("18pt"), ...
                       HAlign("center"), ...
                       FontFamily(obj.font_style),...
                       OuterMargin('0.0in','0in','0in','6pt'), ...
                       WhiteSpace('preserve')};
      tp.PubDate    = p;
      append(obj.rpt, tp);
      append(obj.rpt, TableOfContents);
      chap = obj.get_chap(quest);
      append(obj.rpt, chap);
      chap = obj.get_chap(vest);
      append(obj.rpt, chap);
      chap = obj.get_chap(qekf);
      append(obj.rpt, chap);
      close(obj.rpt);
      if obj.rpt_shw_en 
        rptview(obj.rpt); 
      end
    end

    function gen_plots(obj, dat, dlog, quest, vest, qekf)
      log               = dlog.log;
      [rgt_T, rgt_Q]    = dat.get_kframe_rgtruths();
      quest.res{3}      = obj.plt_log_QuEstp(log, rgt_T, rgt_Q);
      %vest_figname      = obj.plt_log_VEstp(log, rgt_T, rgt_Q);
      vest.res{3}       = nan;
      qekf.res{3}       = obj.plt_log_QEKF(log, rgt_T, rgt_Q);
      disp("plots generated!");
    end 

  end % end of public access 
  methods (Access = private)
    
    function init(obj)
      obj.pos_numAlgs    = length(obj.pos_algs);
      obj.vel_numAlgs    = length(obj.vel_algs);
    end

    function chap = get_chap(obj, mod)
      import mlreportgen.report.* 
      import mlreportgen.dom.* 
      
      chap = Chapter(Title = mod.mod_name);
      append(chap, Section("configs: ")); % --->> configs
      p             = Paragraph();
      p.Style       = {HAlign("left"), ...
                       FontFamily(obj.font_style),...
                       FontSize(obj.text_font_size), ...
                       OuterMargin('0.0in','0in','0in','12pt'), ...
                       WhiteSpace('preserve')};
      append(p, Text("TID: "+mod.TID+newline));
      append(p, Text("benchmark: "+mod.benchmark+newline));
      append(p, Text("del_T: "+string(mod.del_T+newline)));
      append(chap, p); 
      tab = Table(mod.res{2}); % --->> res_tab
      tab.Style = [tab.Style 
                  {NumberFormat(obj.num_format),...
                   Width("95%"),...
                   Border("solid"),...
                   ColSep("solid"),...
                   RowSep("solid"),...
                   mlreportgen.dom.FontSize(obj.tab_font_size),...
                   mlreportgen.dom.FontFamily(obj.font_style)
                   }];
      headerStyle = { BackgroundColor("LightBlue"), ...
                Bold(true) };
      firstRow = tab.Children(1);
      firstRow.Style = [{HAlign("center")},headerStyle];
      tab.TableEntriesHAlign = "center";
      tab.TableEntriesVAlign = "middle";      
      append(chap, tab);
      % append log_fig
      %if isstring(mod.res{3}) % --->> log_fig
      %  % shrink
      %  append(sec, Chapter(Title = mod.res{1}, ...
      %                    Content = Image(mod.res{3})));
      %                    %FontSize = obj.text_font_size, ...
      %                    %FontFamilyName = obj.font_style);
      %end
    end 

    function figname = plt_log_QuEstp(obj, log, rgt_T, rgt_Q)
      idx   = log.cntr_hist;
      T     = log.T_hist;
      Q     = log.Q_hist;
      Tx    = nan(log.numKF, log.pos_numAlgs+1); % kfs vs ests+GT
      Ty    = nan(log.numKF, log.pos_numAlgs+1);
      Tz    = nan(log.numKF, log.pos_numAlgs+1);
      Qw    = nan(log.numKF, log.pos_numAlgs+1);
      Qx    = nan(log.numKF, log.pos_numAlgs+1);
      Qy    = nan(log.numKF, log.pos_numAlgs+1);
      Qz    = nan(log.numKF, log.pos_numAlgs+1);
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
    end % function figname = plt_log_QuEstp(obj, log, rgt_T, rgt_Q)
    
    function fname = plt_log_QEKF(obj, log, rgt_T, rgt_Q)
      idx   = log.cntr_hist;
      Z     = log.Z_hist; 
      X     = log.X_hist;
      Y     = log.Y_hist; % separate fig
      Tx    = nan(log.numKF, log.pos_numAlgs+1); % kfs vs ests+GT
      Ty    = nan(log.numKF, log.pos_numAlgs+1);
      Tz    = nan(log.numKF, log.pos_numAlgs+1);
      Vx    = nan(log.numKF, log.pos_numAlgs+1);
      Vy    = nan(log.numKF, log.pos_numAlgs+1);
      Vz    = nan(log.numKF, log.pos_numAlgs+1);
      Qx    = nan(log.numKF, log.pos_numAlgs+1);
      Qy    = nan(log.numKF, log.pos_numAlgs+1);
      Qz    = nan(log.numKF, log.pos_numAlgs+1);
      %YL1   = zeros(log.numKF, log.pos_numAlgs+1);
      %YL2   = zeros(log.numKF, log.pos_numAlgs+1);
      for a = 1:obj.pos_numAlgs
        Xcols = get_cols(a, log.d_X); % --->> get var cols
        Tx(:,a) = X(:, Xcols(1)); % --->> load to plt cols
        Ty(:,a) = X(:, Xcols(2));
        Tz(:,a) = X(:, Xcols(3));  
        Vx(:,a) = X(:, Xcols(4)); 
        Vy(:,a) = X(:, Xcols(5));
        Vz(:,a) = X(:, Xcols(6));  
        Qx(:,a) = X(:, Xcols(7));
        Qy(:,a) = X(:, Xcols(8));
        Qz(:,a) = X(:, Xcols(9));
      end
      Tx(:, end) = rgt_T(:,1); % --->> load ground truth to last col
      Ty(:, end) = rgt_T(:,2);
      Tz(:, end) = rgt_T(:,3);  
      Vx(:, end) = rgt_T(:,1); % watch these
      Vy(:, end) = rgt_T(:,2);
      Vz(:, end) = rgt_T(:,3);  
      Qx(:, end) = rgt_Q(:,2);
      Qy(:, end) = rgt_Q(:,3);
      Qz(:, end) = rgt_Q(:,4);
      fig = figure(); % 9 subplots Txyz Vxyz Qxyz
      sgtitle("QEKF: State Estimates X vs. Measurement Z","Interpreter",'latex');
      fig.Units    = obj.fig_units;
      fig.Position = obj.fig_pos;
      hold on
      for a = 1:obj.pos_numAlgs+1 % +gt
        subplot(9,1, 1); hold on; subtitle('$T_{x}$',"Interpreter",'latex', ... % Tx
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Tx(:,a), "Color",obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
        subplot(9,1, 2); hold on; subtitle('$T_{y}$',"Interpreter",'latex', ... % Ty
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Ty(:,a), "Color",obj.plt_lclrs(a), "Marker", obj.plt_mrkrs(a));
        subplot(9,1, 3); hold on; subtitle('$T_{z}$',"Interpreter",'latex', ... % Tz
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Tz(:,a), "Color",obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));        
        subplot(9,1, 4); hold on; subtitle('$V_{x}$',"Interpreter",'latex', ... % Vx
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Vx(:,a), "Color",obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
        subplot(9,1, 5); hold on; subtitle('$V_{y}$',"Interpreter",'latex', ... % Vy
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Vy(:,a), "Color",obj.plt_lclrs(a), "Marker", obj.plt_mrkrs(a));
        subplot(9,1, 6); hold on; subtitle('$V_{z}$',"Interpreter",'latex', ... % Vz
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Vz(:,a), "Color",obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));        
        subplot(9,1, 7); hold on; subtitle('$Q_{x}$',"Interpreter",'latex', ... % Qx
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Qx(:,a), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
        subplot(9,1, 8); hold on; subtitle('$Q_{y}$',"Interpreter",'latex', ... % Qy
          'fontsize',obj.fig_txt_size); grid on; 
        plot(idx, Qy(:,a), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));  
        subplot(9,1, 9); hold on; subtitle('$Q_{z}$',"Interpreter",'latex', ... % Qz
          'fontsize',obj.fig_txt_size); grid on;
        plot(idx, Qz(:,a), "Color", obj.plt_lclrs(a), "Marker",obj.plt_mrkrs(a));
      end
      hold off
      lg  = legend([obj.pos_algs; "Groundtruth"]); 
      lg.Units    = obj.leg_units;
      lg.Position = obj.leg_pos;
      lg.FontSize = obj.fig_txt_size-4;
      if obj.plt_questp_sav_en
        fname = strcat(obj.toutDir,'plt_QEKF_logs.png');
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

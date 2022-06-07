classdef report_class < matlab.System 
  
  properties % public vars
    % features 
    rpt_shw_en          = true
    plt_sav_questp_en   = false
    %% config
    rpt_title         = 'XEst main()'
    rpt_subtitle      = 'Real-time vision-based navigation module'
    rpt_author        = 'Bardia Mojra'
    text_font_size    = '12pt'
    num_font_size     = '10pt'
    font_style        = 'Times New Roman'
    num_format        = "%1.4f"
    rpt_outFormat     = 'docx'
    plt_txt_size      = 16;
    % cfg (argin)
    test_ID
    test_outDir
    benchmark
    pos_algs
    vel_algs
    % private vars
    pos_numAlgs
    vel_numAlgs
    rpt
    %numBenchmarks
    % private constants 
    plt_clrs = ["#A2142F", "#77AC30", "#0072BD", "#7E2F8E", ...
                "#EDB120", "#4DBEEE", "#D95319", "#77AC30"] % unique per alg
  end
  methods % constructor
    
    function obj = report_class(varargin) % init obj w name-value args
      setProperties(obj,nargin,varargin{:}) 
    end  
  
  end 
  methods (Access = public) 
  
    function load_cfg(obj, cfg)
      obj.test_ID        = cfg.test_ID;                   
      obj.test_outDir    = cfg.test_outDir;
      obj.benchmark      = cfg.benchmark;
      obj.pos_algs       = cfg.pos_algs;
      obj.vel_algs       = cfg.vel_algs;
      obj.init();
    end

    function gen_report(obj, quest, vest, qekf)
      import mlreportgen.report.* 
      import mlreportgen.dom.* 
      obj.rpt     = Document(strcat(obj.test_outDir,'rep_',obj.test_ID,'_report'), ...
        obj.rpt_outFormat);
      tp          = TitlePage; 
      tp.Title    = obj.rpt_title; 
      tp.Subtitle = obj.rpt_subtitle; 
      tp.Author   = obj.rpt_author; 
      append(obj.rpt, tp); 
      append(obj.rpt, TableOfContents); 
      obj.add_sec(quest);
      obj.add_sec(vest);
      obj.add_sec(qekf);
      if obj.rpt_shw_en 
        rptview(obj.rpt); 
      end
    end

    function gen_plots(obj, dat, dlog)
      log               = dlog.log;
      %idx               = log.cntr_hist;
      %kf                = log.kf_hist;
      [rgt_T, rgt_Q]    = dat.get_kframe_rgtruths();
      
      T     = log.T_hist;
      Q     = log.Q_hist;
 
      questp_plt = obj.get_questp_plt(log, T, Q, rgt_T, rgt_Q);

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
      %obj.numBenchmarks     = length(obj.benchmarks);
      obj.pos_numAlgs    = length(obj.pos_algs);
      obj.vel_numAlgs    = length(obj.vel_algs);
    end

    function add_sec(obj, mod)
      import mlreportgen.report.* 
      import mlreportgen.dom.* 
      %open(obj.rpt);
      h1 = Heading1(mod.mod_name); % heading 
      h1.Bold = true;
      append(obj.rpt, h1); 
      t1 = Text(mod.rpt_note); % note 
      t1.FontSize = obj.text_font_size;
      t1.FontFamilyName = obj.font_style;
      append(obj.rpt, t1);
      t1 = Text(mod.res{1}); % benchmark  
      t1.FontSize = obj.text_font_size;
      t1.FontFamilyName = obj.font_style;
      append(obj.rpt, t1);
      tab = mlreportgen.dom.Table(mod.res{2}); % res_tab
      tab.Style = [tab.Style 
                  {NumberFormat(obj.num_format),...
                   Width("70%"),...
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
      append(obj.rpt, tab);
    end 

    function questp_plt = get_questp_plt(obj, log, T, Q, rgt_T, rgt_Q)
      idx  = log.cntr_hist;
      for a = 1:obj.pos_numAlgs
        Tcols = get_cols(a, log.d_T);
        Qcols = get_cols(a, log.d_Q);
        if a == 1 
          Tx = T(:,Tcols(1)); Ty = T(:,Tcols(2)); Tz = T(:,Tcols(3));
          Qw = Q(:,Qcols(1)); Qx = Q(:,Qcols(2)); Qy = Q(:,Qcols(3)); Qz = Q(:,Qcols(4));
        else
          Tx = horzcat(Tx, T(:,Tcols(1)));
          Ty = horzcat(Ty, T(:,Tcols(2)));
          Tz = horzcat(Tz, T(:,Tcols(3)));    
          Qw = horzcat(Qw, Q(:,Qcols(1)));
          Qx = horzcat(Qx, Q(:,Qcols(1)));
          Qy = horzcat(Qy, Q(:,Qcols(2)));
          Qz = horzcat(Qz, Q(:,Qcols(3)));
        end
      end
      % concat ground truth
      Tx = horzcat(Tx, rgt_T(:,1));
      Ty = horzcat(Ty, rgt_T(:,2));
      Tz = horzcat(Tz, rgt_T(:,3));
      Qw = horzcat(Qw, rgt_Q(:,1));
      Qx = horzcat(Qx, rgt_Q(:,1));
      Qy = horzcat(Qy, rgt_Q(:,2));
      Qz = horzcat(Qz, rgt_Q(:,3));

      questp_plt = figure(); % 7 subplots
      hold on
      for a = 1:obj.pos_numAlgs+1 % +gt
        subplot(7,1,1); hold on; subtitle('$T_{x}$',"Interpreter",'latex', ...
          'fontsize',obj.plt_txt_size); % Tx
        plot(idx, Tx(:,a), "Color", obj.plt_clrs(a));
        %yline(0,'--');
        subplot(7,1,2); hold on; subtitle('$T_{y}$',"Interpreter",'latex', ...
          'fontsize',obj.plt_txt_size); % Ty
        plot(idx, Ty(:,a), "Color", obj.plt_clrs(a));
        %yline(0,'--');
        subplot(7,1,3); hold on; subtitle('$T_{z}$',"Interpreter",'latex', ...
          'fontsize',obj.plt_txt_size); % Tz
        plot(idx, Tz(:,a), "Color", obj.plt_clrs(a));
        %yline(0,'--');
        
        subplot(7,1,4); hold on; subtitle('$Q_{w}$',"Interpreter",'latex', ...
          'fontsize',obj.plt_txt_size); % Qw
        plot(idx, Qw(:,a), "Color", obj.plt_clrs(a));
        %yline(0,'--');
        subplot(7,1,5); hold on; subtitle('$Q_{x}$',"Interpreter",'latex', ...
          'fontsize',obj.plt_txt_size); % Qx
        plot(idx, Qx(:,a), "Color", obj.plt_clrs(a));
        %yline(0,'--');
        subplot(7,1,6); hold on; subtitle('$Q_{y}$',"Interpreter",'latex', ...
          'fontsize',obj.plt_txt_size); % Qy
        plot(idx, Qy(:,a), "Color", obj.plt_clrs(a));
        %yline(0,'--');
        subplot(7,1,7); hold on; subtitle('$Q_{z}$',"Interpreter",'latex', ...
          'fontsize',obj.plt_txt_size); % Qz
        plot(idx, Qz(:,a), "Color", obj.plt_clrs(a));
        %yline(0,'--');
      end

      hold off
      lg  = legend([obj.pos_algs; "Groundtruth"]); 
      lg.Position(1:2) = [.8 .3];
      %lg.Location = 'northeast_outside';
      %lg.Layout.Tile = 'North'; 
      lg.FontSize = obj.plt_txt_size-4;

      if obj.plt_sav_questp_en
        % sav to file
      end
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

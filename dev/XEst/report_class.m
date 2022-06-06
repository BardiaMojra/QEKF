classdef report_class < matlab.System 
  
  properties % public vars
    % features 
    rpt_shw_en        = true
    %% config
    rpt_title         = 'XEst main()'
    rpt_subtitle      = 'Real-time vision-based navigation module'
    rpt_author        = 'Bardia Mojra'
    text_font_size    = '12pt'
    num_font_size     = '10pt'
    font_style        = 'Times New Roman'
    num_format        = "%1.4f"
    rpt_outFormat     = 'docx'
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
    plt_clrs = ['#A2142F', '#77AC30', '#0072BD', '#7E2F8E', '#EDB120', ...
                '#4DBEEE', '#D95319','#77AC30'] % unique per alg
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

    function gen_plots(obj, dlog)
      log = dlog.log;
      idx  = log.cntr_hist;
      %kf   = log.kf_hist;
      
      
      T  = log.T_hist;
      Tx = []; Ty = []; Tz = [];
      for a = 1:obj.pos_numAlgs
        cols = get_cols(a,log.d_T);
        Tx = horxcat(Tx, T(:,cols(1)));
        Ty = horxcat(Ty, T(:,cols(2)));
        Tz = horxcat(Tz, T(:,cols(3)));
      end
      % 
      figure(1); % 10 subplots
      for a = 1:obj.pos_numAlgs
        subplot(10,1,1); % all Tx
        plot(idx, Tx(:,a), obj.plt_clrs(a));
        subplot(10,2,1); % all Ty
        plot(idx, Ty(:,a), obj.plt_clrs(a));
        subplot(10,3,1); % all Tz
        plot(idx, Tz(:,a), obj.plt_clrs(a));
      
      end
      
      T_x = horxcat(T_x, a_cols(1));
      Q_hist     = log.Q_hist;
      V_hist     = log.V_hist;
      W_hist     = log.W_hist;
      Z_hist     = log.Z_hist;
      U_hist     = log.U_hist;
      X_hist     = log.X_hist;
      Y_hist     = log.Y_hist;


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

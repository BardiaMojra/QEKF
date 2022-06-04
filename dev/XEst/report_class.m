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
    rpt_outFormat     = 'pdf'
    % cfg (argin)
    test_ID
    test_outDir
    benchmark
    pos_algs
    vel_algs
    % private vars
    pos_numMethods
    vel_numMethods
    rpt
    %numBenchmarks
    % private constants 
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

    function get_plots(obj,  quest, vest, qekf)
       quest_log = quest.res;  
       vest_log = vest.res;
       qekf_log = qekf.log;
    end 

  end % end of public access 
  methods (Access = private)
    
    function init(obj)
      %obj.numBenchmarks     = length(obj.benchmarks);
      obj.pos_numMethods    = length(obj.pos_algs);
      obj.vel_numMethods    = length(obj.vel_algs);
    end

    function add_sec(obj, mod)
      import mlreportgen.report.* 
      import mlreportgen.dom.* 
      %open(obj.rpt);
      h1 = Heading1(mod.name); % heading 
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

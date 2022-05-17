classdef vest_class < config_class & data_class
  %% public vars
  properties 
    %% configs (passed in via cfg obj or set manually)
%     test_ID % unified name for test run 
%     outDir % out dir     
%     method 
%     st_frame      = 1;% start frame index
%     end_frame   = nan;% end frame index
    %% features (private)

    %% quest configs

    %% run-time variables 
    m 
    m_dot 
    v   % frame vel
    w  % frame ang vel 
   
  end
  %% public consts 
  %   properties(Nontunable) 
  %   end
  %% private vars
  %   properties(DiscreteState) 
  %   end
  %% private consts
  %   properties(Access = private) 
  %   end
  methods
    %% constructor
    function obj = vest_class(varargin)
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end  
  end 
 %% public functions
 methods (Access = public)
    function obj = load_cfg(some_config)
%       obj.some = some_config;
    end
    function [obj, v, w] = get_vel(obj, matches)

      [obj.m, obj.m_dot] = vest.prep_matches(obj, matches);
      [v, w]  = vest.PoCo(obj.m, obj.m_dot); % call algorithm 
  
    end 

    function  [v_err, w_err] = get_err(obj, idx, v, w)
      v = obj.normalize(v);
      w = obj.normalize(w);
      v_err = 1/pi * acos(v' * obj.v_gt(idx,:));
      w_err = sqrt(   (w - obj.w_gt(idx))^2   );
    end 

 end % end of public access 
  
  methods (Access = private)
    function normed = normalize(vector)
      normed  = vector/ norm(vector); % normalize v answer 
        if(normed(3)<0) 
            normed = - normed; 
        end 
    end 
    function [m, m_dot] = prep_matches(obj, matches)
      
      disp(matches);


    end % end of prep_matches
    function  v_err_v, err_w = get_err(obj, i); %
      v_est_norm, v_true_norm, i
        
%         v_est_norm = v_est; 
        v_est_norm = v_est / norm(v_est); % normalize v answer 
        if(v_est_norm(3)<0) 
            v_est_norm = -v_est_norm; 
        end 
       v_n = obj.normalize(v_est);
       
%         error_v = sqrt( (v_est_norm(1)-v_true_norm(1))^2 + (v_est_norm(2)-v_true_norm(2))^2 + (v_est_norm(3)-v_true_norm(3))^2 ); 
        error_v =1/pi*acos((v_'*v_true_norm)); 
        error_w = sqrt( (w_est(1)-w_true(1))^2 + (w_est(2)-w_true(2))^2 + (w_est(3)-w_true(3))^2 ); 
        
        v_error_buffer(1, j) = error_v; 
        w_error_buffer(1, j) = error_w; 
        
    end
    function res = get_res(obj)
      % remove NaN entries (corresponding to skipped frames)
      nanIdx = find(sum(isnan(obj.rotErr),2));
      obj.rotErr(nanIdx,:)  = [];
      obj.tranErr(nanIdx,:) = [];
      % statistics of error for rotation and translation estimates
      rotErrM     = mean(obj.rotErr,1);           % Mean
      rotErrS     = std(obj.rotErr,1);            % Standard deviation
      rotErrMd    = median(obj.rotErr,1);         % Median
      rotErrQ1    = quantile(obj.rotErr,0.25, 1); % 25% quartile
      rotErrQ3    = quantile(obj.rotErr,0.75, 1); % 75% quartile
      tranErrM    = mean(obj.tranErr,1);
      tranErrS    = std(obj.tranErr,1);
      tranErrMd   = median(obj.tranErr,1);
      tranErrQ1   = quantile(obj.tranErr,0.25, 1);
      tranErrQ3   = quantile(obj.tranErr,0.75, 1);
      % Table of results
      RowNames = {'Rot err mean';...
                  'Rot err std';...
                  'Rot err median'; 
                  'Rot err Q_1';...
                  'Rot err Q_3';...
                  'Tran err mean';...
                  'Tran err std';...
                  'Tran err median';...
                  'Tran err Q_1';...
                  'Tran err Q_3'};
      data      = [rotErrM;
                   rotErrS;
                   rotErrMd;
                   rotErrQ1;
                   rotErrQ3;
                   tranErrM;
                   tranErrS;
                   tranErrMd;
                   tranErrQ1;
                   tranErrQ3;];
      if obj.numMethods == 1
        res  = table(data(:,1), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 2
        res  = table(data(:,1), ...
                     data(:,2), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 3
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 4
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     data(:,4), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms); 
      elseif obj.numMethods == 5
        res  = table(data(:,1), ...
                     data(:,2), ...
                     data(:,3), ...
                     data(:,4), ...
                     data(:,5), ...
                     'RowNames', RowNames, ...
                     'VariableNames', obj.algorithms);    
      end

      if obj.res_prt_en
        disp(res);
      end 
      if obj.res_sav_en
        fname = strcat(obj.outDir,'res_',obj.test_ID,'_tab.csv');
        writetable(res,fname);
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
  end
  methods(Access = protected)
    function obj = init(obj)
      [obj.dataset,obj.posp] = LoadDataset(obj.dataroot,obj.benchtype,...
        obj.benchnum,obj.st_frame,obj.end_frame);
      if strcmp(obj.benchtype,'KITTI')
        obj.skipFrame = 1; 
      elseif strcmp(obj.benchtype,'NAIST')
        obj.skipFrame = 1; 
      elseif strcmp(obj.benchtype,'ICL')
        obj.skipFrame = 1; 
      elseif strcmp(obj.benchtype,'TUM')
        obj.skipFrame = 1;     
      end
      obj.numImag       = length(obj.dataset.fnames); 
      obj.keyFrames     = 2+obj.skipFrame:1+obj.skipFrame:obj.numImag; 
      obj.numKeyFrames  = length(obj.keyFrames); 
      obj.numMethods    = length(obj.algorithms); 
      obj.rotErr        = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.tranErr       = NaN(obj.numKeyFrames,obj.numMethods); 
      obj.Q             = cell(obj.numKeyFrames,obj.numMethods); 
      obj.T             = cell(obj.numKeyFrames,obj.numMethods);

      [obj.ppoints_i, obj.Ip_i] = GetFeaturePoints(obj.st_frame,obj.dataset,...
        obj.surfThresh);
      obj.cntr          = 0;
      obj.ppoints       = obj.ppoints_i;
      obj.Ip            = obj.Ip_i;
    end  
  end
end

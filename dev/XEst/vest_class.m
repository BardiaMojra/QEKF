classdef vest_class < matlab.System % & config_class
  %% public vars
  properties 
    %% run-time variables 
    m  % current frame feature points 
    m_dot % m(i) - m(i-1) of matched feature points  
    v   % frame vel
    w  % frame ang vel 
   
  end
  methods
    
    function obj = vest_class(varargin) % constructor
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end  
  end 
 methods (Access = public) % public functions
    function obj = load_cfg(obj, cfg)
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
    function [m, m_dot] = prep_matches(obj, matches)
      
      disp(matches);


    end % end of prep_matches
    function normed = normalize(vector)
      normed  = vector/ norm(vector); % normalize v answer 
        if(normed(3)<0) 
            normed = - normed; 
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
end

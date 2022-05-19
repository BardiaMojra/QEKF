classdef vest_class < matlab.System % & config_class
  %% public vars
  properties 
    %% 
    check_w_QuEst_threshold   = 0.05;
    %% run-time variables 
    m  % current frame feature points 
    m_dot % m(i) - m(i-1) of matched feature points  
    v   % frame vel
    w  % frame ang vel 
   
  end
  methods % constructor
    function obj = vest_class(varargin) 
      setProperties(obj,nargin,varargin{:}) % init obj w name-value args
    end  
  end 
  methods (Access = public) 
    %function load_cfg(obj, cfg)
    %end
    function [v, w] = get_vel(obj, matches)
      [obj.m, obj.m_dot] = obj.prep_matches(matches);
      [v, w]  = PoCo(obj.m, obj.m_dot); % call algorithm 
%       disp(v);
%       disp(w);
    end 

    function  [v_err, w_err] = get_err(obj, idx, v, w)
      v = obj.normalize(v);
      w = obj.normalize(w);
      v_err = 1/pi * acos(v' * obj.v_gt(idx,:));
      w_err = sqrt(   (w - obj.w_gt(idx))^2   );
    end 
  
    function check_w_QuEst(obj, W, Q)
      Q_VEst = obj.exp_map(W);   
      dist = obj.phi03_dist(Q, Q_VEst); 
      if dist < obj.check_w_QuEst_threshold % 
        disp('QuEst and VEst rotation estimates DO NOT match!');
        disp('Phi03 dist: '); disp(dist);
      end
    end
  end % end of public access 
  
  methods (Access = private)
    function dist = phi03_dist(~, qr, q)
      dist = (1/pi) * acos(abs( sum(qr .* q)));
    end 

    function [m, m_dot] = prep_matches(~, matches)
      m         = matches.m2; 
      m_dot = matches.m2 - matches.m1;
    end % end of prep_matches

    function normed = normalize(vector)
      normed  = vector/ norm(vector); % normalize v answer 
        if(normed(3)<0) 
            normed = - normed; 
        end 
    end
    
    function Qwxyz = exp_map(~, x)
      if isequal(size(x),[3,1])
        norm_x = norm(x);
        if norm_x == 0
          Qwxyz = [1; 0; 0; 0];
        else
          qxyz = sin(norm_x/2) .* x/norm_x;
          qw = cos(norm_x/2);
          Qwxyz = [qw; qxyz(1); qxyz(2); qxyz(3)];
        end
      end
    end % Qxyzw = exp_map(x)

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

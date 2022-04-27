classdef log_class
  %LOG_CLASS data log class
  % 
  properties
    idx 
    z_hist % state measurement history
    x_hist % state estimate history
    y_hist % state residual history
    P_hist % state estimation covar history
    K_hist %Kalman gain matrix history 
  end
  methods    
    function obj = log_meas(obj, z, idx)
      z = reshape(z,[],1);
      obj.z_hist = cat(1,obj.z_hist,z);
      obj.idx_hist = cat(1,obj.idx_hist,idx);
    end
    function obj = log_update(obj,y,x,P,K)
      y = reshape(y,[],1);
      x = reshape(x,[],1);
      P = reshape(P,[],1);
      K = reshape(K,[],1);
      obj.y_hist = cat(1,obj.y_hist,y);
      obj.x_hist = cat(1,obj.x_hist,x);
      obj.P_hist = cat(1,obj.P_hist,P);
      obj.K_hist = cat(1,obj.K_hist,K);     
    end
  end
end
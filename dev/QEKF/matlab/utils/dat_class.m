classdef dat_class
  %DATA CLASS dataset class definition 
  %   Detailed explanation goes here
  
  properties
    % define default values 
%     name  
    VestScale      = 1.0
    data_rate_inv  = 1/30.0
%     START_         = 0;
%     END_           = nan;
%     PRINT_         = true;
%     SAVE_          = true;
%     srcDir         = '../data/'
%     outDir         = '../mout/'
%     Vxyz_labels    = ['vx','vy','vz'];
%     Txyz_labels    = ['Tx','Ty','Tz'];
%     Wrpy_labels    = ['wr','wp','wy'];
%     Qxyzw_labels   = ['qx','qy','qz','qw'];
%     vicon_labels   = ['Qx_gt','Qy_gt','Qz_gt','Qw_gt','Tx_gt','Ty_gt','Tz_gt'];
    dat;
    Vxyz; % xyz vel
    Txyz; % xyz translation
    Wrpy; % rpy ang vel
    Qxyzw; % Qxyzw ang ori
  end
  methods
    function obj = load(name)
      % class constructor
      if(nargin == 1)
        obj.name           = name;
     

      end

      % init obj
      obj.outDir = obj.outDir+'out_'+obj.name+'/';
      obj.srcDir = obj.srcDir+name+'_df.csv';
      obj.dat    = csvtable(obj.srcDir,1,1); % skip header row and index column 
      obj.Txyz   = obj.dat(:,1:3); % xyz translation
      obj.Vxyz   = obj.dat(:,4:6); % xyz vel
      obj.Wrpy   = obj.dat(:,7:9); % rpy ang vel
      obj.Qxyzw  = obj.dat(:,10:13); % Qxyzw ang ori
      if isnan(obj.END_)
        obj.END_ = length(obj.dat,1);
      end
    end
  end
end

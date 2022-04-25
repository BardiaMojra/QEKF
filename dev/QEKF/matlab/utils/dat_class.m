classdef dat_class
  %DATA CLASS dataset class definition 
  %   Detailed explanation goes here
  
  properties
    % define default values 
    name  
    VestScale      = 1.0
    data_rate_inv  = 1/30.0
    START_         = 0
    END_           = nan
    PRINT_         = true;
    SAVE_          = true;
    srcDir         = '../data/'
    outDir         = '../mout/'
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
    function obj = load(obj, name)
      % class constructor
      obj.name   = name;
      obj.outDir = strcat(obj.outDir,'out_',obj.name,'/');
      obj.srcDir = strcat(obj.srcDir,name,'_df.csv');
      obj.dat    = readtable(obj.srcDir); 
      obj.Txyz   = horzcat(obj.dat.Tx,obj.dat.Ty,obj.dat.Tz); % Txyz 
      obj.Vxyz   = horzcat(obj.dat.vx,obj.dat.vy,obj.dat.vz); % Vxyz 
      obj.Wrpy   = horzcat(obj.dat.wr,obj.dat.wp,obj.dat.wy); % Wrpy ang vel
      obj.Qxyzw  = horzcat(obj.dat.qx,obj.dat.qy,obj.dat.qz,obj.dat.qw); % Qxyzw ang ori
      if isnan(obj.END_)
        obj.END_ = height(obj.dat);
      end
    end
  end
end

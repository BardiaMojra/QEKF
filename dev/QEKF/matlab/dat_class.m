classdef dat_class
  %DAT_CLASS data class
  % 
  properties
    % define default values 
    name  
    VestScale      = 1.0
    data_rate_inv  = 1/30.0
    START_         = 1
    END_           = nan
    PRINT_         = true;
    SAVE_          = true;
    srcDir         = '../data/'
    outDir         = '../mout/'
    dat;           % dataset
    Vxyz;          % xyz vel
    Txyz;          % xyz translation
    Wrpy;          % rpy ang vel
    Qxyzw;         % Qxyzw ang ori
    z_TVQxyzw;     % meas/obs vec
  end
  methods 
    function obj = load(obj, name) % construct data obj 
      obj.name   = name;
      obj.outDir = strcat(obj.outDir,'out_',obj.name,'/');
      obj.srcDir = strcat(obj.srcDir,name,'_df.csv');
      dat        = readtable(obj.srcDir); 
      obj.Txyz   = horzcat(dat.Tx,dat.Ty,dat.Tz); % Txyz 
      obj.Vxyz   = horzcat(dat.vx,dat.vy,dat.vz); % Vxyz 
      obj.Wrpy   = horzcat(dat.wr,dat.wp,dat.wy); % Wrpy ang vel
      obj.Qxyzw  = horzcat(dat.qx,dat.qy,dat.qz,dat.qw); % Qxyzw ang ori
      obj.z_TVQxyzw = horzcat(obj.Txyz,obj.Vxyz,obj.Qxyzw); 
      obj.dat    = dat;
      if isnan(obj.END_)
        obj.END_ = height(obj.dat);
      end
    end
  end
end

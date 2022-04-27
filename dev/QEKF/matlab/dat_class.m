classdef dat_class
  %DAT_CLASS data class
  % 
  properties
    % define default values 
    name  
    VestScale      = 1.0;
    data_rate_inv  = 1/30.0;
    START_         = 1;
    END_           = nan;
    PRINT_         = true;
    SAVE_          = true;
    srcDir         = '../data/';
    outDir         = '../mout/';
    dat;           % dataset
    Vxyz;          % xyz vel
    Txyz;          % xyz translation
    Wrpy;          % rpy ang vel
    Qxyzw;         % Qxyzw ang ori
    z_TVQxyzw;     % meas/obs vec
    u_Wrpy;        % cont/inp vec
  end
  methods 
    function obj = load(obj, name) % construct data obj 
      obj.name      = name;
      obj.outDir    = strcat(obj.outDir,'out_',obj.name,'/');
      obj.srcDir    = strcat(obj.srcDir,name,'_df.csv');
      dt            = readtable(obj.srcDir); 
      obj.Txyz      = horzcat(dt.Tx,dt.Ty,dt.Tz); % Txyz 
      obj.Vxyz      = horzcat(dt.vx,dt.vy,dt.vz); % Vxyz 
      obj.Wrpy      = horzcat(dt.wr,dt.wp,dt.wy); % Wrpy ang vel
      obj.Qxyzw     = horzcat(dt.qx,dt.qy,dt.qz,dt.qw); % Qxyzw ang ori
      obj.z_TVQxyzw = horzcat(obj.Txyz,obj.Vxyz,obj.Qxyzw); 
      obj.u_Wrpy    = obj.Wrpy;
%       obj.z_TVQxyzw = table2array(obj.z_TVQxyzw);
      obj.dat       = horzcat(obj.Txyz,obj.Vxyz,obj.Wrpy,obj.Qxyzw);
%       obj.dat       = dt;
      if isnan(obj.END_)
        obj.END_ = height(obj.dat);
      end
    end
  end
end

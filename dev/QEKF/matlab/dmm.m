classdef dmm
  %DMM data management module
  %   Detailed explanation goes here

% write a description of the class here.
   properties
   % define default values 
     name;
     VestScale      = 1.0
     data_rate_inv  = 1/30.0;
     START_         = 0;
     END_           = None;
     PRINT_         = True;
     SAVE_          = True;
     srcDir         = '../data/'
     outDir         = '../mout/'
     Vxyz_labels    = ['vx','vy','vz'];
     Txyz_labels    = ['Tx','Ty','Tz'];
     Wrpy_labels    = ['wr','wp','wy'];
     Qxyzw_labels   = ['qx','qy','qz','qw'];
     vicon_labels   = ['Qx_gt','Qy_gt','Qz_gt','Qw_gt','Tx_gt','Ty_gt','Tz_gt'];
     dat;
     Vxyz; % xyz vel
     Txyz; % xyz translation
     Wrpy; % rpy ang vel
     Qxyzw; % Qxyzw ang ori
   end
   methods
   % methods, including the constructor are defined in this block
     function obj = dmm(name,VestScale,data_rate_inv,START_,END_)
     % class constructor
       if(nargin > 0)
         obj.name           = name;
         obj.VestScale      = VestScale;
         obj.data_rate_inv  = data_rate_inv;
         obj.START_         = START_;
         obj.END_           = END_;
       end

       % init obj
       obj.outDir = obj.outDir+'out_'+name+'/'
       obj.srcDir = obj.srcDir+name+'_df.csv'
       obj.dat    = csvtable(obj.srcDir,1,1) % skip header row and index column 
       obj.Vxyz   = ; % xyz vel
       obj.Txyz; % xyz translation
       obj.Wrpy; % rpy ang vel
       obj.Qxyzw; % Qxyzw ang ori

     end
   end
end

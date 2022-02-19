function [m, m_dot] = Create_M_and_Mdot(v,omega,noise_level)
    %% cam1 configuration -------------------------------------------------
    phi_cam1 = 0;       % rotation along x axis 
    beta_cam1 = 0;     % rotation along y axis 
    alpha_cam1 = 0;     % rotation along z axis 

    x_cam1 = 0;       % translation along x axis 
    y_cam1 = 0;         % translation along y axis 
    z_cam1 = 0;         % translation along z axis 
    
    % initial pose of the camera 
    T0 = SE3(x_cam1, y_cam1, z_cam1) * SE3.Rx(phi_cam1) * SE3.Ry(beta_cam1) * SE3.Rz(alpha_cam1); 
    
%     cam1 = CentralCamera('name','cam1', ...
%                          'centre',[512;360], ...
%                          'pose',T0, ... 
%                          'resolution',[1024,720], ...
%                          'color',[1 1 1], ... 
%                          'focal',0.045); % 'image',img_1); % create a camera  

    cam1 = CentralCamera('default', 'color',[1 1 1], ... 
                        'centre', [512;512], ...
                        'pose', T0); 

%     %visualization 
%     figure(1) 
%     axis([-10 10 -10 10 0 10]); 
%     cam1.plot_camera('color','b'); 

    %% movement of cam1 
    
    % v 
    x_move = v(1);         % x movement 
    y_move = v(2);         % y movement 
    z_move = v(3);         % z movement 
    
    % omega 
    phi_move = omega(1);       % rotate along x 
    beta_move = omega(2);      % rotate along y 
    alpha_move = omega(3);     % rotate along z 

    T1 = SE3(x_move, y_move, z_move) * SE3.Rx(phi_move) * SE3.Ry(beta_move) * SE3.Rz(alpha_move); 

    cam1_1 = cam1.move(T1); 

%     %visualization 
%     figure(2); 
%     axis([-10 10 -10 10 0 10]); 
%     cam1_1.plot_camera('color','r'); 

    %% create points 

%     Points = [-1    -.5     0       0      .5      1        1 
%               .5    0       -.5     -1     0       .5       0
%               3     4.1     4.2     3.8    3.5     3.6    2.5]; 

    Points = [-2    -1.5     -1      -.5    0      .5       1 
              .5    0       -.5     -1     0       .5       -.5
              6     7.1     7.2     6.8    6.5      6.6    6.5]; 
          
%     figure(1) 
%     axis([-5 5 -5 5 0 8]); 
%     plot_sphere(Points, 0.1,'r'); 
%     plot_sphere(Points_1, 0.1,'b'); 
%     cam1.plot_camera('color','b'); 


    p1 = cam1.plot(Points,'sequence');     % pixel points from cam1 

    p1_1 = cam1_1.plot(Points);   % pixel points from cam1_after_move 
    
    num_points = size(Points, 2); 
    
    %% feature tracking method of generating optical flows 
    % append 1 at the bottom 
    p1_image = [p1; ones(1,num_points)] + [noise_level*randn(2,num_points);zeros(1,num_points)]; 
    
    p1_image_nonoise = [p1; ones(1,num_points)]; 
    p1_image_noise = p1_image; 
    
    p1_1_image = [p1_1; ones(1,num_points)]; 

    K1 = cam1.K; 

    % image points from cam1 and cam1_1 
    p1_uv = inv(K1) * p1_image; 
    p1_1_uv = inv(K1) * p1_1_image; 

    %% flowfield method of generating optical flows 
    
    vel_flow = [x_move y_move z_move phi_move beta_move alpha_move]; 

    du = [1, num_points]; 
    dv = [1, num_points]; 
    
    for i=1:num_points 
    
        pdot = -cam1.visjac_p( [p1_image_nonoise(1,i); p1_image_nonoise(2,i)], Points(3,i) ) * vel_flow(:); %+.1*randn(2,1); 
        du(1,i) = pdot(1,1); %+ [noise_level*randn(1,1)];
        dv(1,i) = pdot(2,1); %+ [noise_level*randn(1,1)];
    
    end 

    %% result 
    
    % version1, feature vector subtraction 
    m_v1 = p1_uv; 
    m_dot_v1 = p1_1_uv - p1_uv; 
    
    % version2, jacob matrix 
    m_v2 = p1_uv; 
    m_dot_v2 = inv(K1) * [du; dv; zeros(1,num_points)]; 
    
    % final result of points and optical flow 
    m = m_v2; 
    m_dot = m_dot_v2; 

end 



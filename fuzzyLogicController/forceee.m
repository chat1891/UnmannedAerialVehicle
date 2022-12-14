% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%
%  Revised:
%   3/5/2010  - RB 
%   5/14/2010 - RB

function y = forceee(uu, P)

    % relabel the inputs
   pn      = uu(1);
   pe      = uu(2);
    pd      = uu(3);
   u       = uu(4);
   v       = uu(5);
   w       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
   psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    F_x     = uu(13);
    F_y     = uu(14);
    F_z     = uu(15);
%    M_l     = uu(16);
%    M_m     = uu(17);
%    M_n     = uu(18);
    Va      = uu(19);
%    alpha   = uu(20);
%    beta    = uu(21);
%    wn      = uu(22);
%    we      = uu(23);
%    wd      = uu(24);
    
    % simulate rate gyros (units are rad/sec)
%     y_gyro_x = p + P.bias_gyro_x + P.sigma_gyro*randn;
%     y_gyro_y = q + P.bias_gyro_y + P.sigma_gyro*randn;
%     y_gyro_z = r + P.bias_gyro_z + P.sigma_gyro*randn;
% 
%     % simulate accelerometers (units of g)
%     y_accel_x = F_x/P.mass+P.gravity*sin(theta)+P.sigma_accel*randn;
%     y_accel_y = F_y/P.mass - P.gravity*cos(theta)*sin(phi) + P.sigma_accel*randn;
%     y_accel_z = F_z/P.mass - P.gravity*cos(theta)*cos(phi) + P.sigma_accel*randn;
    
    %%
    %calculate actual accelaration 
    acll_x=F_x/P.mass+P.gravity*sin(theta);
    acll_y=F_y/P.mass- P.gravity*cos(theta)*sin(phi);
    acll_z=F_z/P.mass- P.gravity*cos(theta)*cos(phi);
    
    distance = 3000-pn;
    h=-pd;
    
%     yaw = psi;  
% pitch = theta; 
% roll = phi;
P.dcm = angle2dcm( phi, theta, psi );


    % construct total output
    y = [...
        acll_x;...
        acll_y;...
        acll_z;...
        u;...
        v;...
        w;...
        pn;...
        pe;...
        h;...
        distance;...
        dcm;... 
    ];

end




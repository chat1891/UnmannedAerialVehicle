function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
 % persistent variables that define random walk of GPS sensors
    persistent eta_n;
    persistent eta_e;
    persistent eta_h;
    
    if t==0  % initialize persistent variables
        eta_n = 0;
        eta_e = 0;
        eta_h = 0;
    else      % propagate persistent variables
        eta_n = exp(-P.beta_gps*P.Ts_gps)*eta_n + P.sigma_n_gps*randn;
        eta_e = exp(-P.beta_gps*P.Ts_gps)*eta_e + P.sigma_e_gps*randn;
        eta_h = exp(-P.beta_gps*P.Ts_gps)*eta_h + P.sigma_h_gps*randn;
    end

    % construct North, East, and altitude GPS measurements
    y_gps_n = pn + eta_n;
    y_gps_e = pe + eta_e; 
    y_gps_h = -pd + eta_h; 
    
    % construct groundspeed and course measurements
    y_gps_Vg     = sqrt((Va*cos(psi)+wn)^2+(Va*sin(psi)+we)^2)+P.sigma_Vg_gps*randn;
    y_gps_course = atan2(Va*sin(psi)+we,Va*cos(psi)+wn)+P.sigma_course_gps*randn;

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
end
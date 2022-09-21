% estimate_states
%   - Estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
 
   
    % Define persistent variables
    persistent alpha  % constant for low pass filter - only compute once
    persistent lpf_gyro_x   % low pass filter of x-gyro
    persistent lpf_gyro_y   % low pass filter of y-gyro
    persistent lpf_gyro_z   % low pass filter of z-gyro
    persistent lpf_static   % low pass filter of static pressure sensor
    persistent lpf_diff     % low pass filter of diff pressure sensor
    persistent xhat_a       % estimate of roll and pitch
    persistent P_a          % error covariance for roll and pitch angles
    persistent xhat_p       % estimate of pn, pe, Vg, chi, wn, we, psi
    persistent P_p          % error covariance for pn, pe, Vg, chi, wn, we, psi
    persistent y_gps_n_old  % last measurement of gps_n - used to detect new GPS signal
    persistent y_gps_e_old  % last measurement of gps_e - used to detect new GPS signal
    persistent y_gps_Vg_old % last measurement of gps_Vg - used to detect new GPS signal
    persistent y_gps_course_old  % last measurement of gps_course - used to detect new GPS signal
    
    persistent phihat
    persistent thetahat
    persistent pnhat
    persistent pehat
    persistent psihat
    persistent chihat
    persistent Vghat
    persistent wnhat
    persistent wehat
    persistent phat;
    persistent qhat;
    persistent rhat;
    persistent hhat;
    persistent Vahat;
    persistent alphahat;
    persistent betahat;

    
    % Initialize persistent variables
    

    %---------------------tuning variables ----------------
    Q_atti=10^-9*diag([1,1]);
    Q_gps = diag([1 1 0.1 0.001 0.001 0.01 0.001]);
    lpf_a = 225; % 'a' constant in low pass filter 'alpha' equation
    alpha = 0.1;%exp(-lpf_a*P.Ts);
    P.wind_n=3;
    P.wind_e=2;
    %-----------------------------------------------------------
    if t==0
        % Initialize all variables here....
        
        lpf_gyro_x = 0;%y_gyro_x;
        lpf_gyro_y = 0;%y_gyro_y;
        lpf_gyro_z = 0;%y_gyro_z;
        y_gps_n_old = P.pn0; 
        y_gps_e_old = P.pe0;
        y_gps_Vg_old = 35;%norm(P.x_trim(4:6));
        y_gps_course_old = 0;
        lpf_static=P.pd0 * P.rho * P.gravity;
        lpf_diff=P.Va0^2 * P.rho / 2;
        P_p=zeros(7,7); 
        P_a=diag([(15*pi/180)^2,(15*pi/180)^2]);
        
        wnhat=0; %3
        wehat=0; %2
        
        phihat = 0; %
		thetahat = 0; %
        xhat_a = [P.phi0;P.theta0];
        xhat_p = [P.pn0; P.pe0; P.Va0; P.psi0;0;0;P.psi0];%%%%%%%%%%
        
        pnhat = 0;
        pehat = 0;
        Vghat = P.Va0;
        chihat = 0;
        psihat = 0;   
        
        phat = 0; %
        qhat = 0; %
        rhat = 0; %
        hhat = 0; %
        Vahat = P.Va0; %
        
    end
      
%% Low pass filter gyros to estimate angular rates
    
    % Implement gyro low pass filers here....
    lpf_gyro_x = alpha * lpf_gyro_x + (1-alpha)*y_gyro_x;
    lpf_gyro_y = alpha * lpf_gyro_y + (1-alpha)*y_gyro_y;
    lpf_gyro_z = alpha * lpf_gyro_z + (1-alpha)*y_gyro_z;

    phat=lpf_gyro_x;
    qhat=lpf_gyro_y;
    rhat=lpf_gyro_z;

    
%% Low pass filter static pressure sensor and invert to estimate
    
    % Implement altitude low pass filers here....
    lpf_static = alpha * lpf_static+ (1-alpha)*y_static_pres;
    hhat = lpf_static / (P.rho * P.gravity);
    
%% Low pass filter diff pressure sensor and invert to estimate Va
    
    % Implement low pass filers here....
    lpf_diff= alpha * lpf_diff+ (1-alpha)*y_diff_pres;   
    Vahat = sqrt(2/P.rho*lpf_diff)       
    
%% Implement continous-discrete EKF to estimate roll and pitch angles
    
    % Set up the covariance matrices for attitude EKF here....
   
    N = 10; % Choose timestep value between measurement update (5-20)
    %% Prediction step
    for i=1:N
    
    % Implement prediction step here....
    f_atti=[phat+qhat*sin(phihat)*tan(thetahat)+rhat*cos(phihat)*tan(thetahat);...
                qhat*cos(phihat)-rhat*sin(phihat)];
    xhat_a=xhat_a+(P.Ts/N)*f_atti;
    
    A_atti = [ qhat*cos(phihat)*tan(thetahat)-rhat*sin(phihat)*tan(thetahat), ...
                    (qhat*sin(phihat)+rhat*cos(phihat))/(cos(thetahat))^2;...
                    -qhat*sin(phihat)-rhat*cos(phihat),...
                    0]; %df/dx
                
    P_a = P_a+(P.Ts/N)*(A_atti*P_a+P_a*A_atti'+Q_atti);     
    
    phihat = xhat_a(1);
    thetahat = xhat_a(2);
    
    end
    %% Measurement Update step
    % Implement measurement update step here....
    R_accel = P.sigma_accel^2;
    C_1 = [0 qhat*Vahat*cos(thetahat)+P.gravity*cos(thetahat)];
    L_1 = P_a * C_1' / (R_accel + C_1*P_a*C_1');
    P_a = (eye(2) - L_1*C_1) * P_a;
    h_1 = qhat*Vahat*sin(thetahat) + P.gravity*sin(thetahat);
    phihat = phihat + L_1(1) * (y_accel_x - h_1);
    thetahat = thetahat + L_1(2) * (y_accel_x - h_1);
    
    C_2 = [-P.gravity*cos(phihat)*cos(thetahat)   -rhat*Vahat*sin(thetahat)-phat*Vahat*cos(thetahat)+P.gravity*sin(phihat)*sin(thetahat)];
    L_2 = P_a * C_2' / (R_accel + C_2*P_a*C_2');
    P_a = (eye(2) - L_2*C_2) * P_a;
    h_2 = rhat*Vahat*cos(thetahat) - phat*Vahat*sin(thetahat) - P.gravity*cos(thetahat)*sin(phihat);
    phihat = phihat + L_2(1) * (y_accel_y - h_2);
    thetahat = thetahat + L_2(2) * (y_accel_y - h_2);
    
    C_3 = [P.gravity*sin(phihat)*cos(thetahat)    (qhat*Vahat+P.gravity*cos(phihat))*sin(thetahat)];
    L_3 = P_a * C_3' / (R_accel + C_3*P_a*C_3');
    P_a = (eye(2) - L_3*C_3) * P_a;
    h_3 = -qhat*Vahat*cos(thetahat) - P.gravity*cos(thetahat)*cos(phihat);
    phihat = phihat + L_3(1) * (y_accel_z - h_3);
    thetahat = thetahat + L_3(2) * (y_accel_z - h_3);
    

    %% Final roll and pitch estimates
%     phihat   = xhat_a(1);
%     thetahat = xhat_a(2);

%% Implement continous-discrete EKF to estimate pn, pe, chi, Vg
    
    % Setup the covariance matrices for GPS Smoothing here....
    
    N = 10; % Choose timestep value between measurement update (5-20)
    %% Prediction step
    for i=1:N
    
    % Implement prediction step here....
    psihat_dot = qhat*sin(phihat)/cos(thetahat)+rhat*cos(phihat)/cos(thetahat);
    Vghat_dot = psihat_dot*Vahat*(-sin(psihat)*wnhat+cos(psihat)*wehat)/Vghat;
    chihat_dot =  (P.gravity/Vghat)*tan(phihat)*cos(chihat-psihat);
    
    f_gps = [Vghat*cos(chihat);...
                 Vghat*sin(chihat);...
                 Vghat_dot;...
                 chihat_dot;...
                 0;...
                 0;...
                 psihat_dot]
    xhat_p=xhat_p+(P.Ts/N)*f_gps;
    
    dVgdot_dpsi = -psihat_dot*Vahat*(wnhat*cos(psihat)+wehat*sin(psihat));
    dchidot_dVg = -P.gravity/Vghat^2*tan(phihat)*sin(chihat-psihat);
    dchidot_dchi = -P.gravity/Vghat*tan(phihat)*sin(chihat-psihat);
    dchidot_dpsi = P.gravity/Vghat*tan(phihat)*sin(chihat-psihat);
    
    A_gps=[ 0, 0, cos(chihat), -Vghat*sin(chihat), 0, 0, 0;...
            0, 0, sin(chihat), Vghat*cos(chihat), 0, 0, 0;...
            0, 0, -Vghat_dot/Vghat, 0, -psihat_dot*Vahat*sin(psihat)/Vghat, psihat_dot*Vahat*sin(psihat)/Vghat, dVgdot_dpsi;... 
            0, 0, dchidot_dVg, dchidot_dchi, 0, 0, dchidot_dpsi;...
            0, 0, 0, 0, 0, 0, 0;...
            0, 0, 0, 0, 0, 0, 0;...
            0, 0, 0, 0, 0, 0, 0;...
            ];
   P_p = P_p+(P.Ts/N)*(A_gps*P_p+P_p*A_gps'+Q_gps);
    end
    
    %% Measurement Update step
    % Since the GPS sensor measurements are updated at a slower rate (1 Hz)
    % than what the simulation is running at, the measurement update step
    % is conditioned to be executed only when measurements are available,
    % by comparing current GPS values to previous values.
    if   (y_gps_n~=y_gps_n_old)...
        || (y_gps_e~=y_gps_e_old)...
        || (y_gps_Vg~=y_gps_Vg_old)...
        || (y_gps_course~=y_gps_course_old)
    
        % Make sure to wrap the course measurement between -pi and pi if
        % the course error (y_gps_course - xhat_p(4)) is outside that
        % range...

  
    C_1 = [1 0 0 0 0 0 0];
    R_gps_n = P.sigma_n_gps^2;
    L_1 = P_p * C_1' / (R_gps_n + C_1*P_p*C_1');
    P_p = (eye(7) - L_1*C_1) * P_p;
    h_1 = pnhat;
    pnhat = pnhat + L_1(1) * (y_gps_n - h_1);
    pehat = pehat + L_1(2) * (y_gps_n - h_1);
    Vghat = Vghat + L_1(3) * (y_gps_n - h_1);
    chihat = chihat + L_1(4) * (y_gps_n - h_1);
    wnhat = wnhat + L_1(5) * (y_gps_n - h_1);
    wehat = wehat + L_1(6) * (y_gps_n - h_1);
    psihat = psihat + L_1(7) * (y_gps_n - h_1);
    
    C_2 = [0 1 0 0 0 0 0];
    R_gps_e = P.sigma_e_gps^2;
    L_2 = P_p * C_2' / (R_gps_e + C_2*P_p*C_2');
    P_p = (eye(7) - L_2*C_2) * P_p;
    h_2 = pehat;
    pnhat = pnhat + L_2(1) * (y_gps_e - h_2);
    pehat = pehat + L_2(2) * (y_gps_e - h_2);
    Vghat = Vghat + L_2(3) * (y_gps_e - h_2);
    chihat = chihat + L_2(4) * (y_gps_e - h_2);
    wnhat = wnhat + L_2(5) * (y_gps_e - h_2);
    wehat = wehat + L_2(6) * (y_gps_e - h_2);
    psihat = psihat + L_2(7) * (y_gps_e - h_2);
    
    C_3 = [0 0 1 0 0 0 0];
    R_gps_Vg = P.sigma_Vg_gps^2;
    L_3 = P_p * C_3' / (R_gps_Vg + C_3*P_p*C_3');
    P_p = (eye(7) - L_3*C_3) * P_p;
    h_3 = Vghat;
    pnhat = pnhat + L_3(1) * (y_gps_Vg - h_3);
    pehat = pehat + L_3(2) * (y_gps_Vg - h_3);
    Vghat = Vghat + L_3(3) * (y_gps_Vg - h_3);
    chihat = chihat + L_3(4) * (y_gps_Vg - h_3);
    wnhat = wnhat + L_3(5) * (y_gps_Vg - h_3);
    wehat = wehat + L_3(6) * (y_gps_Vg - h_3);
    psihat = psihat + L_3(7) * (y_gps_Vg - h_3);
    
    C_4 = [0 0 0 1 0 0 0];
    R_gps_chi = P.sigma_course_gps^2
    L_4 = P_p * C_4' / (R_gps_chi + C_4*P_p*C_4');
    P_p = (eye(7) - L_4*C_4) * P_p;
    h_4 = chihat;
    pnhat = pnhat + L_4(1) * (y_gps_course - h_4);
    pehat = pehat + L_4(2) * (y_gps_course - h_4);
    Vghat = Vghat + L_4(3) * (y_gps_course - h_4);
    chihat = chihat + L_4(4) * (y_gps_course - h_4);
    wnhat = wnhat + L_4(5) * (y_gps_course - h_4);
    wehat = wehat + L_4(6) * (y_gps_course - h_4);
    psihat = psihat + L_4(7) * (y_gps_course - h_4);
    
    C_5 = [0 0 -cos(chihat) Vghat*sin(chihat) 1 0 -Vahat*sin(psihat)];
    R_gps_wn = 0.000000001;
    L_5 = P_p * C_5' / (R_gps_wn + C_5*P_p*C_5');
    P_p = (eye(7) - L_5*C_5) * P_p;
    h_5 = Vahat*cos(psihat) + P.wind_n - Vghat*cos(chihat);
    pnhat = pnhat + L_5(1) * (P.wind_n - h_5);
    pehat = pehat + L_5(2) * (P.wind_n - h_5);
    Vghat = Vghat + L_5(3) * (P.wind_n - h_5);
    chihat = chihat + L_5(4) * (P.wind_n - h_5);
    wnhat = wnhat + L_5(5) * (P.wind_n - h_5);
    wehat = wehat + L_5(6) * (P.wind_n - h_5);
    psihat = psihat + L_5(7) * (P.wind_n - h_5);

    C_6 = [0 0 -sin(chihat) -Vghat*cos(chihat) 0 1 Vahat*cos(psihat)];
    R_gps_we = 0.0000001;
    L_6 = P_p * C_6' / (R_gps_we + C_6*P_p*C_6');
    P_p = (eye(7) - L_6*C_6) * P_p;
    h_6 = Vahat*sin(psihat) + P.wind_e - Vghat*sin(chihat);
    pnhat = pnhat + L_6(1) * (P.wind_e - h_6);
    pehat = pehat + L_6(2) * (P.wind_e - h_6);
    Vghat = Vghat + L_6(3) * (P.wind_e - h_6);
    chihat = chihat + L_6(4) * (P.wind_e - h_6);
    wnhat = wnhat + L_6(5) * (P.wind_e - h_6);
    wehat = wehat + L_6(6) * (P.wind_e - h_6);
    psihat = psihat + L_6(7) * (P.wind_e - h_6);
    
    chihat =chihat
    
  
    %%
        % update stored GPS signals
        y_gps_n_old      = y_gps_n;
        y_gps_e_old      = y_gps_e;
        y_gps_Vg_old     = y_gps_Vg;
        y_gps_course_old = y_gps_course;

    end
 
    
%     pnhat    = xhat_p(1);
%     pehat    = xhat_p(2);
%     Vghat    = xhat_p(3);
%     chihat   = wrapToPi(xhat_p(4)); %%%
%     wnhat    = xhat_p(5);
%     wehat    = xhat_p(6);
%     psihat   = wrapToPi(xhat_p(7)); %%%
  
    % Not estimating these states 
    alphahat = thetahat;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
    xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat];
end

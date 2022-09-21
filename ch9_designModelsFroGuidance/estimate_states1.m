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
    att_y = [y_accel_x;y_accel_y;y_accel_z];
    att_R = P.sigma_accel^2;
    
        %for i=1:3
        %if(abs(att_y(i) - att_y_old(i)) > 2)
            
    att_C =  [0, qhat*Vahat*cos(thetahat)+P.gravity*cos(thetahat); 
      -P.gravity*cos(phihat)*cos(thetahat), ...
      -rhat*Vahat*sin(thetahat)-phat*Vahat*cos(thetahat)+P.gravity*sin(phihat)*sin(thetahat);
       P.gravity*sin(phihat)*cos(thetahat), (qhat*Vahat+P.gravity*cos(phihat))*sin(thetahat)];
    att_L = P_a*att_C'/(att_R + att_C*P_a*att_C');
    P_a = (diag([1,1])-att_L*att_C)*P_a; 
    att_h_xhat_u = [qhat*Vahat*sin(thetahat)+P.gravity*sin(thetahat);
                rhat*Vahat*cos(thetahat)-phat*Vahat*sin(thetahat)-P.gravity*cos(thetahat)*sin(phihat);
                -qhat*Vahat*cos(thetahat)-P.gravity*cos(thetahat)*cos(phihat)];
    xhat_a =  xhat_a + att_L*(att_y - att_h_xhat_u);
    
   % end

    

    %% Final roll and pitch estimates
    phihat   = xhat_a(1);
    thetahat = xhat_a(2);


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
%%
        % Implement GPS measurement update step here....
    gps_y = [y_gps_n; y_gps_e; y_gps_Vg; y_gps_course;P.wind_n;P.wind_e];
    gps_R = [P.sigma_n_gps^2; P.sigma_e_gps^2; P.sigma_Vg_gps^2; P.sigma_course_gps^2; 0.000001; 0.000001];
    
    %for i=1:6
        
            
            gps_C = [1 0 0 0 0 0 0;...
               0 1 0 0 0 0 0;...
               0 0 1 0 0 0 0;...
               0 0 0 1 0 0 0;...
               0 0 -cos(chihat) Vghat*sin(chihat) 1 0 -Vahat*sin(psihat);...
               0 0 -sin(chihat) -Vghat*cos(chihat) 0 1 Vahat*cos(psihat)];
           
            gps_L = P_p*gps_C'/(gps_R + gps_C*P_p*gps_C');
            P_p = (eye(7)-gps_L*gps_C)*P_p; 
            gps_h_xhat_u = [pnhat;...
                            pehat;...
                            Vghat;... 
                            chihat;...
                            Vahat*cos(psihat) + P.wind_n - Vghat*cos(chihat);...
                            Vahat*sin(psihat) + P.wind_e - Vghat*sin(chihat)];...
           
            xhat_p =  xhat_p + gps_L*(gps_y - gps_h_xhat_u);
                    
       
   % end


  
    %%
        % update stored GPS signals
        y_gps_n_old      = y_gps_n;
        y_gps_e_old      = y_gps_e;
        y_gps_Vg_old     = y_gps_Vg;
        y_gps_course_old = y_gps_course;

    end
 
    
    pnhat    = xhat_p(1);
    pehat    = xhat_p(2);
    Vghat    = xhat_p(3);
    chihat   = xhat_p(4); %%%
    wnhat    = xhat_p(5);
    wehat    = xhat_p(6);
    psihat   = xhat_p(7); %%%
  
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

P.gravity = 9.8; % (m/s^2)
P.Ts = 0.01; % Sample rate

% Wind conditions
P.w_mag = 0; % Wind speed at 6 m altitude (m/s)
P.w_dir = 0; % Wind direction at 6 m altitude (deg clockwise from north)

%% Compute Trim Conditions using 'mavsim_chap5_trim.slx'
P.Va0 = 35; % m/s (~85 mph)
gamma = 0*pi/180; % Desired flight path angle (radians)
R     = Inf; % Desired radius (m) - use (+) for right handed orbit, 
h0    = 100;%500; % Initial altitude

% First cut at Initial Conditions
P.pn0    = 0; % Initial North position
P.pe0    = 0; % Initial East position
P.pd0    = 0; % Initial Down position (negative altitude)
P.u0     = P.Va0; % Initial velocity along body x-axis
P.v0     = 0; % Initial velocity along body y-axis
P.w0     = 0; % Initial velocity along body z-axis
P.phi0   = 0; % Initial roll angle
P.theta0 = 0; % Initial pitch angle
P.psi0   = 0; % Initial yaw angle
P.p0     = 0; % Initial body frame roll rate
P.q0     = 0; % Initial body frame pitch rate
P.r0     = 0; % Initial body frame yaw rate

% Run Trim Commands
[x_trim, u_trim] = compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% Set Initial Conditions to Trim Conditions
P.pn0    = 0; % Initial North position
P.pe0    = 0; % Initial East position
P.pd0    = -h0; % Initial Down position (negative altitude)
P.u0     = x_trim(4); % initial velocity along body x-axis
P.v0     = x_trim(5); % initial velocity along body y-axis
P.w0     = x_trim(6); % initial velocity along body z-axis
P.phi0   = x_trim(7); % initial roll angle
P.theta0 = x_trim(8); % initial pitch angle
P.psi0   = x_trim(9); % initial yaw angle
P.p0     = x_trim(10); % initial body frame roll rate
P.q0     = x_trim(11); % initial body frame pitch rate
P.r0     = x_trim(12); % initial body frame yaw rate

%% Compute Transfer Functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

%% Compute Controls Variables
P.beta_c = 0; % Hold commanded sideslip at 0 (radians)
P.climb_out_throttle = 1; % Take-off and climb out throttle
P.altitude_take_off_zone = 10; % Take-off zone altitude cut-off
P.altitude_hold_zone = 10; % Altitude hold cut-off
P.phi_max = 60*pi/180; % Maximum roll
P.delta_a_max = 45*pi/180; % Maximum aileron deflection
P.theta_max = 30*pi/180; % Maximum pitch
P.delta_e_max = 45*pi/180; % Maximum elevator deflection
P.beta_max = 3; % Maximum sideslip
P.delta_r_max = 20*pi/180; % Maximum rudder deflection
P.throttle_max = 1; % Maximum throttle command

compute_control_gains; % Compute Control Gains

%% Clear Clutter of Variables, except for P
clearvars -except P
%% Close 'mavsim_trim.slx'
close_system('mavsim_trim.slx',0)
% Compute all controller gains

%% Gains for roll loop
% Get transfer function data for delta_a to phi
[num,den]=tfdata(T_phi_delta_a,'v');
a_phi2 = num(3);
a_phi1 = den(2);

% Pick damping ratio
zeta_roll = 1.5; %%0.2

% Compute control gains based on zeta and wn
wn_roll=sqrt(P.delta_a_max/P.phi_max*abs(a_phi2));
e_phi_max = P.phi_max; %design parameter
P.roll_kp = P.delta_a_max/e_phi_max*sign(a_phi2);
P.roll_kd = (2*zeta_roll*wn_roll - a_phi1)/a_phi2;
P.roll_ki = 0;

%% Gains for course loop
% Pick damping ratio
zeta_course = 1.5; %0.9

% Compute control gains based on zeta and wn
wn_course = wn_roll/15.5; %%1.2
P.course_kp = 2*zeta_course*wn_course*P.Va0/P.gravity;
P.course_ki = wn_course^2*P.Va0/P.gravity;

%% Gains for sideslip hold
% Get transfer function data for delta_r to vr
[num2,den2]=tfdata(T_v_delta_r,'v');
a_beta2 = num2(2);
a_beta1 = den2(2);
% Pick damping ratio
zeta_beta = 0.7; 

e_beta_max = P.beta_max;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute control gains based on zeta and wn
P.beta_kp = P.delta_r_max/e_beta_max*sign(a_beta2);
wn_beta = (a_beta1+a_beta2*P.beta_kp)/2/zeta_beta;
P.beta_ki = wn_beta^2/a_beta2; %%%%%%% 0

%% Gains for the pitch loop
% Get transfer function data for delta_e to theta
[num1,den1]=tfdata(T_theta_delta_e,'v');
a_theta1 = den1(2);
a_theta2 = den1(3);
a_theta3 = num1(3);
% Pick damping ratio
zeta_pitch = 0.7; %0.707
e_pitch_max =P.theta_max;% P.theta_max;%%1 * pi / 180; %

% Compute control gains based on zeta and wn
wn_pitch=sqrt(a_theta2+P.delta_e_max/e_pitch_max*abs(a_theta3));
P.pitch_kp = P.delta_e_max/e_pitch_max*sign(a_theta3);
P.pitch_kd = (2*zeta_pitch*wn_pitch-a_theta1)/a_theta3;
P.K_theta_DC = P.pitch_kp*a_theta3/(a_theta2+P.pitch_kp*a_theta3);

%% Gains for altitude hold using pitch
% Pick damping ratio
zeta_altitude = 0.7; %0.9

% Compute control gains based on zeta and wn
wn_altitude = wn_pitch/10; %10,40 but between 5 and 15
P.altitude_kp = 2*zeta_altitude*wn_altitude/P.K_theta_DC/P.Va0;
P.altitude_ki = wn_altitude^2/P.K_theta_DC/P.Va0;

%% Gains for airspeed hold using pitch
% Get transfer function data for theta to airspeed
[num3,den3]=tfdata(T_Va_theta,'v');
a_V1 = den3(2);
% Pick damping ratio
zeta_airspeed_pitch = 1.5;%1.5 0.7

% Compute control gains based on zeta and wn
wn_airspeed_pitch = wn_pitch/10; %10
P.airspeed_pitch_kp = (a_V1-2*zeta_airspeed_pitch*wn_airspeed_pitch)/P.K_theta_DC/P.gravity;
P.airspeed_pitch_ki = -wn_airspeed_pitch^2/P.K_theta_DC/P.gravity;

%% Gains for airspeed hold using throttle
% Get transfer function data for throttle to airspeed
[num4,den4]=tfdata(T_Va_delta_t,'v');
a_Vt1 = den4(2);
a_Vt2 = num4(2);

%NOPE
P.a_Vt22=a_Vt2;
P.a_Vt11=a_Vt1;
%NOPE

% Pick damping ratio and natural frequency
zeta_airspeed_throttle = 2;%0.707; %2  5.5
wn_airspeed_throttle = 3;%4; %%% 3 guessed 0.35

% Compute control gains based on zeta and wn
P.airspeed_throttle_kp = (2*zeta_airspeed_throttle*wn_airspeed_throttle-a_Vt1)/a_Vt2;
%0.2552;

P.airspeed_throttle_ki = wn_airspeed_throttle^2/a_Vt2;
%0.4572;
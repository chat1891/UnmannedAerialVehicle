%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
% Physical parameters of airframe
P.mass = 13.5; % kg
P.Jx   = 0.8244; % kg-m2
P.Jy   = 1.135; % kg-m2
P.Jz   = 1.759; % kg-m2
P.Jxz  = 0.1204; % kg-m2
P.gravity = 9.81;
% Aircraft aerodynamics
P.S_wing        = 0.55; % m2
P.b             = 2.8956; % m
P.c             = 0.18994; % m
P.S_prop        = 0.2027; % m2
P.rho           = 1.2682; % kg/m3
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712; % rad

% Aerodynamic coefficients
P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%trim condition 
Va_trim = 17;
R=Inf;
gamma = 0;
R = Inf; 
% calculation 
gamma0 = P.Jx*P.Jz-P.Jxz^2;
gamma3 = P.Jz/gamma0;
gamma4 = P.Jxz/gamma0;

C_p_p = gamma3*P.C_ell_p+gamma4*P.C_n_p;
C_p_delta_a = gamma3*P.C_ell_delta_a+gamma4*P.C_n_delta_a;

%Va_trim = sqrt(x_trim(4)^2+x_trim(5)^2+x_trim(6)^2);
%theta_trim = x_trim(8);
alpha_trim = atan(4.4875/16.3970);

a_phi1 = -0.5*P.rho*Va_trim^2*P.S_wing*P.b*C_p_p*P.b/(2*Va_trim);
a_phi2 = 0.5*P.rho*Va_trim^2*P.S_wing*P.b*C_p_delta_a;

a_theta1 = -P.rho*Va_trim^2*P.c*P.S_wing/(2*P.Jy)*P.C_m_q*P.c/(2*Va_trim);
a_theta2 = -P.rho*Va_trim^2*P.c*P.S_wing/(2*P.Jy)*P.C_m_alpha;
a_theta3 = P.rho*Va_trim^2*P.c*P.S_wing/(2*P.Jy)*P.C_m_delta_e;

delta_e_trim = -14.31;
delta_t_trim = 0.2344;
a_V1 = P.rho*Va_trim*P.S_wing/P.mass*(P.C_D_0+P.C_D_alpha*alpha_trim+P.C_D_delta_e*delta_e_trim)+P.rho*P.S_prop/P.mass*P.C_prop*Va_trim;
a_V2 = P.rho*P.S_prop/P.mass*P.C_prop*P.k_motor^2*delta_t_trim;
a_V3 = P.gravity;%%%%%%%%%different from book p94

%with reasonable assumption beta is small -> cb=cos(beta)=1
cb=1;
a_beta1 = -P.rho*Va_trim*P.S_wing/(2*P.mass*cb)*P.C_Y_beta;
a_beta2 = P.rho*Va_trim*P.S_wing/(2*P.mass*cb)*P.C_Y_delta_r;

% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2])
T_h_theta       = tf([Va_trim],[1,0])
%T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1])
% T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

eig_roll = P.rho*Va_trim*P.S_wing*P.b^2*C_p_p/4
%sol = T_theta_delta_e*T_h_theta
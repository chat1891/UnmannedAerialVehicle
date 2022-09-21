function plotmavstatevariables(uu)
%
% modified 12/11/2009 - RB

    % process inputs to function
    pn          = uu(1);             % North position (m)
    pe          = uu(2);             % East position (m)
    h           = -uu(3);            % altitude (m)
    u           = uu(4);             % body velocity along x-axis (m/s)
    v           = uu(5);             % body velocity along y-axis (m/s)
    w           = uu(6);             % body velocity along z-axis (m/s)
    phi         = 180/pi*uu(7);      % roll angle (deg)   
    theta       = 180/pi*uu(8);      % pitch angle (deg)
    psi         = uu(9);             % yaw angle (deg)
    p           = 180/pi*uu(10);     % body angular rate along x-axis (deg/s)
    q           = 180/pi*uu(11);     % body angular rate along y-axis (deg/s)
    r           = 180/pi*uu(12);     % body angular rate along z-axis (deg/s)
    Va          = uu(13);            % airspeed (m/s)
    alpha       = 180/pi*uu(14);     % angle of attack (deg)
    beta        = 180/pi*uu(15);     % side slip angle (deg)
    wn          = uu(16);            % wind in the North direction
    we          = uu(17);            % wind in the East direction
    wd          = uu(18);            % wind in the Down direction
    pn_c        = uu(19);            % commanded North position (m)
    pe_c        = uu(20);            % commanded East position (m)
    h_c         = uu(21);            % commanded altitude (m)
    Va_c        = uu(22);            % commanded airspeed (m/s)
    chi_c       = 180/pi*uu(23);     % commanded course (deg)
    alpha_c     = 180/pi*uu(24);     % commanded angle of attack (deg)
    beta_c      = 180/pi*uu(25);     % commanded side slip angle (deg)
    phi_c       = 180/pi*uu(26);     % commanded roll angle (deg)   
    theta_c     = 180/pi*uu(27);     % commanded pitch angle (deg)
    p_c         = 180/pi*uu(28);     % commanded body angular rate along x-axis (deg/s)
    q_c         = 180/pi*uu(29);     % commanded body angular rate along y-axis (deg/s)
    r_c         = 180/pi*uu(30);     % commanded body angular rate along z-axis (deg/s)
    pn_hat      = uu(31);            % estimated North position (meters)
    pe_hat      = uu(32);            % estimated East position (meters)
    h_hat       = uu(33);            % estimated altitude (meters)
    Va_hat      = uu(34);            % estimated airspeed (meters/s)
    alpha_hat   = 180/pi*uu(35);     % estimated angle of attack (deg)
    beta_hat    = 180/pi*uu(36);     % estimated side slip angle (deg)
    phi_hat     = 180/pi*uu(37);     % estimated roll angle (deg)   
    theta_hat   = 180/pi*uu(38);     % estimated pitch angle (deg)
    chi_hat     = 180/pi*uu(39);     % estimated course (deg)
    p_hat       = 180/pi*uu(40);     % estimated body angular rate along x-axis (deg/s)
    q_hat       = 180/pi*uu(41);     % estimated body angular rate along y-axis (deg/s)
    r_hat       = 180/pi*uu(42);     % estimated body angular rate along z-axis (deg/s)
%    Vg_hat      = uu(43);            % estimated groundspeed
%    wn_hat      = uu(44);            % estimated North wind
%    we_hat      = uu(45);            % estimated East wind
%    psi_hat     = 180/pi*uu(46);     % estimated heading
%    bx_hat      = uu(47);            % estimated x-gyro bias
%    by_hat      = uu(48);            % estimated y-gyro bias
%    bz_hat      = uu(49);            % estimated z-gyro bias
    delta_e     = 180/pi*uu(50);     % elevator angle (degrees)
    delta_a     = 180/pi*uu(51);     % aileron angle (degrees)
    delta_r     = 180/pi*uu(52);     % rudder angle (degrees)
    delta_t     = uu(53);            % throttle setting (unitless)
    t           = uu(54);            % simulation time
    
    % compute course angle
    chi = 180/pi*atan2(Va*sin(psi)+we, Va*cos(psi)+wn);

    % define persistent variables 
    persistent pn_handle
    persistent pe_handle
    persistent h_handle
    persistent Va_handle
    persistent alpha_handle
    persistent beta_handle
    persistent phi_handle
    persistent theta_handle
    persistent chi_handle
    persistent p_handle
    persistent q_handle
    persistent r_handle
    persistent delta_e_handle
    persistent delta_a_handle
    persistent delta_r_handle
    persistent delta_t_handle
    

  % first time function is called, initialize plot and persistent vars
    if t==0
        figure(2), clf

        subplot(6,2,1)
        hold on
        pn_handle = graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', []);
        

        subplot(6,2,2)
        hold on
        pe_handle = graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e', []);


        subplot(6,2,3)
        hold on
        h_handle = graph_y_yhat_yd(t, h, h_hat, h_c, 'h', []);

%         subplot(8,2,6)
%         hold on
%         beta_handle = graph_y_yhat_yd(t, beta, beta_hat, beta_c, '\beta', []);
        
        subplot(6,2,4)
        hold on
        alpha_handle = graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha', []);
        
        subplot(6,2,5)
        hold on
        phi_handle = graph_y_yhat_yd(t, phi, phi_hat, phi_c, '\phi', []);
        
%         subplot(8,2,8)
%         hold on
%         p_handle = graph_y_yhat_yd(t, p, p_hat, p_c, 'p', []);
        
        subplot(6,2,6)
        hold on
        theta_handle = graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', []);
        
%         subplot(8,2,10)
%         hold on
%         q_handle = graph_y_yhat_yd(t, q, q_hat, q_c, 'q', []);
%         
%         subplot(8,2,11)
%         hold on
%         chi_handle = graph_y_yhat_yd(t, chi, chi_hat, chi_c, '\chi', []);
        
%         subplot(11,1,12)
%         hold on
%         r_handle = graph_y_yhat_yd(t, r, r_hat, r_c, 'r', []);
        
        subplot(6,2,7)
        hold on
        delta_e_handle = graph_y(t, delta_e, [], 'b');
        ylabel('\delta_e')
        
        subplot(6,2,8)
        hold on
        delta_a_handle = graph_y(t, delta_a, [], 'b');
        ylabel('\delta_a')

        subplot(6,2,9)
        hold on
        delta_r_handle = graph_y(t, delta_r, [], 'b');
        ylabel('\delta_r')
        
        subplot(6,2,10)
        hold on
        delta_t_handle = graph_y(t, delta_t, [], 'b');
        ylabel('\delta_t')
        
        subplot(6,2,11)
        hold on
        Va_handle = graph_y_yhat_yd(t, Va, Va_hat, Va_c, 'V_a', []);
        %ylabel('v_a')
        
    % at every other time step, redraw state variables
    else
        graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', pn_handle);
        graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e', pe_handle);
        graph_y_yhat_yd(t, h, h_hat, h_c, 'h', h_handle);
        graph_y_yhat_yd(t, Va, Va_hat, Va_c, 'V_a', Va_handle);
        graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha', alpha_handle);
        %graph_y_yhat_yd(t, beta, beta_hat, beta_c, '\beta', beta_handle);
        graph_y_yhat_yd(t, phi, phi_hat, phi_c, '\phi', phi_handle);
        graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', theta_handle);
%         graph_y_yhat_yd(t, chi, chi_hat, chi_c, '\chi', chi_handle);
%         graph_y_yhat_yd(t, p, p_hat, p_c, 'p', p_handle);
%         graph_y_yhat_yd(t, q, q_hat, q_c, 'q', q_handle);
        %graph_y_yhat_yd(t, r, r_hat, r_c, 'r', r_handle);
        graph_y(t, delta_e, delta_e_handle);
        graph_y(t, delta_a, delta_a_handle);
        graph_y(t, delta_r, delta_r_handle);
        graph_y(t, delta_t, delta_t_handle);
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, handle, color)
  
  if isempty(handle)
      handle    = plot(t,y,color);
  else
      set(handle,'Xdata',[get(handle,'Xdata'),t]);
      set(handle,'Ydata',[get(handle,'Ydata'),y]);
      %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle)
      handle(1)    = plot(t,y,'b');
      handle(2)    = plot(t,yd,'b');
      ylabel(lab)
      set(get(gca, 'YLabel'),'Rotation',0.0);
  else
      set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
      set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
      set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
      set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
      %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
  
  if isempty(handle)
      handle(1)   = plot(t,y,'b');
      handle(2)   = plot(t,yhat,'b');
      handle(3)   = plot(t,yd,'r-.');
      ylabel(lab)
      set(get(gca,'YLabel'),'Rotation',0.0);
  else
      set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
      set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
      set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
      set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
      set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
      set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
      %drawnow
  end

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

  if in < low
      out = low;
  elseif in > high
      out = high;
  else
      out = in;
  end

% end sat  
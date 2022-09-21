
%plot
font_size = 15;
line_size = 6;
line_width = 1;

%%
figure (1) %gyro

subplot(3,1,1);
plot(gyro_x.time,gyro_x.signals(1).values,'g','Linewidth',line_width) %gyro_x
hold on
plot(gyro_x.time,gyro_x.signals(2).values,'b') %p
title('Angular velocity-x vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('Angular velocity-x','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size);
legend('measured','actual');
grid on
hold off

subplot(3,1,2);
plot(gyro_y.time,gyro_y.signals(1).values,'g','Linewidth',line_width) %gyro_y
hold on
plot(gyro_y.time,gyro_y.signals(2).values,'b') %q
title('Angular velocity-y vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('Angular velocity-y','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size);
legend('measured','actual');
grid on
hold off

subplot(3,1,3);
plot(gyro_z.time,gyro_z.signals(1).values,'g','Linewidth',line_width) %gyro_z
hold on
plot(gyro_z.time,gyro_z.signals(2).values,'b') %r
title('Angular velocity-z vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('Angular velocity-z','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size);
legend('measured','actual');
grid on
hold off


%%
figure (2) %gps

subplot(3,1,1);
plot(pn.time,pn.signals(1).values,'g','Linewidth',line_width) %gyro_x
hold on
plot(pn.time,pn.signals(2).values,'b') %p
title('GPS-pn vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('GPS-pn','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size);
legend('measured','actual');
grid on
hold off

subplot(3,1,2);
plot(pe.time,pe.signals(1).values,'g','Linewidth',line_width) %gyro_x
hold on
plot(pe.time,pe.signals(2).values,'b') %p
title('GPS-pe vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('GPS-pe','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size);
legend('measured','actual');
grid on
hold off

subplot(3,1,3);
plot(h.time,h.signals(1).values,'g','Linewidth',line_width) %gyro_x
hold on
plot(h.time,h.signals(2).values,'b') %p
hold on
plot(h.time,h.signals(3).values,'r')
title('GPS-h vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('GPS-h','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size);
legend('measured','actual','altitude maneuver','Location','SouthEast');
grid on
hold off

%%
figure (3) %accel

subplot(3,1,1);
plot(accel_x.time,accel_x.signals(1).values,'g','Linewidth',line_width) %gyro_x
hold on
plot(accel_x.time,accel_x.signals(2).values,'b') %p
title('accel_x vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('accel_x','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size);
legend('measured','actual');
grid on
hold off

subplot(3,1,2);
plot(accel_y.time,accel_y.signals(1).values,'g','Linewidth',line_width) %gyro_x
hold on
plot(accel_y.time,accel_y.signals(2).values,'b') %p
title('accel_y vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('accel_y','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size);
legend('measured','actual');
grid on
hold off

subplot(3,1,3);
plot(accel_z.time,accel_z.signals(1).values,'g','Linewidth',line_width) %gyro_x
hold on
plot(accel_z.time,accel_z.signals(2).values,'b') %p
title('accel_z vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('accel_z','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size);
legend('measured','actual');
grid on
hold off

%%
%course 
figure (4) 
plot(course.time,course.signals(1).values,'g','Linewidth',line_width) %gyro_x
hold on
plot(course.time,course.signals(2).values,'b') %p
title('course(deg) vs. Time','fontsize',font_size,'Interpreter','latex');
xlabel('Time (s)','fontsize',font_size,'Interpreter','latex');
ylabel('course(deg)','fontsize',font_size,'Interpreter','latex');
set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size);
legend('measured','course maneuver');
grid on
hold off


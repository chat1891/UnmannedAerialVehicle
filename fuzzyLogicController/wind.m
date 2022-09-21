w20=6;
h=1.64:0.1:328.084;
u=w20*log(h/0.15)/log(20/0.15);
figure (5)
 plot(h*0.3048,u)
 xlabel('h (m/s)');
 ylabel('mean wind speed u (m/s)')
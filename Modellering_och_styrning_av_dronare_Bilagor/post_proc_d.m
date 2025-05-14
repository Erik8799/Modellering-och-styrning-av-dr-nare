close all; clear all; clc;

DAT = readmatrix('PID_tuning_components-moment2-20250425T16-38-34.csv','NumHeaderLines',1,'Delimiter',',');

deg2rad = pi/180;

rad2deg = 180/pi;

Ts = 0.01;
p = DAT(:,2)*deg2rad;
q =-DAT(:,3)*deg2rad;
r =-DAT(:,4)*deg2rad;

pwm1 = DAT(:,5);
pwm3 = DAT(:,6);

t = (0:length(p)-1)'*Ts;

figure;
hold on; grid on;
plot(t,p*rad2deg,'r-');
plot(t,q*rad2deg,'g-');
plot(t,r*rad2deg,'b-');
plot(t,0.5*pwm1,'c-');
plot(t,0.5*pwm3,'m-');
title('Data')

idxLo = find(t > 2.4); idxLo = idxLo(1)-1;
idxHi = find(t > 3.7); idxHi = idxHi(1)-1;
tt = t(idxLo:idxHi);
p = p(idxLo:idxHi);
q = q(idxLo:idxHi);
r = r(idxLo:idxHi);

rr = [r(1);r];

rDot = diff(rr)/Ts;

figure;
hold on; grid on;
plot(tt,p*rad2deg,'r-');
plot(tt,q*rad2deg,'g-');
plot(tt,r*rad2deg,'b-');
legend('p','q','r')
title('Vinkelhastighet')
ylabel('deg/s')

figure;
hold on; grid on;
plot(tt,rDot*rad2deg,'r-');
title('Vinkelacceleration')

Jx = 1.5963e-5;
Jy = 1.5914e-5;
Jz = 3.0583e-5;

w1 = 2*pi/60*18515; %propeller speed in rad/s

idxLo = find(tt > 2.5); idxLo = idxLo(1)-1;
idxHi = find(tt > 3.5); idxHi = idxHi(1)-1;

d = (Jz*rDot - (Jx-Jy)*p.*q)/(2*w1^2);
dmean = mean(d(idxLo:idxHi));
figure;
hold on; grid on;
plot(tt,d,'g-');
plot([tt(1) tt(end)],dmean*[1 1],'r-');
title(['$d=' num2str(dmean*1e10,'%1.2f') '\times 10^{-10}\textrm{ Nm/s}^2$'],'Interpreter','latex');


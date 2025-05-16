clc;
clear all;
close all;

rad2deg = 180/pi;
deg2rad = pi/180;

l = 6*(9.221e-2)/2; %times 6 so it's easier to see

[t,X] = ode45(@f,[0 12],[0 0 0 0*deg2rad 0*deg2rad 0*deg2rad 0 0 0 0 0 0 0 0 0 0]);

t_uniform = linspace(t(1), t(end), 100);
X_uniform = interp1(t, X, t_uniform);

fontSize = 12;
O = [0;0;0];

myVideo = VideoWriter('drone_lqr_ap_ani','MPEG-4'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo);

fh = figure('Units', 'normalized', 'OuterPosition', [0 0 1 1]);
tiledlayout(1,1,'Tilespacing','Compact','Padding','Compact');
nexttile;


pos.x = X_uniform(:,1);
pos.y = X_uniform(:,2);
pos.z = X_uniform(:,3);
pos.phi = X_uniform(:,4);
pos.theta = X_uniform(:,5);
pos.psi = X_uniform(:,6);

%animation for loop
for n = 1 : length(t_uniform)
if t_uniform(n) < 0.8
    Ref = [0;1;-1;0*deg2rad];
elseif t_uniform(n) >= 0.8 && t_uniform(n) < 3
    Ref = [1;1;-1;-90*deg2rad];
elseif t_uniform(n) >= 3 && t_uniform(n) < 6
    Ref = [1;0;-1;-180*deg2rad];
elseif t_uniform(n) >= 6 && t_uniform(n) < 8
    Ref = [0;0;-1;-180*deg2rad];
elseif t_uniform(n) >= 8 && t_uniform(n) < 8.6
    Ref = [0;0;-1;-270*deg2rad];
elseif t_uniform(n) >= 8.6 && t_uniform(n) < 10
    Ref = [0;0;-1;-360*deg2rad];
elseif t_uniform(n) >= 10
    Ref = [0;0;-0.5;-360*deg2rad];
end

XB = Rx(pos.phi(n))*Ry(pos.theta(n))*Rz(pos.psi(n))*[pos.x(n);pos.y(n);pos.z(n)];
rotor1B = XB + [ l/sqrt(2); l/sqrt(2); 0];
rotor2B = XB + [-l/sqrt(2); l/sqrt(2); 0];
rotor3B = XB + [-l/sqrt(2);-l/sqrt(2); 0];
rotor4B = XB + [ l/sqrt(2);-l/sqrt(2); 0];

r1E = transpose(Rx(pos.phi(n))*Ry(pos.theta(n))*Rz(pos.psi(n)))*rotor1B;
r2E = transpose(Rx(pos.phi(n))*Ry(pos.theta(n))*Rz(pos.psi(n)))*rotor2B;
r3E = transpose(Rx(pos.phi(n))*Ry(pos.theta(n))*Rz(pos.psi(n)))*rotor3B;
r4E = transpose(Rx(pos.phi(n))*Ry(pos.theta(n))*Rz(pos.psi(n)))*rotor4B;

posxC= Ref(1);
posyC= Ref(2);
poszC= Ref(3);

figure(fh);
plot3([O(1) O(1)],[O(2) O(2)],[O(3) O(3)],'ok'); % Origo

hold on; grid on;
plot3([pos.x( n) pos.x( n)],[pos.y( n) pos.y( n)],[pos.z( n) pos.z( n)],'ob'); % Center of mass
plot3([posxC posxC],[posyC posyC],[poszC poszC],'or'); % Reference coordinate
% Projections of reference coordinate
plot3([O(1)  posxC],[O(2)  O(2) ],[O(3) O(3)],'k--','HandleVisibility','off');
plot3([O(1)  O(1) ],[O(2)  posyC],[O(3) O(3)],'k--','HandleVisibility','off');
plot3([O(1)  posxC],[posyC posyC],[O(3) O(3)],'k--','HandleVisibility','off');
plot3([posxC posxC],[O(2)  posyC],[O(3) O(3)],'k--','HandleVisibility','off');
plot3([posxC posxC],[posyC posyC],[O(3) poszC],'k--','HandleVisibility','off');

plot3([pos.x(n) r1E(1)],[pos.y(n) r1E(2)],[pos.z(n) r1E(3)],'k-','HandleVisibility','off');
plot3([pos.x(n) r2E(1)],[pos.y(n) r2E(2)],[pos.z(n) r2E(3)],'k-','HandleVisibility','off');
plot3([pos.x(n) r3E(1)],[pos.y(n) r3E(2)],[pos.z(n) r3E(3)],'k-','HandleVisibility','off');
plot3([pos.x(n) r4E(1)],[pos.y(n) r4E(2)],[pos.z(n) r4E(3)],'k-','HandleVisibility','off');
plot3([r1E(1) r1E(1)],[r1E(2) r1E(2)],[r1E(3) r1E(3)],'og');
plot3([r2E(1) r2E(1)],[r2E(2) r2E(2)],[r2E(3) r2E(3)],'oc');
plot3([r3E(1) r3E(1)],[r3E(2) r3E(2)],[r3E(3) r3E(3)],'oy');
plot3([r4E(1) r4E(1)],[r4E(2) r4E(2)],[r4E(3) r4E(3)],'om');

set(gca,'YDir','reverse'); % Reverse y-direction for east (E)
set(gca,'ZDir','reverse'); % Reverse z-direction for down (D)

k = 1.0;
quiver3([0 0],[0 0],[0 0], [0 k],[0 0],[0 0], 'r','HandleVisibility','off');
quiver3([0 0],[0 0],[0 0], [0 0],[0 k],[0 0], 'g','HandleVisibility','off');
quiver3([0 0],[0 0],[0 0], [0 0],[0 0],[0 k], 'b','HandleVisibility','off');
text(k,0,0,'$N$','FontSize',fontSize,'interpreter','latex');
text(0,k,0,'$E$','FontSize',fontSize,'interpreter','latex');
text(0,0,k,'$D$','FontSize',fontSize,'interpreter','latex');
hold off;
az = -48; el = 34;
view(az,el);
axis equal;
hi = 2; lo = -2;
axis([lo hi lo hi lo hi]);
title(['Tid t = ' num2str(t_uniform(n),'%1.2f')]);
drawnow;

frame = getframe(gcf); %get frame
writeVideo(myVideo, frame);
end

close(myVideo);

%nonlinear equations
function dxdt = f(t,X)
rad2deg = 180/pi;
deg2rad = pi/180;

%define constants
m = 30.3e-3;
mp = 0.3e-3;
g = 9.82;
l = (9.221e-2)/2;

J = [1.5963e-5 0         0;
     0         1.5914e-5 0;
     0         0         3.0583e-5];

J_x = J(1,1);
J_y = J(2,2);
J_z = J(3,3);
J_p = mp*((4.2e-2)^2+(0.84e-2)^2)/12;

k = 1.946e-8;
d = 7.905e-10;

omega12 = m*g/(4*k);
omega22 = m*g/(4*k);
omega32 = m*g/(4*k);
omega42 = m*g/(4*k);

if t < 0.8
    Ref = [0;1;-1;0*deg2rad];
    mode = 1;
    psi = 1;
    L = getL(mode,psi);
elseif t >= 0.8 && t < 3
    Ref = [1;1;-1;-90*deg2rad];
    mode = 1;
    psi = 10;
    L = getL(mode,psi);
elseif t >= 3 && t < 6
    Ref = [1;0;-1;-180*deg2rad];
    mode = 1;
    psi = 7;
    L = getL(mode,psi);
elseif t >= 6 && t < 8
    Ref = [0;0;-1;-180*deg2rad];
    mode = 1;
    psi = 7;
    L = getL(mode,psi);
elseif t >= 8 && t < 8.6
    Ref = [0;0;-1;-270*deg2rad];
    mode = 1;
    psi = 4;
    L = getL(mode,psi);
elseif t >= 8.6 && t < 10
    Ref = [0;0;-1;-360*deg2rad];
    mode = 1;
    psi = 1;
    L = getL(mode,psi);
elseif t >= 10
    Ref = [0;0;-0.5;-360*deg2rad];
    mode = 1;
    psi = 1;
    L = getL(mode,psi);
end


U = [-L(1,1)*(X(1)-Ref(1))-L(1,2)*(X(2)-Ref(2))-L(1,3)*(X(3)-Ref(3))-L(1,4)*X(4)-L(1,5)*X(5)-L(1,6)*(X(6)-Ref(4))-L(1,7)*X(7)-L(1,8)*X(8)-L(1,9)*X(9)-L(1,10)*X(10)-L(1,11)*X(11)-L(1,12)*X(12)-L(1,13)*X(13)-L(1,14)*X(14)-L(1,15)*X(15)-L(1,16)*X(16)+omega12;
     -L(2,1)*(X(1)-Ref(1))-L(2,2)*(X(2)-Ref(2))-L(2,3)*(X(3)-Ref(3))-L(2,4)*X(4)-L(2,5)*X(5)-L(2,6)*(X(6)-Ref(4))-L(2,7)*X(7)-L(2,8)*X(8)-L(2,9)*X(9)-L(2,10)*X(10)-L(2,11)*X(11)-L(2,12)*X(12)-L(2,13)*X(13)-L(2,14)*X(14)-L(2,15)*X(15)-L(2,16)*X(16)+omega22;
     -L(3,1)*(X(1)-Ref(1))-L(3,2)*(X(2)-Ref(2))-L(3,3)*(X(3)-Ref(3))-L(3,4)*X(4)-L(3,5)*X(5)-L(3,6)*(X(6)-Ref(4))-L(3,7)*X(7)-L(3,8)*X(8)-L(3,9)*X(9)-L(3,10)*X(10)-L(3,11)*X(11)-L(3,12)*X(12)-L(3,13)*X(13)-L(3,14)*X(14)-L(3,15)*X(15)-L(3,16)*X(16)+omega32;
     -L(4,1)*(X(1)-Ref(1))-L(4,2)*(X(2)-Ref(2))-L(4,3)*(X(3)-Ref(3))-L(4,4)*X(4)-L(4,5)*X(5)-L(4,6)*(X(6)-Ref(4))-L(4,7)*X(7)-L(4,8)*X(8)-L(4,9)*X(9)-L(4,10)*X(10)-L(4,11)*X(11)-L(4,12)*X(12)-L(4,13)*X(13)-L(4,14)*X(14)-L(4,15)*X(15)-L(4,16)*X(16)+omega42];


% X = [x = X(1),y = X(2),z = X(3),phi = X(4),theta = X(5),psi = X(6),u = X(7),v = X(8),w = X(9),p = X(10),q = X(11),r = X(12)]
dxdt = [X(7); 
        X(8); 
        X(9); 
        X(10)+X(11)*sin(X(4))*tan(X(5))+X(12)*cos(X(4))*tan(X(5)); 
        X(11)*cos(X(4))-X(12)*sin(X(4)); 
        X(11)*sin(X(4))*sec(X(5))+X(12)*cos(X(4))*sec(X(5)); 
        -(k/m)*(U(1)+U(2)+U(3)+U(4))*(cos(X(4))*sin(X(5))*cos(X(6))+sin(X(4))*sin(X(6)));
        -(k/m)*(U(1)+U(2)+U(3)+U(4))*(cos(X(4))*sin(X(5))*sin(X(6))-sin(X(4))*cos(X(6))); 
        g-(k/m)*(U(1)+U(2)+U(3)+U(4))*cos(X(4))*cos(X(5)); 
        ((J_y-J_z)/J_x)*X(11)*X(12) + (1/(sqrt(2)*J_x))*l*k*(-U(1)-U(2)+U(3)+U(4)) - (J_p/J_x)*X(11)*(-sqrt(U(1))+sqrt(U(2))-sqrt(U(3))+sqrt(U(4)));
        ((J_z-J_x)/J_y)*X(10)*X(12) + (1/(sqrt(2)*J_y))*l*k*(U(1)-U(2)-U(3)+U(4)) + (J_p/J_y)*X(10)*(-sqrt(U(1))+sqrt(U(2))-sqrt(U(3))+sqrt(U(4))); 
        ((J_x-J_y)/J_z)*X(10)*X(11) + (1/J_z)*d*(U(1)-U(2)+U(3)-U(4));
        X(1)-Ref(1);
        X(2)-Ref(2);
        X(3)-Ref(3);
        X(6)-Ref(4)];
end


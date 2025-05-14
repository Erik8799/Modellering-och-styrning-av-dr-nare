clc;
clear;
syms x y z u v w phi theta psi p q r omega12 omega22 omega32 omega42 real;
syms m g J_x J_y J_z J_p l k d real;

%nonlinear equations
dx = u;
dy = v;
dz = w;
dphi = p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
dtheta = q*cos(phi)-r*sin(phi);
dpsi = q*sin(phi)*sec(theta)+r*cos(phi)*sec(theta);
du = -(k/m)*(omega12+omega22+omega32+omega42)*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
dv = -(k/m)*(omega12+omega22+omega32+omega42)*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
dw = g-(k/m)*(omega12+omega22+omega32+omega42)*cos(phi)*cos(theta);
dp = ((J_y-J_z)/J_x)*q*r + (1/(sqrt(2)*J_x))*l*k*(-omega12-omega22+omega32+omega42) - (J_p/J_x)*q*(-sqrt(omega12)+sqrt(omega22)-sqrt(omega32)+sqrt(omega42));
dq = ((J_z-J_x)/J_y)*p*r + (1/(sqrt(2)*J_y))*l*k*(omega12-omega22-omega32+omega42) + (J_p/J_y)*p*(-sqrt(omega12)+sqrt(omega22)-sqrt(omega32)+sqrt(omega42));
dr = ((J_x-J_y)/J_z)*p*q + (1/J_z)*d*(omega12-omega22+omega32-omega42);

%linear model
A = jacobian([dx,dy,dz,dphi,dtheta,dpsi,du,dv,dw,dp,dq,dr],[x,y,z,phi,theta,psi,u,v,w,p,q,r])

B = jacobian([dx,dy,dz,dphi,dtheta,dpsi,du,dv,dw,dp,dq,dr],[omega12,omega22,omega32,omega42])

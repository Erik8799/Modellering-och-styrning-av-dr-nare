function [A,B] = modell(psi,m,l,J_x,J_y,J_z,J_p,k,d)

%trim values
u = 0;
v = 0;
w = 0;
phi = 0;
theta = 0;
p = 0;
q = 0;
r = 0;

g = 9.82;

omega12 = m*g/(4*k);
omega22 = m*g/(4*k);
omega32 = m*g/(4*k);
omega42 = m*g/(4*k);



A = [0 0 0                                                                                                 0                                                                             0                                                                                                 0 1 0 0                                                                                                 0                                                                                                 0                    0;
     0 0 0                                                                                                 0                                                                             0                                                                                                 0 0 1 0                                                                                                 0                                                                                                 0                    0;
     0 0 0                                                                                                 0                                                                             0                                                                                                 0 0 0 1                                                                                                 0                                                                                                 0                    0;
     0 0 0                                                   (q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta))               (r*cos(phi)*(tan(theta)^2 + 1) + q*sin(phi)*(tan(theta)^2 + 1))                                                                                                 0 0 0 0                                                                                                 1                                                                               sin(phi)*tan(theta)  cos(phi)*tan(theta);
     0 0 0                                                                        (-r*cos(phi) - q*sin(phi))                                                                             0                                                                                                 0 0 0 0                                                                                                 0                                                                                          cos(phi)            -sin(phi);
     0 0 0                                               ((q*cos(phi))/cos(theta) - (r*sin(phi))/cos(theta)) ((r*cos(phi)*sin(theta))/cos(theta)^2 + (q*sin(phi)*sin(theta))/cos(theta)^2)                                                                                                 0 0 0 0                                                                                                 0                                                                               sin(phi)/cos(theta)  cos(phi)/cos(theta);
     0 0 0 -(k*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(omega12 + omega22 + omega32 + omega42))/m   -(k*cos(phi)*cos(psi)*cos(theta)*(omega12 + omega22 + omega32 + omega42))/m -(k*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(omega12 + omega22 + omega32 + omega42))/m 0 0 0                                                                                                 0                                                                                                 0                    0;
     0 0 0  (k*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(omega12 + omega22 + omega32 + omega42))/m   -(k*cos(phi)*cos(theta)*sin(psi)*(omega12 + omega22 + omega32 + omega42))/m -(k*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(omega12 + omega22 + omega32 + omega42))/m 0 0 0                                                                                                 0                                                                                                 0                    0;
     0 0 0                                 (k*cos(theta)*sin(phi)*(omega12 + omega22 + omega32 + omega42))/m             (k*cos(phi)*sin(theta)*(omega12 + omega22 + omega32 + omega42))/m                                                                                                 0 0 0 0                                                                                                 0                                                                                                 0                    0;
     0 0 0                                                                                                 0                                                                             0                                                                                                 0 0 0 0                                                                                                 0 ((J_p*(omega12^(1/2) - omega22^(1/2) + omega32^(1/2) - omega42^(1/2)))/J_x + (r*(J_y - J_z))/J_x)  (q*(J_y - J_z))/J_x;
     0 0 0                                                                                                 0                                                                             0                                                                                                 0 0 0 0 (-(J_p*(omega12^(1/2) - omega22^(1/2) + omega32^(1/2) - omega42^(1/2)))/J_y - (r*(J_x - J_z))/J_y)                                                                                                0 -(p*(J_x - J_z))/J_y;
     0 0 0                                                                                                 0                                                                             0                                                                                                 0 0 0 0                                                                               (q*(J_x - J_y))/J_z                                                                               (p*(J_x - J_y))/J_z                    0];
 
 
B = [                                                        0                                                         0                                                         0                                                         0;
                                                             0                                                         0                                                         0                                                         0;
                                                             0                                                         0                                                         0                                                         0;
                                                             0                                                         0                                                         0                                                         0;
                                                             0                                                         0                                                         0                                                         0;
                                                             0                                                         0                                                         0                                                         0;
     -(k*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/m -(k*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/m -(k*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/m -(k*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/m;
      (k*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/m  (k*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/m  (k*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/m  (k*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/m;
                                    -(k*cos(phi)*cos(theta))/m                                -(k*cos(phi)*cos(theta))/m                                -(k*cos(phi)*cos(theta))/m                                -(k*cos(phi)*cos(theta))/m;
       ((J_p*q)/(2*J_x*omega12^(1/2)) - (2^(1/2)*k*l)/(2*J_x))  (-(2^(1/2)*k*l)/(2*J_x) - (J_p*q)/(2*J_x*omega22^(1/2)))   ((2^(1/2)*k*l)/(2*J_x) + (J_p*q)/(2*J_x*omega32^(1/2)))   ((2^(1/2)*k*l)/(2*J_x) - (J_p*q)/(2*J_x*omega42^(1/2)));
       ((2^(1/2)*k*l)/(2*J_y) - (J_p*p)/(2*J_y*omega12^(1/2)))   ((J_p*p)/(2*J_y*omega22^(1/2)) - (2^(1/2)*k*l)/(2*J_y))  (-(2^(1/2)*k*l)/(2*J_y) - (J_p*p)/(2*J_y*omega32^(1/2)))   ((2^(1/2)*k*l)/(2*J_y) + (J_p*p)/(2*J_y*omega42^(1/2)));
                                                         d/J_z                                                    -d/J_z                                                     d/J_z                                                    -d/J_z];

end



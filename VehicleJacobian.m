function [F,B]=VehicleJacobian(Xk,P,gyroOp)
x=Xk(1);
y=Xk(2);
z=Xk(3);
u=Xk(4);
v=Xk(5);
w=Xk(6);
phi=Xk(7);
theta=Xk(8);
psi=Xk(9);
p=gyroOp(1);
q=gyroOp(2);
r=gyroOp(3);
Temp=zeros(9,9);
Temp(1:3,4:6)=rotationMatb_to_w(phi,theta,psi);
Temp(1:3,7:9)=jacR1(phi,theta,psi,u,v,w);
Temp(4:6,4:6)=jacOmegaV(p,q,r);
Temp(4:6,7:9)=P.g*jacGravityVec(phi,theta,psi);
Temp(7:9,7:9)=jacR2(phi,theta,psi,p,q,r);
F=eye(9)+P.Ts*Temp;
B=zeros(9,6);
B(4:6,1:3)=jacOmegaVB(u,v,w);
B(4:6,4:6)=eye(3);
B(7:9,1:3)=Gyro2BodyRotation(phi,theta);
B=P.Ts*B;

function F1=jacR1(phi,theta,psi,u,v,w)
F1 =[   v*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + w*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), w*cos(phi)*cos(psi)*cos(theta) - u*cos(psi)*sin(theta) + v*cos(psi)*cos(theta)*sin(phi), w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - u*cos(theta)*sin(psi)
    - v*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - w*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), w*cos(phi)*cos(theta)*sin(psi) - u*sin(psi)*sin(theta) + v*cos(theta)*sin(phi)*sin(psi), w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta)
    w*cos(theta)*sin(phi) - v*cos(phi)*cos(theta),                            u*cos(theta) + w*cos(phi)*sin(theta) + v*sin(phi)*sin(theta),                                                                                                                                   0];
F1(3,1:3)=-F1(3,1:3);
function F2=jacGravityVec(phi,theta,psi)
F2 =[                    0,          -cos(theta), 0
    cos(phi)*cos(theta), -sin(phi)*sin(theta), 0
    -cos(theta)*sin(phi), -cos(phi)*sin(theta), 0];
function F3=jacR2(phi,theta,psi,p,q,r)
F3 =[     q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta),               r*cos(phi)*(tan(theta)^2 + 1) + q*sin(phi)*(tan(theta)^2 + 1), 0
- r*cos(phi) - q*sin(phi),                                                                           0, 0
(q*cos(phi))/cos(theta) - (r*sin(phi))/cos(theta), (r*cos(phi)*sin(theta))/cos(theta)^2 + (q*sin(phi)*sin(theta))/cos(theta)^2, 0];
function F4=jacOmegaV(p,q,r)
F4 =[  0,  r, -q
 -r,  0,  p
  q, -p,  0];
function F5=jacOmegaVB(u,v,w)
F5 =[  0, -w,  v
  w,  0, -u
 -v,  u,  0];

function  xk1=QuadRootor12StateDynamics(x,P,in)


% pwm_f = in(1);
% pwm_r = in(2);
% pwm_b = in(3);
% pwm_l = in(4);
% 
% FORCES = P.M*[pwm_f; pwm_r; pwm_b; pwm_l];

F_thrust = P.m*in(1);
tauPhi = in(2);
tauTheta = in(3);
tauPsi = in(4);

% % % pn = x(1);
% % % pe = x(2);
% % % pd = x(3);
u = x(4);
v = x(5);
w = x(6);
phi = x(7); 
theta = x(8);
psi = x(9);
p = x(10);
q = x(11);
r = x(12);
fx=zeros(12,1);
fx(1:3,1)=rotationMatb_to_w(phi,theta,psi)*[u;v;w]; %Rotates [u;v;w]
fx(4:6,1)=-(cross([p;q;r],[u;v;w]))+P.g*[-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)]+(1/P.m) * [0;0;-F_thrust];
fx(7:9,1)=Gyro2BodyRotation(phi,theta)*[p;q;r]; %Rotates [p;q;r]





fx(10:12,1) = [   ( ((P.Jy - P.Jz)/P.Jx) * q * r );...
                ( ((P.Jz - P.Jx)/P.Jy) * p * r );...
                ( ((P.Jx - P.Jy)/P.Jz) * p * q )	] + [ ((1/P.Jx) * tauPhi);...
                                                          ((1/P.Jy) * tauTheta);...
                                                          ((1/P.Jz) * tauPsi) ];
            



xdot = fx;
xk1=x+P.Ts*xdot;

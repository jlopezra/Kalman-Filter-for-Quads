function [accelOp]=Accelerometer(Xk,P,i,in)
x=Xk(1);
y=Xk(2);
z=Xk(3);
u=Xk(4);
v=Xk(5);
w=Xk(6);
phi=Xk(1);
theta=Xk(1);
psi=Xk(1);
F_thrust = P.m*in(1);
tauPhi = in(2);
tauTheta = in(3);
tauPsi = in(4);
%[gyroOp]=Gyro(Xk);
accelOp=(1/P.m) * [0;...
                   0;...
                  -F_thrust];

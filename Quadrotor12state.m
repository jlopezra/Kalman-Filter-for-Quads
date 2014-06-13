function [Xtrue,iw,gyroOpwn,accelOpwn,in]=Quadrotor12state(P,Xtrue,iw,i,qcount)
%% Initialization
%% Input
% iw=waypoint number
% i= iteration
% Xtrue is 12 state quadrotor
%% Output
%gyroOpwn= gyro o/p with noise
%accelOpwn= accelreometer o/p with noise
%% main block
Xw=[P.Xw(iw,qcount);P.Yw(iw,qcount);P.Zw(iw,qcount)];% current waypoint selction
[gyroOp]=Gyro(Xtrue,i,P); 
in=quadrotorControl(Xtrue,P,Xw,gyroOp);
[accelOp]=Accelerometer(Xtrue,P,i,in);
Xtrue=QuadRootor12StateDynamics(Xtrue,P,in);
[accelOpwn,gyroOpwn]=AddProcessNoise(accelOp,gyroOp,P);
iw=WaypointSelection(Xtrue,Xw,iw,P);
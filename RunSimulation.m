function RunSimulation(P)
%% Initialization
pos=[0;0;100];
vel=[2;0;0];
attitude=[0;0;0];
Xtrue=[...
    pos;...
    vel;...
    attitude];
% initial filter state
Xfilter=Xtrue+[...
    randn(1)*P.sx;...
    randn(1)*P.sy;...
    randn(1)*P.sz;...
    randn(1)*P.svx;...
    randn(1)*P.svy;...
    randn(1)*P.svz;...
    randn(1)*P.sphi;...
    randn(1)*P.stheta;...
    randn(1)*P.spsi];
% initial covariance
Pfilter=zeros(9);

Pfilter(1,1)=P.sx^2;
Pfilter(2,2)=P.sy^2;
Pfilter(3,3)=P.sz^2;
Pfilter(4,4)=P.svx^2;
Pfilter(5,5)=P.svy^2;
Pfilter(6,6)=P.svz^2;
Pfilter(7,7)=P.sphi^2;
Pfilter(8,8)=P.stheta^2;
Pfilter(9,9)=P.spsi^2;
% Process noise
Q1=zeros(6);
Q1(1,1)=P.sax^2;
Q1(2,2)=P.say^2;
Q1(3,3)=P.saz^2;
Q1(4,4)=P.sp^2;
Q1(5,5)=P.sq^2;
Q1(6,6)=P.sr^2;



%% Number of iterations
iter=10000;
Xtrue_plot=zeros(9,iter);
T=zeros(1,iter);
Xfilter_plot=zeros(9,iter);
Pfilter_plot=zeros(9,iter);
%% Main Loop
for i=1:iter
    T(i)=(i-1)*P.Ts;
    Xtrue_plot(:,i)=Xtrue;
    Xfilter_plot(:,i)=Xfilter;
    Pfilter_plot(:,i)=3*diag(Pfilter).^0.5;
    [gyroOp]=Gyro(Xtrue,i,P);
    [accelOp]=Accelerometer(Xtrue,P,i);
    [accelOpwn,gyroOpwn]=AddProcessNoise(accelOp,gyroOp,P);
    Xtrue=QuadRootor9StateDynamics(Xtrue,P,accelOp,gyroOp);
    Z=GeneralSensor(Xtrue,P);
%     % Prediction Step
     X0=Xfilter;
    [Xfilter,Pfilter]=Predict(Xfilter,Pfilter,Q1,P,gyroOpwn,accelOpwn);
    [Yt,yt]=TotalMeasurementInfo(Z,Xfilter,X0,P);
    % measurement update
%     [I,info]=informationGainbyMeasurement(Z,Xfilter,P,X0);
    Yfilter=Pfilter^(-1);
    yfilter=Yfilter*Xfilter;
    Y=Yfilter+Yt;
    y=yfilter+yt;
    Pfilter=Y^(-1);
    Xfilter=Pfilter*y;
end
%% Plotting Data
% true pos data
xtrue=Xtrue_plot(1,:);
ytrue=Xtrue_plot(2,:);
ztrue=Xtrue_plot(3,:);
% true vel data
vxtrue=Xtrue_plot(4,:);
vytrue=Xtrue_plot(5,:);
vztrue=Xtrue_plot(6,:);
% true att data
phitrue=Xtrue_plot(7,:)*180/pi;
thetatrue=Xtrue_plot(8,:)*180/pi;
psitrue=Xtrue_plot(9,:)*180/pi;
% filter data
xfilter=Xfilter_plot(1,:);
yfilter=Xfilter_plot(2,:);
zfilter=Xfilter_plot(3,:);
% filter vel data
vxfilter=Xfilter_plot(4,:);
vyfilter=Xfilter_plot(5,:);
vzfilter=Xfilter_plot(6,:);
% filter att data
phifilter=Xfilter_plot(7,:)*180/pi;
thetafilter=Xfilter_plot(8,:)*180/pi;
psifilter=Xfilter_plot(9,:)*180/pi;
% error bounds
% filter data
Px=Pfilter_plot(1,:);
Py=Pfilter_plot(2,:);
Pz=Pfilter_plot(3,:);
% filter vel data
Pvx=Pfilter_plot(4,:);
Pvy=Pfilter_plot(5,:);
Pvz=Pfilter_plot(6,:);
% filter att data
Pphi=Pfilter_plot(7,:)*180/pi;
Ptheta=Pfilter_plot(8,:)*180/pi;
Ppsi=Pfilter_plot(9,:)*180/pi;
%% Actual Plots
figure(1)% trajectory
plot3(xtrue,ytrue,-ztrue,xfilter,yfilter,-zfilter)
title(' 3-D Trajectory ');
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
legend('True');
axis([-100 100 -100 100 -100 100])
figure(2)% velocity
plot(T,vxtrue,T,vytrue,T,vztrue,T,vxfilter,T,vyfilter,T,vzfilter)
title('Velocity in Body Axis');
xlabel('Time(s)');
ylabel('Vel (m/s)');
legend('u','v','w');
figure(3)% attitude
plot(T,phitrue,T,thetatrue,T,psitrue,T,phifilter,T,thetafilter,T,psifilter)
title('Attitude of Quadrotor');
xlabel('Time(s)');
ylabel('Angle (rad)');
legend('\phi','\theta','\psi');
%% Error Plots
figure(4)%
subplot(3,3,1)
plot(T,xtrue-xfilter,T,Px,T,-Px)
ylabel('e_x(m)')
subplot(3,3,2)
plot(T,ytrue-yfilter,T,Py,T,-Py)
ylabel('e_y(m)')
subplot(3,3,3)
plot(T,ztrue-zfilter,T,Pz,T,-Pz)
ylabel('e_z(m)')
subplot(3,3,4)
plot(T,vxtrue-vxfilter,T,Pvx,T,-Pvx)
ylabel('e_{vx}(m)')
subplot(3,3,5)
plot(T,vytrue-vyfilter,T,Pvy,T,-Pvy)
ylabel('e_{vy}(m)')
subplot(3,3,6)
plot(T,vztrue-vzfilter,T,Pvz,T,-Pvz)
ylabel('e_{vz}(m)')
subplot(3,3,7)
plot(T,phitrue-phifilter,T,Pphi,T,-Pphi)
ylabel('e_{\phi}(m)')
xlabel('time(s)')
subplot(3,3,8)
plot(T,thetatrue-thetafilter,T,Ptheta,T,-Ptheta)
ylabel('e_{\theta}(m)')
xlabel('time(s)')
subplot(3,3,9)
plot(T,psitrue-psifilter,T,Ppsi,T,-Ppsi)
ylabel('e_{\psi}(m)')
xlabel('time(s)')




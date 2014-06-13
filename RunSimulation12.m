function RunSimulation12(P)
%% Initialization
Xtrue=zeros(12,P.Nq);
Xfilter=zeros(9,P.Nq);
Pfilter=zeros(9,9,P.Nq);
in=zeros(4,P.Nq);
for qcount=1:P.Nq
    Xtrue(:,qcount)=[...
        P.xn(qcount);...
        P.ye(qcount);...
        P.zd(qcount);...
        P.u(qcount);...
        P.v(qcount);...
        P.w(qcount);...
        P.phi(qcount);...
        P.theta(qcount);...
        P.psi(qcount);...
        P.p(qcount);...
        P.q(qcount);...
        P.r(qcount)];
    Xfilter(:,qcount)=Xtrue(1:9,qcount)+[...
        randn(1)*P.sx;...
        randn(1)*P.sy;...
        randn(1)*P.sz;...
        randn(1)*P.svx;...
        randn(1)*P.svy;...
        randn(1)*P.svz;...
        randn(1)*P.sphi;...
        randn(1)*P.stheta;...
        randn(1)*P.spsi];
    
    Ptemp=zeros(9);
    Ptemp(1,1)=P.sx^2;
    Ptemp(2,2)=P.sy^2;
    Ptemp(3,3)=P.sz^2;
    Ptemp(4,4)=P.svx^2;
    Ptemp(5,5)=P.svy^2;
    Ptemp(6,6)=P.svz^2;
    Ptemp(7,7)=P.sphi^2;
    Ptemp(8,8)=P.stheta^2;
    Ptemp(9,9)=P.spsi^2;
    Pfilter(:,:,qcount)=Ptemp;
    
    
    in(:,qcount)=[P.g;0;0;0];% Force and other three torques
    
end



% Process noise
Q1=zeros(6);
Q1(1,1)=P.sax^2;
Q1(2,2)=P.say^2;
Q1(3,3)=P.saz^2;
Q1(4,4)=P.sp^2;
Q1(5,5)=P.sq^2;
Q1(6,6)=P.sr^2;



%% Number of iterations
iter=20000;
Xtrue_plot=zeros(12,iter,P.Nq);
T=zeros(1,iter);
Xfilter_plot=zeros(9,iter,P.Nq);
Pfilter_plot=zeros(9,iter,P.Nq);
Plot_input=zeros(4,iter,P.Nq);
iw=ones(1,P.Nq);
% Force and other three torques
%% Main Loop

for i=1:iter
    T(i)=(i-1)*P.Ts;
    for qcount=1:P.Nq
        Xtrue_plot(:,i,qcount)=Xtrue(:,qcount);
        Xfilter_plot(:,i,qcount)=Xfilter(:,qcount);
        Pfilter_plot(:,i,qcount)=3*diag(Pfilter(:,:,qcount)).^0.5;
        Plot_input(:,i,qcount)=in(:,qcount);
        
        [Xtrue(:,qcount),iw(qcount),gyroOpwn,accelOpwn,in(:,qcount)]=Quadrotor12state(P,Xtrue(:,qcount),iw(qcount),i,qcount);
        Z=GeneralSensor(Xtrue(:,qcount),P);
        %     % Prediction Step
        X0=Xfilter(:,qcount);
        [Xfilter(:,qcount),Pfilter(:,:,qcount)]=Predict(Xfilter(:,qcount),Pfilter(:,:,qcount),Q1,P,gyroOpwn,accelOpwn);
        [Yt,yt]=TotalMeasurementInfo(Z,Xfilter(:,qcount),X0,P);
        
        Yfilter=Pfilter(:,:,qcount)^(-1);
        yfilter=Yfilter*Xfilter(:,qcount);
        Y=Yfilter+Yt;
        y=yfilter+yt;
        Pfilter(:,:,qcount)=Y^(-1);
        Xfilter(:,qcount)=Pfilter(:,:,qcount)*y;
    end
end
%% Plotting Data
% true pos data
for qcount=1:P.Nq
xtrue(qcount,:)=Xtrue_plot(1,:,qcount);
ytrue(qcount,:)=Xtrue_plot(2,:,qcount);
ztrue(qcount,:)=Xtrue_plot(3,:,qcount);
% true vel data
vxtrue(qcount,:)=Xtrue_plot(4,:,qcount);
vytrue(qcount,:)=Xtrue_plot(5,:,qcount);
vztrue(qcount,:)=Xtrue_plot(6,:,qcount);
ptrue(qcount,:)=Xtrue_plot(10,:,qcount);
qtrue(qcount,:)=Xtrue_plot(11,:,qcount);
rtrue(qcount,:)=Xtrue_plot(12,:,qcount);
% true att data
phitrue(qcount,:)=Xtrue_plot(7,:,qcount)*180/pi;
thetatrue(qcount,:)=Xtrue_plot(8,:,qcount)*180/pi;
psitrue(qcount,:)=Xtrue_plot(9,:,qcount)*180/pi;
% filter data
xfilter(qcount,:)=Xfilter_plot(1,:,qcount);
yyfilter(qcount,:)=Xfilter_plot(2,:,qcount);
zfilter(qcount,:)=Xfilter_plot(3,:,qcount);
% filter vel data
vxfilter(qcount,:)=Xfilter_plot(4,:,qcount);
vyfilter(qcount,:)=Xfilter_plot(5,:,qcount);
vzfilter(qcount,:)=Xfilter_plot(6,:,qcount);
% filter att data
phifilter(qcount,:)=Xfilter_plot(7,:,qcount)*180/pi;
thetafilter(qcount,:)=Xfilter_plot(8,:,qcount)*180/pi;
psifilter(qcount,:)=Xfilter_plot(9,:,qcount)*180/pi;
% error bounds
% filter data
Px(qcount,:)=Pfilter_plot(1,:,qcount);
Py(qcount,:)=Pfilter_plot(2,:,qcount);
Pz(qcount,:)=Pfilter_plot(3,:,qcount);
% filter vel data
Pvx(qcount,:)=Pfilter_plot(4,:,qcount);
Pvy(qcount,:)=Pfilter_plot(5,:,qcount);
Pvz(qcount,:)=Pfilter_plot(6,:,qcount);
% filter att data
Pphi(qcount,:)=Pfilter_plot(7,:,qcount)*180/pi;
Ptheta(qcount,:)=Pfilter_plot(8,:,qcount)*180/pi;
Ppsi(qcount,:)=Pfilter_plot(9,:,qcount)*180/pi;
end
%% Actual Plots
figure(1)% trajectory
plot3(xtrue(1,:),ytrue(1,:),ztrue(1,:),xfilter(1,:),yyfilter(1,:),zfilter(1,:),xtrue(2,:),ytrue(2,:),ztrue(2,:),xfilter(2,:),yyfilter(2,:),zfilter(2,:))
title(' 3-D Trajectory ');
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
legend('True');
%axis([-100 100 -100 100 -100 100])
% figure(2)% velocity
% plot(T,vxtrue,T,vytrue,T,vztrue,T,vxfilter,T,vyfilter,T,vzfilter)
% title('Velocity in Body Axis');
% xlabel('Time(s)');
% ylabel('Vel (m/s)');
% legend('u','v','w');
% figure(3)% attitude
% plot(T,phitrue,T,thetatrue,T,psitrue,T,phifilter,T,thetafilter,T,psifilter)
% title('Attitude of Quadrotor');
% xlabel('Time(s)');
% ylabel('Angle (rad)');
% legend('\phi','\theta','\psi');
% %% Error Plots
% figure(4)%
% subplot(3,3,1)
% plot(T,xtrue-xfilter,T,Px,T,-Px)
% ylabel('e_x(m)')
% subplot(3,3,2)
% plot(T,ytrue-yfilter,T,Py,T,-Py)
% ylabel('e_y(m)')
% subplot(3,3,3)
% plot(T,ztrue-zfilter,T,Pz,T,-Pz)
% ylabel('e_z(m)')
% subplot(3,3,4)
% plot(T,vxtrue-vxfilter,T,Pvx,T,-Pvx)
% ylabel('e_{vx}(m)')
% subplot(3,3,5)
% plot(T,vytrue-vyfilter,T,Pvy,T,-Pvy)
% ylabel('e_{vy}(m)')
% subplot(3,3,6)
% plot(T,vztrue-vzfilter,T,Pvz,T,-Pvz)
% ylabel('e_{vz}(m)')
% subplot(3,3,7)
% plot(T,phitrue-phifilter,T,Pphi,T,-Pphi)
% ylabel('e_{\phi}(m)')
% xlabel('time(s)')
% subplot(3,3,8)
% plot(T,thetatrue-thetafilter,T,Ptheta,T,-Ptheta)
% ylabel('e_{\theta}(m)')
% xlabel('time(s)')
% subplot(3,3,9)
% plot(T,psitrue-psifilter,T,Ppsi,T,-Ppsi)
% ylabel('e_{\psi}(m)')
% xlabel('time(s)')
% figure(5)
% plot(T,ptrue*180/pi,T,qtrue*180/pi,T,rtrue*180/pi)
% legend('p','q','r');
% figure(6)
% plot(T,Plot_input(1,:,qcount))
% figure(7)
% plot(T,Plot_input(2,:,qcount),T,Plot_input(3,:,qcount),T,Plot_input(4,:,qcount))
% 


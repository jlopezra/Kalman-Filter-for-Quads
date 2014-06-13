function RunSimulationCNSquad(P,P2)
%% Initialization
% Xtrue=zeros(12,P.Nq);
% Xfiltert=zeros(9,P.Nq);
%in=zeros(4,P.Nq);
% Ptemp=zeros(9);
% Ptemp(1,1)=P.sx^2;
% Ptemp(2,2)=P.sy^2;
% Ptemp(3,3)=P.sz^2;
% Ptemp(4,4)=P.svx^2;
% Ptemp(5,5)=P.svy^2;
% Ptemp(6,6)=P.svz^2;
% Ptemp(7,7)=P.sphi^2;
% Ptemp(8,8)=P.stheta^2;
% Ptemp(9,9)=P.spsi^2;
% Pfilter=zeros(P.Nq*9);
% for qcount=1:P.Nq
%     Xtrue(:,qcount)=[...
%         P.xn(qcount);...
%         P.ye(qcount);...
%         P.zd(qcount);...
%         P.u(qcount);...
%         P.v(qcount);...
%         P.w(qcount);...
%         P.phi(qcount);...
%         P.theta(qcount);...
%         P.psi(qcount);...
%         P.p(qcount);...
%         P.q(qcount);...
%         P.r(qcount)];
%     Xfiltert(:,qcount)=Xtrue(1:9,qcount)+[...
%         randn(1)*P.sx;...
%         randn(1)*P.sy;...
%         randn(1)*P.sz;...
%         randn(1)*P.svx;...
%         randn(1)*P.svy;...
%         randn(1)*P.svz;...
%         randn(1)*P.sphi;...
%         randn(1)*P.stheta;...
%         randn(1)*P.spsi];
%     in(:,qcount)=[P.g;0;0;0];% Force and other three torques
%     Pfilter(9*(qcount-1)+1:9*(qcount-1)+9,9*(qcount-1)+1:9*(qcount-1)+9)=Ptemp;
%     
% end
Xfiltert=P.Xfiltert;
Xtrue=P.Xtrue;
Pfilter=P.Pfilter;
Xfilter=reshape(Xfiltert,P.Nq*9,1);
in=P.in;
P.N=1;
P.Ts=1/30;

% process noise
Q1=zeros(6);
Q1(1,1)=P.sax^2;
Q1(2,2)=P.say^2;
Q1(3,3)=P.saz^2;
Q1(4,4)=P.sp^2;
Q1(5,5)=P.sq^2;
Q1(6,6)=P.sr^2;

gyroOpwn=zeros(3,P.Nq);
accelOpwn=zeros(3,P.Nq);
%% Number of iterations
iter=30*20;
Xtrue_plot=zeros(12,iter,P.Nq);
Xfilter_plot=zeros(9*P.Nq,iter);
Pfilter_plot=zeros(9*P.Nq,iter);
connectivity=zeros(P.Nq,iter);
T=zeros(1,iter);
iw=ones(1,P.Nq);
% Force and other three torques
%% Main Loop
close all;
hold off;
 [h,h2]=setupAnimation(P);
for i=1:iter
    i;
    T(i)=(i-1)*P.Ts;
    %states=[Xtrue(:,1);T(i)];
   % plotquadrotor2(states,P2);
    %% Real vehicle
  h=plot3DTraj(h,reshape(Xfilter,9,P.Nq),Xtrue,P);
    Xfilter_plot(:,i)=Xfilter;
    Pfilter_plot(:,i)=diag(Pfilter).^0.5;
    for qcount=1:P.Nq
        Xtrue_plot(:,i,qcount)=Xtrue(:,qcount);
        [Xtrue(:,qcount),iw(qcount),gyroOpwn(:,qcount),accelOpwn(:,qcount),in(:,qcount)]=Quadrotor12state(P,Xtrue(:,qcount),iw(qcount),i,qcount);
    end
    Xfilter0=Xfilter;
    %% Adjacency Matrix
    Adj=zeros(P.Nq+P.N);
    %% Prediction
    [Xfilter,Pfilter]=Predict(Xfilter,Pfilter,Q1,P,gyroOpwn,accelOpwn);
     [Yfilter,yfilter]=Convert2Info(Xfilter,Pfilter);
    %% Update
    for qcount=1:P.Nq
        [Zlm,Zq,Zr]=GeneralSensor(Xtrue,P,qcount,i);
         Adj=GenerateAdjacencyMatrix(Adj,Zlm,Zq,Zr,qcount,P.Nq);% compute adjecency matrix
        [xl,yl,zl]=findTopConnections(Xtrue,Zlm,Zq,Zr,P,qcount);
        h2(qcount)=drawlines(xl,yl,zl,h2(qcount));
        [Yl,yl]=TotalMeasurementInfoFromLM(Zlm,Xfilter,Xfilter0,P,qcount);
        [Yq,yq]=TotalMeasurementInfoFromInterQuadrotor(Zq,Xfilter,Xfilter0,P,qcount);
        Yfilter=Yfilter+Yl+Yq;
        yfilter=yfilter+yl+yq;
    end
     Conn=GenerateConnectivityMatrix(Adj,P.Nq+1);
     connectivity(:,i)=GenerateConnVec(Conn,P.Nq,P.N);
    [Xfilter,Pfilter]=Convert2State(Yfilter,yfilter);
    
end
hold off;
%% save (use care fully it can delete the previous data)
%save('simdataTs30OneLM.mat','P','Xtrue_plot','Xfilter_plot','Pfilter_plot','T');
figure(2)
for i=1:P.Nq
plot3(Xtrue_plot(1,:,i),Xtrue_plot(2,:,i),Xtrue_plot(3,:,i),'LineWidth',2)
hold on
plot3(Xfilter_plot(9*(i-1)+1,:),Xfilter_plot(9*(i-1)+2,:),Xfilter_plot(9*(i-1)+3,:),'Color','r','LineWidth',2,'LineStyle','-.');
hold on
end
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
axis([-1.3 1.3 -1.3 1.3 0 1.3])

hold off
myLabel=['x_{err}(m)' 'y_{err}(m)' 'z_{err}(m)' '\phi_{err}(m)' '\theta_{err}(m)' '\psi_{err}(m)'];
size(myLabel)
for i=1:P.Nq
    figure(3+(i-1))
for j=1:6
subplot(2,3,j)
plot(T,Xtrue_plot(j,:,i)-Xfilter_plot(9*(i-1)+j,:),T,3*Pfilter_plot(9*(i-1)+j,:),T,-3*Pfilter_plot(9*(i-1)+j,:))
xlabel('time(s)');
ylabel(myLabel(j));

end

end
figure(3+P.Nq+1)
plot(T,connectivity(1,:),T,connectivity(2,:),T,connectivity(3,:))


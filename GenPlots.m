function GenPlots
clc
close all;
clear all;
%load simdataTs30TwoLM.mat;
%load simdataTs10TwoLM.mat;
load simdataTs30OneLM.mat;
[m,n]=size(T);

freq=60;
c=n/freq;
count=1;
Xtrue=zeros(12,c+1,P.Nq);
Xfilter=zeros(9*P.Nq,c+1);
Pfilter=zeros(9*P.Nq,c+1);
TT=zeros(1,c+1);
for qcount=1:P.Nq
        Xtrue(:,count,qcount)=Xtrue_plot(:,1,qcount);
     end
    Xfilter(:,count)=Xfilter_plot(:,1);
    Pfilter(:,count)=Pfilter_plot(:,1);
for i=1:n
   if(rem(i,freq)==0)
       count=count+1;
     for qcount=1:P.Nq
        Xtrue(:,count,qcount)=Xtrue_plot(:,i,qcount);
     end
    Xfilter(:,count)=Xfilter_plot(:,i);
    Pfilter(:,count)=Pfilter_plot(:,i);
    TT(count)=T(i);
   end
end
figure(1)

for i=1:P.Nq
plot3(Xtrue(1,:,i),Xtrue(2,:,i),Xtrue(3,:,i),'LineWidth',1.3,'Color','k')
hold on
plot3(Xfilter(9*(i-1)+1,:),Xfilter(9*(i-1)+2,:),Xfilter(9*(i-1)+3,:),'Color','r','LineWidth',1.2,'LineStyle','--','Marker','o','MarkerSize',3);
hold on
if(i==1)
    legend('truth','filter');
end
end
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
axis([-1.3 1.3 -1.3 1.3 0 1.3])
view(53,10)
for i=1:P.N
    plot3(P.Xl(i),P.Yl(i),P.Zl(i),'*','markersize',13,'LineWidth',2)
    hold on
    text(P.Xl(i)+0.09,P.Yl(i)+0.09,P.Zl(i)+0.09,['LM' int2str(i)])
end
hold off
myLabel=['x_{err}(m)' 'y_{err}(m)' 'z_{err}(m)' '\phi_{err}(m)' '\theta_{err}(m)' '\psi_{err}(m)'];
size(myLabel)
for i=1:P.Nq
    figure(3+(i-1))
%for j=1:6
title(['UAV' int2str(i)])
j=1;
subplot(3,3,j)
plot(TT,Xtrue(j,:,i)-Xfilter(9*(i-1)+j,:),'LineWidth',1.2,'LineStyle','--','Marker','o','MarkerSize',3);
hold on
plot(TT,3*Pfilter(9*(i-1)+j,:),TT,-3*Pfilter(9*(i-1)+j,:),'Color','r')
xlabel('time(s)');
ylabel('X_{err}(m)');
%legend('err','\pm3\sigma')
j=2;
subplot(3,3,j)
plot(TT,Xtrue(j,:,i)-Xfilter(9*(i-1)+j,:),'LineWidth',1.2,'LineStyle','--','Marker','o','MarkerSize',3);
hold on
plot(TT,3*Pfilter(9*(i-1)+j,:),TT,-3*Pfilter(9*(i-1)+j,:),'Color','r')
xlabel('time(s)');
ylabel('Y_{err}(m)');

j=3;
subplot(3,3,j)
plot(TT,Xtrue(j,:,i)-Xfilter(9*(i-1)+j,:),'LineWidth',1.2,'LineStyle','--','Marker','o','MarkerSize',3);
hold on
plot(TT,3*Pfilter(9*(i-1)+j,:),TT,-3*Pfilter(9*(i-1)+j,:),'Color','r')
xlabel('time(s)');
ylabel('Z_{err}(m)');

j=4;
subplot(3,3,j)
plot(TT,Xtrue(j,:,i)-Xfilter(9*(i-1)+j,:),'LineWidth',1.2,'LineStyle','--','Marker','o','MarkerSize',3);
hold on
plot(TT,3*Pfilter(9*(i-1)+j,:),TT,-3*Pfilter(9*(i-1)+j,:),'Color','r')
xlabel('time(s)');
ylabel('u_{err}(m/s)');

j=5;
subplot(3,3,j)
plot(TT,Xtrue(j,:,i)-Xfilter(9*(i-1)+j,:),'LineWidth',1.2,'LineStyle','--','Marker','o','MarkerSize',3);
hold on
plot(TT,3*Pfilter(9*(i-1)+j,:),TT,-3*Pfilter(9*(i-1)+j,:),'Color','r')
xlabel('time(s)');
ylabel('v_{err}(m/s)');

j=6;
subplot(3,3,j)
plot(TT,Xtrue(j,:,i)-Xfilter(9*(i-1)+j,:),'LineWidth',1.2,'LineStyle','--','Marker','o','MarkerSize',3);
hold on
plot(TT,3*Pfilter(9*(i-1)+j,:),TT,-3*Pfilter(9*(i-1)+j,:),'Color','r')
xlabel('time(s)');
ylabel('w_{err}(m/s');

j=7;
subplot(3,3,j)
plot(TT,Xtrue(j,:,i)-Xfilter(9*(i-1)+j,:),'LineWidth',1.2,'LineStyle','--','Marker','o','MarkerSize',3);
hold on
plot(TT,3*Pfilter(9*(i-1)+j,:),TT,-3*Pfilter(9*(i-1)+j,:),'Color','r')
xlabel('time(s)');
ylabel('\phi_{err}(rad)');

j=8;
subplot(3,3,j)
plot(TT,Xtrue(j,:,i)-Xfilter(9*(i-1)+j,:),'LineWidth',1.2,'LineStyle','--','Marker','o','MarkerSize',3);
hold on
plot(TT,3*Pfilter(9*(i-1)+j,:),TT,-3*Pfilter(9*(i-1)+j,:),'Color','r')
xlabel('time(s)');
ylabel('\theta_{err}(rad)');

j=9;
subplot(3,3,j)
plot(TT,Xtrue(j,:,i)-Xfilter(9*(i-1)+j,:),'LineWidth',1.2,'LineStyle','--','Marker','o','MarkerSize',3);
hold on
plot(TT,3*Pfilter(9*(i-1)+j,:),TT,-3*Pfilter(9*(i-1)+j,:),'Color','r')
xlabel('time(s)');
ylabel('\psi_{err}(rad)');

%end

end
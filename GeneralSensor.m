function [Zlm,Zq,Zr]=GeneralSensor(Xtrue,P,quadindex,t)

Xk=Xtrue(:,quadindex);
x=Xk(1);
y=Xk(2);
z=Xk(3);
u=Xk(4);
v=Xk(5);
w=Xk(6);
phi=Xk(7);
theta=Xk(8);
psi=Xk(9);

persistent flag;
if(t==1)
    flag=1;
end
SwT=30;
aa=rem(t,SwT);
% if(aa==0)
%     if(flag==0)
%         flag=1;
%     else
%         flag=0;
%     end
% end
Zlm=[];
lm=0;
if(flag==1)
for i=1:size(P.Xl,1) % Iterates as many times as there are landmarks JCL
    Xj=[P.Xl(i);P.Yl(i);P.Zl(i)];
    Rij=RangeMeasurement(Xk,P,Xj);
    [n1,n2]=BearingMeasurement3d(Xk,P,Xj);
    if((Rij<P.Rsensor) && (abs(n1)<.525) && (abs(n2)<.525))
        lm=lm+1;
        Zlm(lm,:)=[Rij+randn(1)*P.sig_R n1+P.sig_eta*randn(1) n2+P.sig_eta*randn(1) i]; % vector of the three measurements with errors JCL
    end
end
end
Zq=[];
lq=0;
for i=1:P.Nq %
    if(i~=quadindex)
        Xj=Xtrue(1:3,i);
   % Xj=[P.Xl(i);P.Yl(i);P.Zl(i)];
    Rij=RangeMeasurement(Xk,P,Xj);
    [n1,n2]=BearingMeasurement3d(Xk,P,Xj);
    if((Rij<P.Rsensor) && (abs(n1)<.525) && (abs(n2)<.525)) % Rsensor = 1.8 
        lq=lq+1;
        Zq(lq,:)=[Rij+randn(1)*P.sig_R n1+P.sig_eta*randn(1) n2+P.sig_eta*randn(1) i ]; % vector of the three measurements with errors. Only calculated if the other quadcopters are in range JCL
    end
    end
end
Zr=[];
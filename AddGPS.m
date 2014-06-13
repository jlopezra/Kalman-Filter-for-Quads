function [Igps,igps]=AddGPS(index,Xtrue,P,Xfilter)
% Xk=Xtrue(1:9,index);
% Xgps=Xk+0.01*randn(9,1);
% Igps=zeros(9*P.Nq);
% igps=zeros(9*P.Nq,1);
% Rgps=0.01^2*eye(9);
% H=zeros(9,9*P.Nq);
% H(1:9,9*(index-1)+1:9*(index-1)+9)=eye(9);
% 
%  Igps=Igps+H'*Rgps^(-1)*H;
%     X1=Xfilter(9*(index-1)+1:9*(index-1)+9,1);
%     igps=igps+H'*Rgps^(-1)*((Xgps-X1)+X1);
    
Xk=Xtrue(1:3,index);
Xgps=Xk+0.1*randn(3,1);
Igps=zeros(9*P.Nq);
igps=zeros(9*P.Nq,1);
Rgps=0.1^2*eye(3);
H=zeros(3,9*P.Nq);
H(1:3,9*(index-1)+1:9*(index-1)+3)=eye(3);

Igps=Igps+H'*Rgps^(-1)*H;
X1=Xfilter(9*(index-1)+1:9*(index-1)+3,1);
igps=igps+H'*Rgps^(-1)*((Xgps-X1)+X1);
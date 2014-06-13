function [Xk,Pk]=Predict(Xk,Pk,Q1,P,gyroOp,accelOp)
F=zeros(9*P.Nq);
B=zeros(9*P.Nq,6*P.Nq);
Q1t=zeros(6*P.Nq);
for i=1:P.Nq
[F(9*(i-1)+1:9*(i-1)+9,9*(i-1)+1:9*(i-1)+9),B(9*(i-1)+1:9*(i-1)+9,6*(i-1)+1:6*(i-1)+6)]=VehicleJacobian(Xk(9*(i-1)+1:9*(i-1)+9,1),P,gyroOp(:,i));
Xk(9*(i-1)+1:9*(i-1)+9,1)=QuadRootor9StateDynamics(Xk(9*(i-1)+1:9*(i-1)+9,1),P,accelOp(:,i),gyroOp(:,i));
Q1t(6*(i-1)+1:6*(i-1)+6,6*(i-1)+1:6*(i-1)+6)=Q1;
end
Pk=F*Pk*F'+B*Q1t*B';
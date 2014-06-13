function [I,i]=informationGainbyMeasurement(Z,Xk,P,X0)
I=zeros(9);
i=zeros(9,1);
R=eye(3)*P.sig_eta^2;
if(isempty(Z)==1)
    I=zeros(9);
    i=zeros(9,1);
else
    for j=1:size(Z,1)
        [H]=jacobianMeasurement(X0,j,P);
        Zest=compute3dbearing(Xk,j,P);
        I=I+H'*R^(-1)*H;
        eta=[Z(j,1);Z(j,2);Z(j,3)];
        mu=eta-Zest;
        mu(2,1)=pi_to_pi(mu(2,1));
        mu(3,1)=pi_to_pi(mu(3,1));
        i=i+H'*R^(-1)*(mu);
    end
end

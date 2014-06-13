function [Yt,yt]=TotalMeasurementInfo(Z,Xk,X0,P)
Yt=zeros(9);
yt=zeros(9,1);
if(isempty(Z)==0)
    for i=1:size(Z,1)
        j=Z(i,4);
        %% Info From range
%         Hr=SingleRangeJacobian(X0,P,j);
%         InfoR=Hr'*((P.sig_R)^2)^(-1)*Hr;
%         Yt=Yt+InfoR;
%         Ract=Z(i,1);
%         Rest=RangeMeasurement(Xk,P,j);
%         mu=Ract-Rest;
%         infoR=Hr'*((P.sig_R)^2)^(-1)*(mu+Hr*Xk);
%         yt=yt+infoR;
        %% info from bearing 1
        [Hb1,Hb2]=SingleBearingJacobian(X0,P,j);
        %Hr=SingleRangeJacobian(X0,P,j);
        Infob1=Hb1'*((P.sig_eta)^2)^(-1)*Hb1;
        Yt=Yt+Infob1;
        n1=Z(i,2);
        [n1est,n2est]=BearingMeasurement3d(Xk,P,j);
        mu=pi_to_pi(n1-n1est);
        infob1=Hb1'*((P.sig_eta)^2)^(-1)*(mu+Hb1*Xk);
        yt=yt+infob1;
        
        %% Info from bearing 2
        Infob2=Hb2'*((P.sig_eta)^2)^(-1)*Hb2;
        Yt=Yt+Infob2;
        n2=Z(i,3);
        %[n1est,n2est]=BearingMeasurement3d(Xk,P,j);
        mu=pi_to_pi(n2-n2est);
        infob2=Hb2'*((P.sig_eta)^2)^(-1)*(mu+Hb2*Xk);
        yt=yt+infob2;
        
    end
end

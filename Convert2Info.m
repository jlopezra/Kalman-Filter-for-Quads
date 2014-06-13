function [Yfilter,yfilter]=Convert2Info(Xfilter,Pfilter)
Yfilter=Pfilter^(-1);
yfilter=Yfilter*Xfilter;

function [Xfilter,Pfilter]=Convert2State(Yfilter,yfilter)
Pfilter=Yfilter^(-1);
Xfilter=Pfilter*yfilter;

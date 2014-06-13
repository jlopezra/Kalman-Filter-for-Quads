function [xl,yl,zl]=findTopConnections(Xtrue,Zlm,Zq,Zr,P,index)
% this function plots the inter vehicle and landmark bearing conncetions
%% ------Bearing only cooperative navigation system------------------------
%  MAGICC LAB, Department of Electrical and Computer Engineering,
%  Brigham Young University, Provo
%  Authors: Rajnikant Sharma :  raj.drdo@gmail.com
%           Clark N.Taylor   :  clark.taylor@byu.edu
%  Last Updated: Rajnikant Sharma Oct 12 2009. 
%--------------------------------------------------------------------------
X1=Xtrue(:,index);

xl=[];
yl=[];
zl=[];
if(isempty(Zq)==0)
    for k=1:size(Zq,1) 
        j=Zq(k,4);
        
        X2=Xtrue(:,j);
        xx=[X1(1) X2(1)];
        yy=[X1(2) X2(2)];
        zz=[X1(3) X2(3)];
        xl=[xl xx];
        yl=[yl yy];
        zl=[zl zz];
    end
end
if(isempty(Zlm)==0)
    for k=1:size(Zlm,1)
        j=Zlm(k,4);
        clear X2;
        X2=[P.Xl(j);P.Yl(j);P.Zl(j)];
        xx=[X1(1) X2(1)];
        yy=[X1(2) X2(2)];
        zz=[X1(3) X2(3)];
        xl=[xl xx];
        yl=[yl yy];
        zl=[zl zz];
    end
end



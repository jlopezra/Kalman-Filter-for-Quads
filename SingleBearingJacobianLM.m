function [H1,H2]=SingleBearingJacobianLM(Xfilter,P,j,index)
% j= landmark index
% index= quadrotor index
Xk=Xfilter(9*(index-1)+1:9*(index-1)+9,1);
x=Xk(1);
y=Xk(2);
z=Xk(3);
u=Xk(4);
v=Xk(5);
w=Xk(6);
phi=Xk(7);
theta=Xk(8);
psi=Xk(9);
xl=P.Xl(j);
yl=P.Yl(j);
zl=P.Zl(j);
Hb1 =[ -(y - yl)/((x - xl)^2*((y - yl)^2/(x - xl)^2 + 1)), 1/((x - xl)*((y - yl)^2/(x - xl)^2 + 1)), 0, 0, 0, 0, 0, 0, -1];
Hb2 =[ ((2*x - 2*xl)*(z - zl))/(2*((x - xl)^2 + (y - yl)^2)^(3/2)*((z - zl)^2/((x - xl)^2 + (y - yl)^2) + 1)), ((2*y - 2*yl)*(z - zl))/(2*((x - xl)^2 + (y - yl)^2)^(3/2)*((z - zl)^2/((x - xl)^2 + (y - yl)^2) + 1)), -1/(((x - xl)^2 + (y - yl)^2)^(1/2)*((z - zl)^2/((x - xl)^2 + (y - yl)^2) + 1)), 0, 0, 0, 0, -1, 0];
H1=zeros(1,9*P.Nq);
H2=zeros(1,9*P.Nq);
H1(1,9*(index-1)+1:9*(index-1)+9)=Hb1;
H2(1,9*(index-1)+1:9*(index-1)+9)=Hb2;
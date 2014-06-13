function H=SingleRangeJacobianLM(Xfilter,P,j,index)
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
Hr =[(2*x - 2*xl)/(2*((x - xl)^2 + (y - yl)^2 + (z - zl)^2)^(1/2)),                                          (2*y - 2*yl)/(2*((x - xl)^2 + (y - yl)^2 + (z - zl)^2)^(1/2)),                   (2*z - 2*zl)/(2*((x - xl)^2 + (y - yl)^2 + (z - zl)^2)^(1/2)), 0, 0, 0, 0,  0,  0];
H=zeros(1,9*P.Nq);

H(1,9*(index-1)+1:9*(index-1)+9)=Hr;
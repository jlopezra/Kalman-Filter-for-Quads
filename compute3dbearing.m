function Zest=compute3dbearing(Xk,j,P)

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
n1=atan2((yl-y),(xl-x))-psi;
Rx=((x-xl)^2+(y-yl)^2)^0.5;
R=((x-xl)^2+(y-yl)^2+(z-zl)^2)^0.5;
n2=atan2((zl-z),Rx)-theta;

Zest=[R;pi_to_pi(n1);(n2)];

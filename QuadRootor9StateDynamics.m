function  Xk_plus_1=QuadRootor9StateDynamics(Xk,P,accelOp,gyroOp)
x=Xk(1);
y=Xk(2);
z=Xk(3);
u=Xk(4);
v=Xk(5);
w=Xk(6);
phi=Xk(7);
theta=Xk(8);
psi=Xk(9);

fx=zeros(9,1);
fx(1:3,1)=rotationMatb_to_w(phi,theta,psi)*[u;v;w];
fx(4:6,1)=-(cross(gyroOp,[u;v;w]))+P.g*[-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)]+accelOp;
fx(7:9,1)=Gyro2BodyRotation(phi,theta)*gyroOp;
fx;
Xk_plus_1=Xk+P.Ts*fx;


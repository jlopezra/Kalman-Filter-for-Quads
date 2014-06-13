function u=PID_rollAngle(kp,kd,ki,phi,phi_c,P,p)
err=phi_c-phi;
u=kp*err-kd*p+ki*err*P.Ts;


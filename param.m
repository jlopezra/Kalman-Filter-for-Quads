% param file
close all;
clear all;
clc;
P.Ts=1/30;
P.Jx = 0.114700;
P.Jy = 0.057600;
P.Jz = 0.171200;
% gravity (m/s^2)
P.g = 9.806650;
% mass (kg)
P.m = 1.56;
P.l    = 0.3;       % wingspan (m)
P.rho  = 1.268200;  % air density

% propeller characteristics
P.kmotor_lift = 5.45;  % F = 5.45*delta, where delta\in[0,1]
P.kmotor_drag = 0.0549; %

P.M = [...
    P.kmotor_lift    , P.kmotor_lift     , P.kmotor_lift     , P.kmotor_lift    ;...
    0                , -P.l*P.kmotor_lift, 0                 , P.l*P.kmotor_lift;...
    P.l*P.kmotor_lift, 0                 , -P.l*P.kmotor_lift, 0                ;...
    -P.kmotor_drag   , P.kmotor_drag     , -P.kmotor_drag    , P.kmotor_drag    ...
    ];
%% initial state error
P.sx=0.1;
P.sy=0.1;
P.sz=0.1;
P.sphi=0.05;
P.stheta=0.05;
P.spsi=0.05;
P.svx=0.01;
P.svy=0.01;
P.svz=0.01;
%% Process Noise
P.sax=0.03;
P.say=0.03;
P.saz=0.03;
P.sp=0.03;
P.sq=0.03;
P.sr=0.03;
%% Measurement Noise
P.sig_eta=0.02;
P.Rsensor=1.8;
P.sig_R=1;
P.sig_eta=0.02;
%% Landmarks
 P.N=2;
% P.Xl=randn(P.N,1)*2;
% P.Yl=randn(P.N,1)*2;
% P.Zl=rand(P.N,1)*1.5;
% P.Xl=[1];
% P.Yl=[0.5];
% P.Zl=[1.2];
%  P.Xl=[1;-1.1];
%  P.Yl=[0.5;0.9];
P.Xl=[1;1.5];
 P.Yl=[0.5;1];
 P.Zl=[1.2;0.5];
%% PID  Controller constants
% Attitude Controller
P.Att.kp=0.1; %% 0.5 0.4 0.2
P.Att.kd=0.2;
P.Att.ki=0.1;
% Waypoint Controller
P.Waypoint.kp=0.08;
P.Waypoint.kd=0.6;
P.Waypoint.ki=0.05;
%

%% Quadrotor True States
P.Nq=5; % number of quadrotors
P.xn=[-0.5 -1.2 0.2 1.2 -1.2];% pos
P.ye=[-0.5 0.1 0.1 -0.1 -1.1];
P.zd=[0.3 0.5 0.8 1 1.2];
P.u=[0 0 0 0 0];%vel
P.v=[0 0 0 0 0];
P.w=[0 0 0 0 0];
P.phi=[0 0 0 0 0];% attitude
P.theta=[0 0 0 0 0];
P.psi=[0 0 0 0 0];
P.p=[0 0 0 0 0];% angular rates
P.q=[0 0 0 0 0];
P.r=[0 0 0 0 0];
%% Way points
P.Wn=4;
% P.Xw(:,1)=[0;0.5;0.5;0];
% P.Yw(:,1)=[0;0;0.5;0.5];
% P.Zw(:,1)=[1;1;1;1];
% P.Xw(:,2)=[0.25;0.75;0.75;0.25];
% P.Yw(:,2)=[-0.25;-0.5;0.5;0.5];
% P.Zw(:,2)=[1.5;1.5;1.5;1.5];
% P.Xw(:,3)=[0.5;0.5;-0.5;-0.5];
% P.Yw(:,3)=[0.5;1.5;1.5;0.5];
% P.Zw(:,3)=[0.5;0.5;0.5;0.5];

P.Xw(:,1)=[-0.5;0.5;0.5;-0.5];
P.Yw(:,1)=[-0.5;-0.5;0.5;0.5];
P.Zw(:,1)=[0.3;0.3;0.3;0.3];
P.Xw(:,2)=[-1.2;-0.2;-0.2;-1.2];
P.Yw(:,2)=[0.1;0.1;1.1;1.1];
P.Zw(:,2)=0.5*[1;1;1;1];
P.Xw(:,3)=[0.2;1.2;1.2;0.2];
P.Yw(:,3)=[0.1;0.1;1.1;1.1];
P.Zw(:,3)=0.8*[1;1;1;1];
P.Xw(:,4)=[1.2;1.2;0.2;0.2];
P.Yw(:,4)=[-0.1;-1.1;-1.1;-0.1];
P.Zw(:,4)=1*[1;1;1;1];
P.Xw(:,5)=[-1.2;-0.2;-0.2;-1.2];
P.Yw(:,5)=[-1.1;-1.1;-0.1;-0.1];
P.Zw(:,5)=1.2*[1;1;1;1];
P.Rd=0.1;
%%
P.Xtrue=zeros(12,P.Nq);
P.Xfiltert=zeros(9,P.Nq);
P.in=zeros(4,P.Nq);
Ptemp=zeros(9);
Ptemp(1,1)=P.sx^2;
Ptemp(2,2)=P.sy^2;
Ptemp(3,3)=P.sz^2;
Ptemp(4,4)=P.svx^2;
Ptemp(5,5)=P.svy^2;
Ptemp(6,6)=P.svz^2;
Ptemp(7,7)=P.sphi^2;
Ptemp(8,8)=P.stheta^2;
Ptemp(9,9)=P.spsi^2;
P.Pfilter=zeros(P.Nq*9);
for qcount=1:P.Nq
   P.Xtrue(:,qcount)=[...
        P.xn(qcount);...
        P.ye(qcount);...
        P.zd(qcount);...
        P.u(qcount);...
        P.v(qcount);...
        P.w(qcount);...
        P.phi(qcount);...
        P.theta(qcount);...
        P.psi(qcount);...
        P.p(qcount);...
        P.q(qcount);...
        P.r(qcount)];
    P.Xfiltert(:,qcount)=P.Xtrue(1:9,qcount)+[...
        randn(1)*P.sx;...
        randn(1)*P.sy;...
        randn(1)*P.sz;...
        randn(1)*P.svx;...
        randn(1)*P.svy;...
        randn(1)*P.svz;...
        randn(1)*P.sphi;...
        randn(1)*P.stheta;...
        randn(1)*P.spsi];
    P.in(:,qcount)=[P.g;0;0;0];% Force and other three torques
    P.Pfilter(9*(qcount-1)+1:9*(qcount-1)+9,9*(qcount-1)+1:9*(qcount-1)+9)=Ptemp;
    
end

%% robot parameters
P1.N=3; % number of robots
P1.sig_eta=0.2;% bearing measurement noise
P1.sx=2;% intial x uncertanity
P1.sy=2;% intial y uncertanity
P1.sTheta=0.1;% initial heading uncertanity
P1.Xn=0.15;% process noise % gyro
P1.V=5; % vehicle velocity
P1.Vn=0.3; %

%% Plot Parameters
% physical constants
P2.M = 2;
P2.m = 10;
P2.L = 0.5;
P2.g = 9.8;

% initial conditions
P2.y0 = 2;
P2.ydot0 = 0;
P2.theta0 = 40*pi/180;
P2.thetadot0 = 0;

% sample time
P2.Ts = 0.01;

% gain for dirty derivative
P2.tau = 1/20;

% control design
A = [...
    0, 1, 0, 0;...
    0, 0, -P2.m/P2.M*P2.g, 0;...
    0, 0, 0, 1;...
    0, 0, (P2.M+P2.m)/P2.M/P2.L*P2.g, 0;....
    ];
B = [...
    0;...
    1/P2.M;...
    0;...
    -1/P2.M/P2.L;...
    ];
P2.Q = diag([1,1,1,1]);
P2.R = 1;
P2.K = lqr(A,B,P2.Q,P2.R);

% drawing parameters
% distances are in feet.
%------------------------------------------------------
% course dimentions
%6x11 from the center
P2.courselength = 11;
P2.coursewidth = 6;
%6ft columns plus four feet of head room
P2.courseheight = 10;

% quadrotor starting point at 3 ft high and 3 ft in front of the pillars
P2.zstart = 0;
P2.xstart = 0;
P2.ystart = 0;

% quadrotor dimentions
%2 inches from the center
P2.boxlength = 1/6;
P2.boxwidth = P2.boxlength;
%4.5 inches from the bottom to the top
P2.boxheight = 0.375;
%6 inches from box to motor
P2.armlength = 0.5;
%1/8 inch from center to the side of a rod
P2.armwidth = 1/96;
%1/4 inch from the top of the box
P2.armposition = 1/48;
%25*(2)^(1/2)/2 inches from center to perimeter along x and y axis
P2.perimeter = 1.47;
%perimeter has 3/16 inch width
P2.periwidth = 1/64;

% pillar dimentions
%6 ft high
P2.pillarheight = 6;
%5 ft offset;
P2.pillarposition = 5;
%5 in radius
P2.pillarradius = 0.417;
%The adjustment for the none circular pillar
P2.pillaradd = 0.167;
%The height of each strip on color
P2.pillarc1 = 0.833;
P2.pillarc2 = 0.5;


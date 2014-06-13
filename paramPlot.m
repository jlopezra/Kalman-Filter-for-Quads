

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


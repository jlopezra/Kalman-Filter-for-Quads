%==========================================================================
%This is a modificaition of the quadrotor graphics code given to ECEn483
%students of fall 08.
%
%1/17/09: added pillars to the arena and a perimeter to the quadrotor. RKR
%==========================================================================

function plotquadrotor2(uu,P)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    %rotate phi and theta according to psi
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    tmp = R * [phi;theta];
    phi = tmp(1);
    theta = tmp(2);
    
    % define persistent variables 
    persistent fig_quadrotor;
    persistent fig_pillarsl;
    persistent fig_pillarsr;
    
    % parameters of the course
    length = P.courselength;
    width = P.coursewidth;
    height = P.courseheight;

    
    % first time function is called, initialize plot and persistent vars
    if t<=0.1,
        figure(1), clf
        
        %draws cylinders
%         [X, Y, Z] = cylinder(r);
%         Rpx =[D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D;...
%               D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D, D];
%         
%         X1 = Rpx+X;
%         X2 = -Rpx+X;
%         Z = P.pillarheight*Z;
% 
%         surf(X1, Y, Z);
%         hold on
%         surf(X2, Y, Z);
%         hold on
        
        %draws quadrotor at initial position
        fig_quadrotor = drawquadrotor(pn,pe,pd,phi,theta,psi, P, [], 'normal');
%         fig_pillarsl = drawpillarsl(P, [], 'normal');
%         fig_pillarsr = drawpillarsr(P, [], 'normal');
        
        axis([-11,11,-11,11,0,22]); % see if it is possible to change the axis and not distort the image
        xlabel('X axis');
        ylabel('Y axis');
        zlabel('Z axis');
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawquadrotor(pn,pe,pd,phi,theta,psi, P, fig_quadrotor);
    end
end

  
%=======================================================================
% drawquadrotor
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
function handle = drawquadrotor(pn,pe,pd,phi,theta,psi, P, handle, mode)
  
  % define points on spacecraft
  [V, F, patchcolors] = quadrotorVFC(P);
  
  % rotate spacecraft
  V = rotateVert(V, phi, theta, psi);
  
  % translate spacecraft
  V = translateVert(V, [pn; pe; pd]);
  
  if isempty(handle),
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
end

%=======================================================================
% drawpillarsl / draws left pillar
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
function handle = drawpillarsl(P, handle, mode)
  
  % define points on spacecraft
  [Vp, Fp, patchcolorsp] = pillarVFCl(P);
  
  if isempty(handle),
  handle = patch('Vertices', Vp, 'Faces', Fp,...
                 'FaceVertexCData',patchcolorsp,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
end

%=======================================================================
% drawpillarsr / draws right pillar
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
function handle = drawpillarsr(P, handle, mode)
  
  % define points on spacecraft
  [Vp, Fp, patchcolorsp] = pillarVFCr(P);
  
  if isempty(handle),
  handle = patch('Vertices', Vp, 'Faces', Fp,...
                 'FaceVertexCData',patchcolorsp,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
end


%=======================================================================
% quadrotor
%=======================================================================
%not done
function [V, F, patchcolors]=quadrotorVFC(P)

% Define the vertices (physical location of vertices)
  V = [...
	-P.boxlength P.boxwidth 0;... %1  center box
    -P.boxlength -P.boxwidth 0;... %2
    -P.boxlength -P.boxwidth P.boxheight;... %3
    -P.boxlength P.ystart+P.boxwidth P.zstart+P.boxheight;... %4
	P.boxlength P.ystart+P.boxwidth P.zstart;... %5
    P.boxlength P.ystart-P.boxwidth P.zstart;... %6
    P.boxlength P.ystart-P.boxwidth P.zstart+P.boxheight;... %7
    P.boxlength P.ystart+P.boxwidth P.zstart+P.boxheight;... %8
	-P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %9 arm left
    -P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %10
    -P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %11
    -P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %12
	-P.boxlength-P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %13
    -P.boxlength-P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %14
    -P.boxlength-P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %15
    -P.boxlength-P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %16
	P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %17 arm right
    P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %18
    P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %19
    P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %20
	P.boxlength+P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %21
    P.boxlength+P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %22
    P.boxlength+P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %23
    P.boxlength+P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %24
    P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %25 arm front
    P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %26
    -P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %27
    -P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %28
    P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %29
    P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %30
    -P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %31
    -P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %32
    P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %33 back front
    P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %34
    -P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %35
    -P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %36
    P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %37
    P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %38
    -P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %39
    -P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %40 
    0 P.ystart-P.perimeter P.zstart+P.boxheight;... %41 perimeter
    0 P.ystart-P.perimeter P.zstart+P.boxheight-P.periwidth;... %42
    0 P.ystart+P.perimeter P.zstart+P.boxheight;... %43 
    0 P.ystart+P.perimeter P.zstart+P.boxheight-P.periwidth;... %44
    P.perimeter P.ystart P.zstart+P.boxheight;... %45
    P.perimeter P.ystart P.zstart+P.boxheight-P.periwidth;... %46
    -P.perimeter P.ystart P.zstart+P.boxheight;... %47
    -P.perimeter P.ystart P.zstart+P.boxheight-P.periwidth;... %48
    0 0 P.boxheight+(1/12);... %49 camera
    -1/24 1/12 P.boxheight+(1/12)-(1/32);... %50
    1/24 1/12 P.boxheight+(1/12)-(1/32);... %51
    -1/24 1/12 P.boxheight+(1/12)+(1/32);... %52
    1/24 1/12 P.boxheight+(1/12)+(1/32);... %53
    
  ];

% define faces as a list of vertices numbered above
  F = [...
        1, 2, 3, 4;... % left
        5, 6, 7, 8;... % right
        2, 6, 7, 3;... % back 
        1, 5, 8, 4;... % front
        4, 3, 7, 8;... % top
        1, 2, 6, 5;... % bottom
        16, 15, 11, 12;... %left arm
        15, 14, 10, 11;...
        13, 14, 10, 9;...
        13, 9, 12, 16;...
        19, 23, 22, 18;... %right arm
        20, 19, 23, 24;...
        17, 18, 22, 21;...
        17, 21, 24, 20;...
        31, 27, 28, 32;... %front arm
        29, 32, 28, 25;...
        29, 25, 26, 30;...
        31, 27, 26, 30;...
        35, 39, 38, 34;... %back arm
        35, 36, 40, 39;...
        33, 36, 40, 37;...
        34, 37, 38, 34;...
        41, 42, 46, 45;... %perimeter
        45, 46, 44, 43;...
        43, 44, 48, 47;...
        47, 48, 42, 41;...
        49, 50, 51, 49;... %camera
        49, 50, 52, 49;...
        49, 52, 53, 49;...
        49, 51, 53, 49;...
      ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

  patchcolors = [...
    mygreen;... % left
    mygreen;... % right
    mygreen;... % back
    myred;... % front
    mycyan;... % top
    mycyan;... % bottom
    myblue;... %left arm
    myblue;...
    myblue;...
    myblue;...
    myblue;... %right arm
    myblue;...
    myblue;...
    myblue;...
    myyellow;... %front arm
    myyellow;...
    myyellow;...
    myyellow;...
    myblue;... %back arm
    myblue;...
    myblue;...
    myblue;... 
    myred;... %perimeter
    myred;...
    myred;...
    myred;...
    myyellow;...%camera
    myyellow;...
    myyellow;...
    myyellow;...
    ];
end

%==========================================================================
%Draw left pillar that have the color scheme for vision tracking on the
%   quadrotor.
%The center of each face is to be the radius distance from the center. 
%
%Try using fill3 or fill3d
%==========================================================================
function [Vp, Fp, patchcolorsp]=pillarVFCl(P)

% Define the vertices (physical location of vertices)
  Vp = [...
  -P.pillarposition+P.pillarradius P.pillaradd 0;... %1 left pillarlevel 0 points
  -P.pillarposition+P.pillaradd P.pillarradius 0;... %2
  -P.pillarposition-P.pillaradd P.pillarradius 0;... %3
  -P.pillarposition-P.pillarradius P.pillaradd 0;... %4
  -P.pillarposition-P.pillarradius -P.pillaradd 0;... %5
  -P.pillarposition-P.pillaradd -P.pillarradius 0;... %6
  -P.pillarposition+P.pillaradd -P.pillarradius 0;... %7
  -P.pillarposition+P.pillarradius -P.pillaradd 0;... %8
  -P.pillarposition+P.pillarradius P.pillaradd P.pillarc1;... %9 level 1 points
  -P.pillarposition+P.pillaradd P.pillarradius P.pillarc1;... %10
  -P.pillarposition-P.pillaradd P.pillarradius P.pillarc1;... %11
  -P.pillarposition-P.pillarradius P.pillaradd P.pillarc1;... %12
  -P.pillarposition-P.pillarradius -P.pillaradd P.pillarc1;... %13
  -P.pillarposition-P.pillaradd -P.pillarradius P.pillarc1;... %14
  -P.pillarposition+P.pillaradd -P.pillarradius P.pillarc1;... %15
  -P.pillarposition+P.pillarradius -P.pillaradd P.pillarc1;... %16
  -P.pillarposition+P.pillarradius P.pillaradd 2*P.pillarc1;... %17 level 2 points
  -P.pillarposition+P.pillaradd P.pillarradius 2*P.pillarc1;... %18
  -P.pillarposition-P.pillaradd P.pillarradius 2*P.pillarc1;... %19
  -P.pillarposition-P.pillarradius P.pillaradd 2*P.pillarc1;... %20
  -P.pillarposition-P.pillarradius -P.pillaradd 2*P.pillarc1;... %21
  -P.pillarposition-P.pillaradd -P.pillarradius 2*P.pillarc1;... %22
  -P.pillarposition+P.pillaradd -P.pillarradius 2*P.pillarc1;... %23
  -P.pillarposition+P.pillarradius -P.pillaradd 2*P.pillarc1;... %24
  -P.pillarposition+P.pillarradius P.pillaradd 3*P.pillarc1;... %25 level 3 points
  -P.pillarposition+P.pillaradd P.pillarradius 3*P.pillarc1;... %26
  -P.pillarposition-P.pillaradd P.pillarradius 3*P.pillarc1;... %27
  -P.pillarposition-P.pillarradius P.pillaradd 3*P.pillarc1;... %28
  -P.pillarposition-P.pillarradius -P.pillaradd 3*P.pillarc1;... %29
  -P.pillarposition-P.pillaradd -P.pillarradius 3*P.pillarc1;... %30
  -P.pillarposition+P.pillaradd -P.pillarradius 3*P.pillarc1;... %31
  -P.pillarposition+P.pillarradius -P.pillaradd 3*P.pillarc1;... %32
  -P.pillarposition+P.pillarradius P.pillaradd 3*P.pillarc1+P.pillarc2;... %33 level 4 points
  -P.pillarposition+P.pillaradd P.pillarradius 3*P.pillarc1+P.pillarc2;... %34
  -P.pillarposition-P.pillaradd P.pillarradius 3*P.pillarc1+P.pillarc2;... %35
  -P.pillarposition-P.pillarradius P.pillaradd 3*P.pillarc1+P.pillarc2;... %36
  -P.pillarposition-P.pillarradius -P.pillaradd 3*P.pillarc1+P.pillarc2;... %37
  -P.pillarposition-P.pillaradd -P.pillarradius 3*P.pillarc1+P.pillarc2;... %38
  -P.pillarposition+P.pillaradd -P.pillarradius 3*P.pillarc1+P.pillarc2;... %39
  -P.pillarposition+P.pillarradius -P.pillaradd 3*P.pillarc1+P.pillarc2;... %40
  -P.pillarposition+P.pillarradius P.pillaradd 4*P.pillarc1+P.pillarc2;... %41 level 5 points
  -P.pillarposition+P.pillaradd P.pillarradius 4*P.pillarc1+P.pillarc2;... %42
  -P.pillarposition-P.pillaradd P.pillarradius 4*P.pillarc1+P.pillarc2;... %43
  -P.pillarposition-P.pillarradius P.pillaradd 4*P.pillarc1+P.pillarc2;... %44
  -P.pillarposition-P.pillarradius -P.pillaradd 4*P.pillarc1+P.pillarc2;... %45
  -P.pillarposition-P.pillaradd -P.pillarradius 4*P.pillarc1+P.pillarc2;... %46
  -P.pillarposition+P.pillaradd -P.pillarradius 4*P.pillarc1+P.pillarc2;... %47
  -P.pillarposition+P.pillarradius -P.pillaradd 4*P.pillarc1+P.pillarc2;... %48
  -P.pillarposition+P.pillarradius P.pillaradd 5*P.pillarc1+P.pillarc2;... %49 level 6 points
  -P.pillarposition+P.pillaradd P.pillarradius 5*P.pillarc1+P.pillarc2;... %50
  -P.pillarposition-P.pillaradd P.pillarradius 5*P.pillarc1+P.pillarc2;... %51
  -P.pillarposition-P.pillarradius P.pillaradd 5*P.pillarc1+P.pillarc2;... %52
  -P.pillarposition-P.pillarradius -P.pillaradd 5*P.pillarc1+P.pillarc2;... %53
  -P.pillarposition-P.pillaradd -P.pillarradius 5*P.pillarc1+P.pillarc2;... %54
  -P.pillarposition+P.pillaradd -P.pillarradius 5*P.pillarc1+P.pillarc2;... %55
  -P.pillarposition+P.pillarradius -P.pillaradd 5*P.pillarc1+P.pillarc2;... %56
  -P.pillarposition+P.pillarradius P.pillaradd 6*P.pillarc1+P.pillarc2;... %57 level 7 points
  -P.pillarposition+P.pillaradd P.pillarradius 6*P.pillarc1+P.pillarc2;... %58
  -P.pillarposition-P.pillaradd P.pillarradius 6*P.pillarc1+P.pillarc2;... %59
  -P.pillarposition-P.pillarradius P.pillaradd 6*P.pillarc1+P.pillarc2;... %60
  -P.pillarposition-P.pillarradius -P.pillaradd 6*P.pillarc1+P.pillarc2;... %61
  -P.pillarposition-P.pillaradd -P.pillarradius 6*P.pillarc1+P.pillarc2;... %62
  -P.pillarposition+P.pillaradd -P.pillarradius 6*P.pillarc1+P.pillarc2;... %63
  -P.pillarposition+P.pillarradius -P.pillaradd 6*P.pillarc1+P.pillarc2;... %64
  -P.pillarposition+P.pillarradius P.pillaradd 6*P.pillarc1+2*P.pillarc2;... %65 level 8 points
  -P.pillarposition+P.pillaradd P.pillarradius 6*P.pillarc1+2*P.pillarc2;... %66
  -P.pillarposition-P.pillaradd P.pillarradius 6*P.pillarc1+2*P.pillarc2;... %67
  -P.pillarposition-P.pillarradius P.pillaradd 6*P.pillarc1+2*P.pillarc2;... %68
  -P.pillarposition-P.pillarradius -P.pillaradd 6*P.pillarc1+2*P.pillarc2;... %69
  -P.pillarposition-P.pillaradd -P.pillarradius 6*P.pillarc1+2*P.pillarc2;... %70
  -P.pillarposition+P.pillaradd -P.pillarradius 6*P.pillarc1+2*P.pillarc2;... %71
  -P.pillarposition+P.pillarradius -P.pillaradd 6*P.pillarc1+2*P.pillarc2;... %72
  ];

% define faces as a list of vertices numbered above
  Fp = [...
  1, 8, 16, 9;... %left pillar, bottom strip
  2, 1, 9, 10;...
  3, 2, 10, 11;...
  4, 3, 11, 12;...
  5, 4, 12, 13;...
  6, 5, 13, 14;...
  7, 6, 14, 15;...
  8, 7, 15, 16;...
  9, 16, 24, 17;...%second strip
  10, 9, 17, 18;...
  11, 10, 18, 19;...
  12, 11, 19, 20;...
  13, 12, 20, 21;...
  14, 13, 21, 22;...
  15, 14, 22, 23;...
  16, 15, 23, 24;...
  17, 24, 32, 25;...%third strip
  18, 17, 25, 26;...
  19, 18, 26, 27;...
  20, 19, 27, 28;...
  21, 20, 28, 29;...
  22, 21, 29, 30;...
  23, 22, 30, 31;...
  24, 23, 31, 32;...
  25, 32, 40, 33;...%fourth strip
  26, 25, 33, 34;...
  27, 26, 34, 35;...
  28, 27, 35, 36;...
  29, 28, 36, 37;... 
  30, 29, 37, 38;...
  31, 30, 38, 39;...
  32, 31, 39, 40;...
  33, 40, 48, 41;...%fifth strip
  34, 33, 41, 42;...
  35, 34, 42, 43;...
  36, 35, 43, 44;...
  37, 36, 44, 45;...
  38, 37, 45, 46;...
  39, 38, 46, 47;...
  40, 39, 47, 48;...
  41, 48, 56, 49;... %sixth strip
  42, 41, 49, 50;...
  43, 42, 50, 51;...
  44, 43, 51, 52;...
  45, 44, 52, 53;...
  46, 45, 53, 54;...
  47, 46, 54, 55;...
  48, 47, 55, 56;...
  49, 56, 64, 57;... %seventh strip
  50, 49, 57, 58;...
  51, 50, 58, 59;...
  52, 51, 59, 60;...
  53, 52, 60, 61;...
  54, 53, 61, 62;...
  55, 54, 62, 63;...
  56, 55, 63, 64;...
  57, 64, 72, 65;... %top strip
  58, 57, 65, 66;...
  59, 58, 66, 67;...
  60, 59, 67, 68;...
  61, 60, 68, 69;...
  62, 61, 69, 70;...
  63, 62, 70, 71;...
  64, 63, 71, 72;...
  ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mywhite = [1, 1, 1];

  patchcolorsp = [...
  myblue; myblue; myblue; myblue; myblue; myblue; myblue; myblue;... %left pillar, bottom strip
  myred; mygreen; mygreen; mygreen; mygreen; myred; myred; myred;... %second strip
  mygreen; myred; myred; myred; myred; mygreen; mygreen; mygreen;... %third strip
  mywhite; mywhite; mywhite; mywhite; mywhite; mywhite; mywhite; mywhite;... %fourth strip
  mygreen; myred; myred; myred; myred; mygreen; mygreen; mygreen;... %fifth strip
  myred; mygreen; mygreen; mygreen; mygreen; myred; myred; myred;... %sixth strip
  myblue; myblue; myblue; myblue; myblue; myblue; myblue; myblue;... %seventh strip
  myyellow; myyellow; myyellow; myyellow; myyellow; myyellow; myyellow; myyellow;... %top strip
  ];

end

%==========================================================================
%Draw right pillar that have the color scheme for vision tracking on the
%   quadrotor.
%The center of each face is to be the radius distance from the center. 
%
%Try using fill3 or fill3d
%==========================================================================
function [Vp, Fp, patchcolorsp]=pillarVFCr(P)

% Define the vertices (physical location of vertices)
  Vp = [...
  P.pillarposition+P.pillarradius P.pillaradd 0;... %1 left pillarlevel 0 points
  P.pillarposition+P.pillaradd P.pillarradius 0;... %2
  P.pillarposition-P.pillaradd P.pillarradius 0;... %3
  P.pillarposition-P.pillarradius P.pillaradd 0;... %4
  P.pillarposition-P.pillarradius -P.pillaradd 0;... %5
  P.pillarposition-P.pillaradd -P.pillarradius 0;... %6
  P.pillarposition+P.pillaradd -P.pillarradius 0;... %7
  P.pillarposition+P.pillarradius -P.pillaradd 0;... %8
  P.pillarposition+P.pillarradius P.pillaradd P.pillarc1;... %9 level 1 points
  P.pillarposition+P.pillaradd P.pillarradius P.pillarc1;... %10
  P.pillarposition-P.pillaradd P.pillarradius P.pillarc1;... %11
  P.pillarposition-P.pillarradius P.pillaradd P.pillarc1;... %12
  P.pillarposition-P.pillarradius -P.pillaradd P.pillarc1;... %13
  P.pillarposition-P.pillaradd -P.pillarradius P.pillarc1;... %14
  P.pillarposition+P.pillaradd -P.pillarradius P.pillarc1;... %15
  P.pillarposition+P.pillarradius -P.pillaradd P.pillarc1;... %16
  P.pillarposition+P.pillarradius P.pillaradd 2*P.pillarc1;... %17 level 2 points
  P.pillarposition+P.pillaradd P.pillarradius 2*P.pillarc1;... %18
  P.pillarposition-P.pillaradd P.pillarradius 2*P.pillarc1;... %19
  P.pillarposition-P.pillarradius P.pillaradd 2*P.pillarc1;... %20
  P.pillarposition-P.pillarradius -P.pillaradd 2*P.pillarc1;... %21
  P.pillarposition-P.pillaradd -P.pillarradius 2*P.pillarc1;... %22
  P.pillarposition+P.pillaradd -P.pillarradius 2*P.pillarc1;... %23
  P.pillarposition+P.pillarradius -P.pillaradd 2*P.pillarc1;... %24
  P.pillarposition+P.pillarradius P.pillaradd 3*P.pillarc1;... %25 level 3 points
  P.pillarposition+P.pillaradd P.pillarradius 3*P.pillarc1;... %26
  P.pillarposition-P.pillaradd P.pillarradius 3*P.pillarc1;... %27
  P.pillarposition-P.pillarradius P.pillaradd 3*P.pillarc1;... %28
  P.pillarposition-P.pillarradius -P.pillaradd 3*P.pillarc1;... %29
  P.pillarposition-P.pillaradd -P.pillarradius 3*P.pillarc1;... %30
  P.pillarposition+P.pillaradd -P.pillarradius 3*P.pillarc1;... %31
  P.pillarposition+P.pillarradius -P.pillaradd 3*P.pillarc1;... %32
  P.pillarposition+P.pillarradius P.pillaradd 3*P.pillarc1+P.pillarc2;... %33 level 4 points
  P.pillarposition+P.pillaradd P.pillarradius 3*P.pillarc1+P.pillarc2;... %34
  P.pillarposition-P.pillaradd P.pillarradius 3*P.pillarc1+P.pillarc2;... %35
  P.pillarposition-P.pillarradius P.pillaradd 3*P.pillarc1+P.pillarc2;... %36
  P.pillarposition-P.pillarradius -P.pillaradd 3*P.pillarc1+P.pillarc2;... %37
  P.pillarposition-P.pillaradd -P.pillarradius 3*P.pillarc1+P.pillarc2;... %38
  P.pillarposition+P.pillaradd -P.pillarradius 3*P.pillarc1+P.pillarc2;... %39
  P.pillarposition+P.pillarradius -P.pillaradd 3*P.pillarc1+P.pillarc2;... %40
  P.pillarposition+P.pillarradius P.pillaradd 4*P.pillarc1+P.pillarc2;... %41 level 5 points
  P.pillarposition+P.pillaradd P.pillarradius 4*P.pillarc1+P.pillarc2;... %42
  P.pillarposition-P.pillaradd P.pillarradius 4*P.pillarc1+P.pillarc2;... %43
  P.pillarposition-P.pillarradius P.pillaradd 4*P.pillarc1+P.pillarc2;... %44
  P.pillarposition-P.pillarradius -P.pillaradd 4*P.pillarc1+P.pillarc2;... %45
  P.pillarposition-P.pillaradd -P.pillarradius 4*P.pillarc1+P.pillarc2;... %46
  P.pillarposition+P.pillaradd -P.pillarradius 4*P.pillarc1+P.pillarc2;... %47
  P.pillarposition+P.pillarradius -P.pillaradd 4*P.pillarc1+P.pillarc2;... %48
  P.pillarposition+P.pillarradius P.pillaradd 5*P.pillarc1+P.pillarc2;... %49 level 6 points
  P.pillarposition+P.pillaradd P.pillarradius 5*P.pillarc1+P.pillarc2;... %50
  P.pillarposition-P.pillaradd P.pillarradius 5*P.pillarc1+P.pillarc2;... %51
  P.pillarposition-P.pillarradius P.pillaradd 5*P.pillarc1+P.pillarc2;... %52
  P.pillarposition-P.pillarradius -P.pillaradd 5*P.pillarc1+P.pillarc2;... %53
  P.pillarposition-P.pillaradd -P.pillarradius 5*P.pillarc1+P.pillarc2;... %54
  P.pillarposition+P.pillaradd -P.pillarradius 5*P.pillarc1+P.pillarc2;... %55
  P.pillarposition+P.pillarradius -P.pillaradd 5*P.pillarc1+P.pillarc2;... %56
  P.pillarposition+P.pillarradius P.pillaradd 6*P.pillarc1+P.pillarc2;... %57 level 7 points
  P.pillarposition+P.pillaradd P.pillarradius 6*P.pillarc1+P.pillarc2;... %58
  P.pillarposition-P.pillaradd P.pillarradius 6*P.pillarc1+P.pillarc2;... %59
  P.pillarposition-P.pillarradius P.pillaradd 6*P.pillarc1+P.pillarc2;... %60
  P.pillarposition-P.pillarradius -P.pillaradd 6*P.pillarc1+P.pillarc2;... %61
  P.pillarposition-P.pillaradd -P.pillarradius 6*P.pillarc1+P.pillarc2;... %62
  P.pillarposition+P.pillaradd -P.pillarradius 6*P.pillarc1+P.pillarc2;... %63
  P.pillarposition+P.pillarradius -P.pillaradd 6*P.pillarc1+P.pillarc2;... %64
  P.pillarposition+P.pillarradius P.pillaradd 6*P.pillarc1+2*P.pillarc2;... %65 level 8 points
  P.pillarposition+P.pillaradd P.pillarradius 6*P.pillarc1+2*P.pillarc2;... %66
  P.pillarposition-P.pillaradd P.pillarradius 6*P.pillarc1+2*P.pillarc2;... %67
  P.pillarposition-P.pillarradius P.pillaradd 6*P.pillarc1+2*P.pillarc2;... %68
  P.pillarposition-P.pillarradius -P.pillaradd 6*P.pillarc1+2*P.pillarc2;... %69
  P.pillarposition-P.pillaradd -P.pillarradius 6*P.pillarc1+2*P.pillarc2;... %70
  P.pillarposition+P.pillaradd -P.pillarradius 6*P.pillarc1+2*P.pillarc2;... %71
  P.pillarposition+P.pillarradius -P.pillaradd 6*P.pillarc1+2*P.pillarc2;... %72
  ];

% define faces as a list of vertices numbered above
  Fp = [...
  1, 8, 16, 9;... %left pillar, bottom strip
  2, 1, 9, 10;...
  3, 2, 10, 11;...
  4, 3, 11, 12;...
  5, 4, 12, 13;...
  6, 5, 13, 14;...
  7, 6, 14, 15;...
  8, 7, 15, 16;...
  9, 16, 24, 17;...%second strip
  10, 9, 17, 18;...
  11, 10, 18, 19;...
  12, 11, 19, 20;...
  13, 12, 20, 21;...
  14, 13, 21, 22;...
  15, 14, 22, 23;...
  16, 15, 23, 24;...
  17, 24, 32, 25;...%third strip
  18, 17, 25, 26;...
  19, 18, 26, 27;...
  20, 19, 27, 28;...
  21, 20, 28, 29;...
  22, 21, 29, 30;...
  23, 22, 30, 31;...
  24, 23, 31, 32;...
  25, 32, 40, 33;...%fourth strip
  26, 25, 33, 34;...
  27, 26, 34, 35;...
  28, 27, 35, 36;...
  29, 28, 36, 37;... 
  30, 29, 37, 38;...
  31, 30, 38, 39;...
  32, 31, 39, 40;...
  33, 40, 48, 41;...%fifth strip
  34, 33, 41, 42;...
  35, 34, 42, 43;...
  36, 35, 43, 44;...
  37, 36, 44, 45;...
  38, 37, 45, 46;...
  39, 38, 46, 47;...
  40, 39, 47, 48;...
  41, 48, 56, 49;... %sixth strip
  42, 41, 49, 50;...
  43, 42, 50, 51;...
  44, 43, 51, 52;...
  45, 44, 52, 53;...
  46, 45, 53, 54;...
  47, 46, 54, 55;...
  48, 47, 55, 56;...
  49, 56, 64, 57;... %seventh strip
  50, 49, 57, 58;...
  51, 50, 58, 59;...
  52, 51, 59, 60;...
  53, 52, 60, 61;...
  54, 53, 61, 62;...
  55, 54, 62, 63;...
  56, 55, 63, 64;...
  57, 64, 72, 65;... %top strip
  58, 57, 65, 66;...
  59, 58, 66, 67;...
  60, 59, 67, 68;...
  61, 60, 68, 69;...
  62, 61, 69, 70;...
  63, 62, 70, 71;...
  64, 63, 71, 72;...
  ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mywhite = [1, 1, 1];

  patchcolorsp = [...
  myred; myred; myred; myred; myred; myred; myred; myred;... %left pillar, bottom strip
  mygreen; myblue; myblue; myblue; myblue; mygreen; mygreen; mygreen;... %second strip
  myblue; mygreen; mygreen; mygreen; mygreen; myblue; myblue; myblue;... %third strip
  mywhite; mywhite; mywhite; mywhite; mywhite; mywhite; mywhite; mywhite;... %fourth strip
  myblue; mygreen; mygreen; mygreen; mygreen; myblue; myblue; myblue;... %fifth strip
  mygreen; myblue; myblue; myblue; myblue; mygreen; mygreen; mygreen;... %sixth strip
  myred; myred; myred; myred; myred; myred; myred; myred;... %seventh strip
  myyellow; myyellow; myyellow; myyellow; myyellow; myyellow; myyellow; myyellow;... %top strip
  ];

end

%%%%%%%%%%%%%%%%%%%%%%%not done
function Vert=rotateVert(Vert,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll'*R_pitch'*R_yaw';

  % rotate vertices
  Vert = (R*Vert')';
  
end % rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by column vector T
%not done
function Vert = translateVert(Vert, T)

  Vert = Vert + repmat(T', size(Vert,1),1);

end % translateVert
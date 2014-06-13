function plotquadrotor(states)

    % process inputs to function
    pn       = states(1);       % inertial North position     
    pe       = states(2);       % inertial East position
    h       = states(3);       % down position    
    u    = states(4);
    v    = states(5);
    w    = states(6);
    phi      = states(7);       % roll angle         
    theta    = states(8);       % pitch angle     
    psi      = states(9);       % yaw angle     
    p   = states(10);                
    q = states(11);           
    r   = states(12);
    t        = states(13);       %time
    
  
    % define persistent variables 
    persistent fig_quadrotor;
    
    
    % first time function is called, initialize plot and persistent vars
    if t<=0.1,
        figure(1), clf
        fig_quadrotor = drawquadrotor(pn,pe,h,phi,theta,psi, [],  'normal');
        hold on
        view(32,47);
        axis([-10,10,-10,10,-10,10]);
        xlabel('X axis');
        ylabel('Y axis');
        zlabel('Z axis');
        
    % at every other time step, redraw base and rod
    else 
        drawquadrotor(pn,pe,h,phi,theta,psi, fig_quadrotor);
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawquadrotor(pn,pe,h,phi,theta,psi, handle, mode)

% define points on spacecraft
  [V, F, patchcolors] = quadrotorVFC;
  [Vp, Fp, patchcolorsp] = propellerVFC;
  
  % rotate spacecraft
  V = rotateVert(V, phi, theta, psi);
  Vp = rotateVert(V, phi, theta, psi);
  % translate spacecraft
  V = translateVert(V, [pe; pn; h]);
  Vp = translateVert(V, [pe; pn; h]);
  
  if isempty(handle)
  figure(1), clf
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  
  else
    set(handle,'Vertices',V,'Faces',F);
    
    drawnow
  end
end
%
%=======================================================================
% quadrotorVFC
%=======================================================================
function [V,F,C]=quadrotorVFC

  V = [...
	1 1 0;... %1  box
    1 -1 0;... %2
    -1 -1 0;... %3
    -1 1 0;... %4
	1 1 2;... %5
    1 -1 2;... %6
    -1 -1 2;... %7
    -1 1 2;... %8
    1.9 1 1;... %9 %front propeller1
    2.1 1 1;... %10
    2.1 -1 1;...%11
    1.9 -1 1;...%12
    1.2, .1, 1;...%13 %front propeller2
    3, .1, 1;...%14
    3, -.1, 1;...%15
    1.2, -.1, 1;...%16
    -1.9 1 1;... %17 %back propeller1
    -2.1 1 1;... %18
    -2.1 -1 1;...%19
    -1.9 -1 1;...%20
    -1.2, .1, 1;...%21 %back propeller2
    -3, .1, 1;...%22
    -3, -.1, 1;...%23
    -1.2, -.1, 1;...%24
    -1, 2.1, 1;...%25% right propeller1
    -1, 1.9,  1;...%26
    1, 1.9, 1;...%27
    1, 2.1,  1;...%28
    -.1, 3, 1;...%29 right propeller2
    -.1, 1.2, 1;...%30
    .1, 1.2, 1;...%31
    .1, 3, 1;...%32
    -1, -2.1, 1;...%33 left propeller1
    -1, -1.9,  1;...%34
    1, -1.9, 1;...%35
    1, -2.1,  1;...%36
    -.1, -3, 1;...%37 left propeller2
    -.1, -1.2, 1;...%38
    .1, -1.2, 1;...%39
    .1, -3, 1;...%40
   ];

  F = [...
      1, 2, 6, 5;... % front
      4, 3, 7, 8;... % back
      1, 5, 8, 4;... % right 
      2, 6, 7, 3;... % left
      5, 6, 7, 8;... % top
      2, 1, 4, 3;...    %bottom
      9, 10, 11, 12;... %front propeller1
      13, 14, 15, 16;...%front propeller2
      17, 18, 19, 20;...%back propeller1
      21, 22, 23, 24;...%back propeller2
      25, 26, 27, 28;...%right propeller1
      29, 30, 31, 32;...%right propeller2
      33, 34, 35, 36;...%left propeller1
      37, 38, 39, 40;...%left propeller2
      
      ];
  


   % define colors for each face    
  
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myred = [1, 0, 0];
  myblack= [0, 0, 0];
  

  C = [...
    mygreen;... % front
    myblue;... % back
    myblue;... % right
    myblue;... % left
    myblack;... % top
    myblue;... % bottom
    mygreen; ... %front propeller
    mygreen; ... %front propeller
    myblack; ... %back propeller
    myblack; ... %back propeller
    myblack; ... %right propeller
    myblack; ... %right propeller
    myblack; ... %left propeller
    myblack; ... %left propeller
    ];
end

function [V,F,C]=propellerVFC

  V = [...
	1.9 1 1;... %1 %right propeller
    2.1 1 1;... %2
    2.1 -1 1;...%3
    1.9 -1 1;...%4
    1, .1, 1;...%5
    3, .1, 1;...%6
    3, -.1, 1;...%7
    1, -.1, 1;...%8

  ];

  F = [...
      1, 2, 3, 4;... %rightpropeller
      5, 6, 7, 8;...
      ];
  


   % define colors for each face    
  
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myred = [1, 0, 0];
  

  C = [...
    myblue; ... %right propeller
    ];
end
%%%%%%%%%%%%%%%%%%%%%%%
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
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by column vector T
function Vert = translateVert(Vert, T)

  Vert = Vert + repmat(T', size(Vert,1),1);

end % translateVert

  
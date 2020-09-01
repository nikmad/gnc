function drawAircraft(uu)

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

    % define persistent variables 
    persistent spacecraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        [Vertices, Faces, facecolors] = defineSpacecraftBody;
        spacecraft_handle = drawSpacecraftBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the view angle for figure
        axis([-500,500,-500,500,-500,500]);
        hold on
        grid on
        grid minor
        
    % at every other time step, redraw base and rod
    else 
        drawSpacecraftBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           spacecraft_handle);
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawSpacecraftBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V', phi, theta, psi)';  % rotate spacecraft
  V = translate(V', pn, pe, pd)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  if isempty(handle)
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat');
                 %'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)
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
  R = R_roll*R_pitch*R_yaw;
  % rotate vertices
  XYZ = R*XYZ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define spacecraft vertices and faces
function [V,F,colors] = defineSpacecraftBody()
    % Define the vertices (physical location of vertices
    fuse_l1=20;
    fuse_l2=10;
    fuse_l3=40;
    wing_l=15;
    fuse_h=10;
    tailwing_l=10;
    tail_h=10;
    fuse_w=10;
    wing_w=50;
    tailwing_w=30;


    V = [...
     	
     	fuse_l1			0			0		; % point 1
     	fuse_l2		fuse_w/2	-fuse_h/2	; % point 2
     	fuse_l2	   -fuse_w/2	-fuse_h/2	; % point 3
     	
     	fuse_l2	   -fuse_w/2	fuse_h/2	; % point 4
     	fuse_l2		fuse_w/2	fuse_h/2	; % point 5
     	-fuse_l3		0			0		; % point 6

     		0			wing_w/2	0		; % point 7
     	-wing_l			wing_w/2	0		; % point 8
     	-wing_l		   -wing_w/2	0		; % point 9
     		0		   -wing_w/2	0		; % point 10
     	
     	-(fuse_l3-tailwing_l)	tailwing_w/2	0; % point 11
     	-fuse_l3				tailwing_w/2	0; % point 12
     	-fuse_l3			   -tailwing_w/2	0; % point 13
     	-(fuse_l3-tailwing_l)  -tailwing_w/2	0; % point 14

     	-(fuse_l3-tailwing_l)  		0			0; % point 15
     	-fuse_l3 					0	  -tail_h; % point 16

    ];

    % define faces as a list of vertices numbered above
    F = [...
        
    	% Nose	
        1, 2, 3;
        1, 3, 4;
        1, 2, 5;
        1, 4, 5;
        
        % Fuselage
        2, 3, 6;
        3, 4, 6;
        2, 5, 6;
        4, 5, 6;

        % Wing
        %7, 8, 9, 10;
        7, 8, 9;
        7, 9, 10;

        % Horizontal tail
        %11, 12, 13, 14;
        11, 12, 13;
        11, 13, 14;

        % Vertical fin
        6, 15, 16;

        ];

    % define colors for each face    
%     myred = [1, 0, 0];
     mygreen = [0, 1, 0];
%     myblue = [0, 0, 1];
%     myyellow = [1, 1, 0];
%     mycyan = [0, 1, 1];


    colors = [...
        mygreen;
        mygreen;
        mygreen;
        mygreen;
        mygreen;
        mygreen;
        mygreen;
        mygreen;
        mygreen;
        mygreen;
        mygreen;
        mygreen;
        mygreen;
        ];
end
  
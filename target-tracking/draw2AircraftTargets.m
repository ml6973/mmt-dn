function draw2AircraftTargets(uu,V,F,colors,map,R_min,cam_fov,NOU,NOT,Vt,Ft)
 NOU=2;
pn       = zeros(1,NOU);       % inertial North position
pe       = zeros(1,NOU);       % inertial East position
pd       = zeros(1,NOU);       % inertial Down position
u        = zeros(1,NOU);       % body frame velocities
v        = zeros(1,NOU);
w        = zeros(1,NOU);
phi      = zeros(1,NOU);       % roll angle
theta    = zeros(1,NOU);      % pitch angle
psi      = zeros(1,NOU);       % yaw angle
p        = zeros(1,NOU);      % roll rate
q        = zeros(1,NOU);      % pitch rate
r        = zeros(1,NOU);
az       = zeros(1,NOU);
el       = zeros(1,NOU);
target   =zeros(3,NOT);
for i=1:NOU
    % process inputs to function
    pn(i)       = uu(14*(i-1)+1)+(i-1)*200;       % inertial North position
    pe(i)       = uu(14*(i-1)+2)+(i-1)*100;       % inertial East position
    pd(i)       = uu(14*(i-1)+3);       % inertial Down position
    u(i)        = uu(14*(i-1)+4);       % body frame velocities
    v(i)        = uu(14*(i-1)+5);
    w(i)        = uu(14*(i-1)+6);
    phi(i)      = uu(14*(i-1)+7);       % roll angle
    theta(i)    = uu(14*(i-1)+8);       % pitch angle
    psi(i)      = uu(14*(i-1)+9);       % yaw angle
    p(i)        = uu(14*(i-1)+10);      % roll rate
    q(i)        = uu(14*(i-1)+11);      % pitch rate
    r(i)        = uu(14*(i-1)+12);
    az(i)          = uu(14*(i-1)+13);       % gimbal azimuth angle
    el (i)         = uu(14*(i-1)+14);
end% yaw rate
t        = uu(14*NOU+1);
NN=14*NOU+1;
for i=1:NOT
    target(1,i)=uu(NN+6*(i-1)+1);
    target(2,i)=uu(NN+6*(i-1)+2);
    target(3,i)=uu(NN+6*(i-1)+3);
end
% time

% define persistent variables
persistent aircraft_handle;
persistent fov_handle;
 persistent target_handle;

% figure handle for UAV


% first time function is called, initialize plot and persistent vars

    
    if t==0,
        figure(1), clf
       
        aircraft_handle=zeros(1,NOU);
        fov_handle=zeros(1,NOU);
        target_handle=zeros(1,NOT);
        for i=1:NOU
            
        aircraft_handle(i) = drawBody(V,F,colors,...
            pn(i),pe(i),pd(i),phi(i),theta(i),psi(i),...
            [], 'normal');
         hold on 
          fov_handle(i) = drawFov(pn(i), pe(i), pd(i), phi(i), theta(i), psi(i),az(i),el(i),cam_fov,map,[],'normal');
%         title('UAV')
%         xlabel('East')
%         ylabel('North')
%         zlabel('-Down')
%         view(32,47)  % set the vieew angle for figure
%         axis([-1000,1000,-1000,1000,-.1,  1000]);
%         grid on

       drawMap(map);% uncoment this to draw map and buildings
        title('UAV')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        axis([-map.width/5,map.width,-map.width/5,map.width,0,5*map.MaxHeight]);
        view(-40,70)  % set the view angle for figure
        grid on
        end
        for i=1:NOT
            target_handle(i) = drawTarget(target(:,i), map.StreetWidth/12, [], 'normal');
        end
      
        % at every other time step, redraw quadrotor and target
    else
        for i=1:NOU
        drawBody(V,F,colors,...
            pn(i),pe(i),pd(i),phi(i),theta(i),psi(i),...
            aircraft_handle(i));
        drawFov(pn(i), pe(i), pd(i), phi(i), theta(i), psi(i),az(i),el(i),cam_fov,map,fov_handle(i));

        end
        for i=1:NOT
             drawTarget(target(:,i), map.StreetWidth/12, target_handle(i));
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawBody(V,F,colors,...
    pn, pe, pd, phi, theta, psi,...
    handle, mode)
V = rotate(V', phi, theta, psi)';  % rotate rigid body
V = translate(V', pn, pe, pd)';  % translate after rotation

% transform vertices from NED to XYZ (for matlab rendering)
R = [...
    0, 1, 0;...
    1, 0, 0;...
    0, 0, -1;...
    ];
V = V*R;

if isempty(handle),
    handle = patch('Vertices', V, 'Faces', F,...
        'FaceVertexCData',colors,...
        'FaceColor','flat',...
        'EraseMode', mode);
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

% rotate vertices
%XYZ = R_yaw*R_pitch*R_roll*XYZ;
XYZ  = R_roll*R_pitch*R_yaw*XYZ;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)

XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));

end

function handle = drawFov(pn, pe, pd, phi, theta, psi,az,el,cam_fov,map,handle, mode)
                           

    %-------vertices and faces for camera field-of-view --------------
    % vertices
    % define unit vectors along fov in the camera gimbal frame
    pts = [...
        cos(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2);...
        cos(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2);...
        cos(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2);...
        cos(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2);...
        ]';
    % transform from gimbal coordinates to the vehicle coordinates
    pts = Rot_v_to_b(phi,theta,psi)'*Rot_b_to_g(az,el)'*pts;

    % first vertex is at center of MAV vehicle frame
    Vert = [pn, pe, pd];  
    % project field of view lines onto ground plane and make correction
    % when the projection is above the horizon
    for i=1:4,
        % alpha is the angle that the field-of-view line makes with horizon
        alpha = atan2(pts(3,i),norm(pts(1:2,i)));
        if alpha > 0,
            % this is the normal case when the field-of-view line
            % intersects ground plane
            Vert = [...
                Vert;...
                [pn-pd*pts(1,i)/pts(3,i), pe-pd*pts(2,i)/pts(3,i), 0];...
                ];
        else
            % this is when the field-of-view line is above the horizon.  In
            % this case, extend to a finite, but far away (9999) location.
            Vert = [...
                Vert;...
                [pn+9999*pts(1,i), pe+9999*pts(2,i),0];...
            ];
        end
    end

    Faces = [...
          1, 1, 2, 2;... % x-y face
          1, 1, 3, 3;... % x-y face
          1, 1, 4, 4;... % x-y face
          1, 1, 5, 5;... % x-y face
          2, 3, 4, 5;... % x-y face
        ];

    edgecolor      = [1, 1, 1]; % black
    footprintcolor = [0,0,0];%[1,0,1];%[1,1,0];
    colors = [edgecolor; edgecolor; edgecolor; edgecolor; footprintcolor];  

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  Vert = Vert*R;

  if isempty(handle),
    handle = patch('Vertices', Vert, 'Faces', Faces,...
                 'FaceVertexCData',colors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',Vert,'Faces',Faces);
  end
  
end 

function R = Rot_v_to_b(phi,theta,psi);
% Rotation matrix from body coordinates to vehicle coordinates

Rot_v_to_v1 = [...
    cos(psi), sin(psi), 0;...
    -sin(psi), cos(psi), 0;...
    0, 0, 1;...
    ];
    
Rot_v1_to_v2 = [...
    cos(theta), 0, -sin(theta);...
    0, 1, 0;...
    sin(theta), 0, cos(theta);...
    ];
    
Rot_v2_to_b = [...
    1, 0, 0;...
    0, cos(phi), sin(phi);...
    0, -sin(phi), cos(phi);...
    ];
    
R = Rot_v2_to_b * Rot_v1_to_v2 * Rot_v_to_v1;

end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_b_to_g(az,el);
% Rotation matrix from body coordinates to gimbal coordinates
Rot_b_to_g1 = [...
    cos(az), sin(az), 0;...
    -sin(az), cos(az), 0;...
    0, 0, 1;...
    ];

Rot_g1_to_g = [...
    cos(el), 0, -sin(el);...
    0, 1, 0;...
    sin(el), 0, cos(el);...
    ];

R = Rot_g1_to_g * Rot_b_to_g1;

end
function drawMap(map)%,path,smoothedPath,tree,R_min)
  
 
  % draw buildings 
  V = [];
  F = [];
  patchcolors = [];
  count = 0;
  for i=1:map.NumBlocks,
      for j=1:map.NumBlocks,
        [Vtemp,Ftemp,patchcolorstemp] = buildingVertFace(map.buildings_n(i),...
            map.buildings_e(j),map.BuildingWidth,map.heights(j,i));
        V = [V; Vtemp];
        Ftemp = Ftemp + count;
        F = [F; Ftemp];
        count = count + 8;
        patchcolors = [patchcolors;patchcolorstemp];
      end
  end
  
  patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat');
 

end
function [V,F,patchcolors] = buildingVertFace(n,e,width,height)
 
  % vertices of the building
  V = [...
        e+width/2, n+width/2, 0;...
        e+width/2, n-width/2, 0;...
        e-width/2, n-width/2, 0;...
        e-width/2, n+width/2, 0;...
        e+width/2, n+width/2, height;...
        e+width/2, n-width/2, height;...
        e-width/2, n-width/2, height;...
        e-width/2, n+width/2, height;...
        ];    
  % define faces of fuselage
  F = [...
        1, 4, 8, 5;... % North Side
        1, 2, 6, 5;... % East Side
        2, 3, 7, 6;... % South Side
        3, 4, 8, 7;... % West Side
        5, 6, 7, 8;... % Top
        ];   

  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1,1,0];
  mymagenta   = [0, 1, 1];

  patchcolors = [...
    mygreen;... % North
    mygreen;... % East
    mygreen;... % South
    mygreen;... % West
    myyellow;...  % Top
    ];

end
function handle=drawTarget(z, R, handle, mode);

  th = 0:.1:2*pi;
  X = z(1)+ R*cos(th);
  Y = z(2)+ R*sin(th);
  Z = z(3)*ones(length(th));
  
  if isempty(handle),
    handle = fill(Y, X, 'r', 'EraseMode', mode);
  else
    set(handle,'XData',Y,'YData',X);
  end
  
end
% geolocation
%
% compute location of target given position in camera
% input is 
%    uu(1:3)   - camera data (eps_x, eps_y, eps_s)
%    uu(4:15)  - MAV states
%    uu(16:17) - gimbal azimuth, elevation 
%    uu(18)    - time
%
% output is 
%    tn - estimated North postion of target
%    te - estimated East position of target
%    L  - range to target
%
% 
%
function out = geolocationMultiTargetGimCont2(in,P,nt)

    % process inputs
    eps_x =zeros(1,nt);
    eps_y =zeros(1,nt); % y-pixel
    eps_s =zeros(1,nt);
    out=zeros(5*nt+25*nt,1);
    NN = 0;
    for i=1:nt
    eps_x (i)    = in(3*(i-1)+1+NN); % x-pixel
    eps_y (i)    = in(3*(i-1)+2+NN); % y-pixel
    eps_s (i)    = in(3*(i-1)+3+NN); % pixel size
    end
    NN = NN + 3*nt;
    
    pn        = in(1+NN);
    pe        = in(2+NN);
    pd        = -in(3+NN);
    % Va      = in(4+NN);
    % alpha   = in(5+NN);
    % beta    = in(6+NN);
    phi       = in(7+NN);
    theta     = in(8+NN);
    chi       = in(9+NN);
    % p       = in(10+NN);
    % q       = in(11+NN);
    % r       = in(12+NN);
    Vg        = in(13+NN);
    % wn      = in(14+NN);
    % we      = in(15+NN);
    psi     = in(16+NN);
    NN = NN + 16;
    az        = in(1+NN); % gimbal azimuth angle
    el        = in(2+NN); % gimbal elevation angle
    NN = NN + 2;
    t         = in(1+NN); % time

    
    %--------------------------------------
    % begin geolocation code 
    
    
    % define persistent variables
    persistent xhat_t       % estimate of tn and te
    persistent P_t          % error covariance for tn and te

    if t==0,
        xhat_t=zeros(5,nt);
        P_t= zeros(5,5,nt);
        for i=1:nt
       xhat_t (:,i)      = [pn+10*randn(1); pe+10*randn(1);-pd+10*randn(1);0;0]; % initial estimate is position of MAV
        P_t (:,:,i)         = 100*eye(5);
        end% assume 100 m error
    end
    
    %-------------------------------------------------------------------
    % implement continous-discrete EKF to estimate roll and pitch angles
    %-------------------------------------------------------------------
    % implement continous-discrete EKF to estimate roll and pitch angles
    Ts=P.Tc;
    Q_t =1* diag([0.0001,0.0001,0.0001,0.1,0.1]);
    R_t = diag([P.sigma_measurement_n^2, P.sigma_measurement_e^2, P.sigma_measurement_h^2]);
    for i=1:nt
    N = 10;
    % prediction step
    for j=1:N,
        p_mav_dot = [Vg*cos(chi); Vg*sin(chi); 0];
        f_t = [...
            xhat_t(4,i);...
            xhat_t(5,i);...
            [xhat_t(1,i)-pn; xhat_t(2,i)-pe; pd]'*([xhat_t(4,i);xhat_t(5,i); 0]-p_mav_dot)/xhat_t(3,i);...
            0;...
            0 ];
        A_t = [...
            0, 0, 0, 1, 0;...
            0, 0, 0, 0, 1;...
           (xhat_t(4,i)-p_mav_dot(1))/xhat_t(3,i), (xhat_t(5,i)-p_mav_dot(2))/xhat_t(3,i),  -[xhat_t(1,i)-pn; xhat_t(2,i)-pe; pd]'*([xhat_t(4,i);xhat_t(5,i); 0]-p_mav_dot)/(xhat_t(3,i))^2, (xhat_t(1,i)-pn)/xhat_t(3,i), (xhat_t(2,i)-pe)/xhat_t(3,i);...
           0, 0, 0, 0, 0;...
           0, 0, 0, 0, 0];
        %A_t=eye(5)+P.Ts*A_t;
        xhat_t(:,i) = xhat_t(:,i) + (Ts/N)*f_t;
        P_t(:,:,i) = P_t(:,:,i) + (Ts/N)*(A_t*P_t(:,:,i) + P_t(:,:,i)*A_t' + Q_t);
       %P_t(:,:,i)=A_t*P_t(:,:,i)*A_t'+(P.Ts/N)^2*Q_t;
    end
%     Ybar=P_t(1:3,1:3,i)^-1 ;
    if (eps_x(i)~=-9999)&(eps_y(i)~=-9999), % don't update when out of the camera FOV
        % measurement updates
        ell_c = [eps_x(i); eps_y(i); P.f]/sqrt(eps_x(i)^2+eps_y(i)^2+P.f^2);
        R_b_i = Rot_v_to_b(phi,theta,psi)';
        R_g_b = Rot_b_to_g(az,el)';
        R_c_g = [...                        % camera to gimbal
        0, 1, 0;...
        0, 0, 1;...
        1, 0, 0]';
        ell_i = R_b_i*R_g_b*R_c_g*ell_c;
        h = [xhat_t(1,i); xhat_t(2,i); 0] - xhat_t(3,i)*ell_i;
        H = [[1,0;0,1;0,0], -ell_i, zeros(3,2)];
%         InfoMat1=H'*H
%         NNoise=R_t^-1
%         Infomat2=H'*R_t^-1*H
        % North MAV position
        L_t = P_t(:,:,i)*H(1,:)'/(R_t(1,1)+H(1,:)*P_t(:,:,i)*H(1,:)');
        P_t(:,:,i) = (eye(5)-L_t*H(1,:))*P_t(:,:,i);
        xhat_t(:,i) = xhat_t(:,i) + L_t*(pn - h(1));
        % East MAV position
        L_t = P_t(:,:,i)*H(2,:)'/(R_t(2,2)+H(2,:)*P_t(:,:,i)*H(2,:)');
        P_t(:,:,i) = (eye(5)-L_t*H(2,:))*P_t(:,:,i);
        xhat_t(:,i) = xhat_t(:,i) + L_t*(pe - h(2));
        % Down MAV position
        L_t = P_t(:,:,i)*H(3,:)'/(R_t(3,3)+H(3,:)*P_t(:,:,i)*H(3,:)');
        P_t(:,:,i) = (eye(5)-L_t*H(3,:))*P_t(:,:,i);
        xhat_t(:,i) = xhat_t(:,i) + L_t*(pd - h(3));
        
%    YYYn= P_t(1:3,1:3,i)^-1 
    end
    tn   = xhat_t(1,i);
    te   = xhat_t(2,i);
    L    = xhat_t(3,i);
    vx   = xhat_t(4,i);
    vy   = xhat_t(5,i);
    
    
    
    
    % create output
      Pvec=reshape(P_t(:,:,i),25,1);
    out(30*(i-1)+1:30*(i-1)+30) = [tn; te; L;vx;vy;Pvec];
    end
    
% end geolocation code 
    %-------------------------------------- 
end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_v_to_b(phi,theta,psi)
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
function R = Rot_b_to_g(az,el)
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

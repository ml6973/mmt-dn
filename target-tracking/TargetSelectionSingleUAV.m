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
function out = TargetSelectionSingleUAV(in,P,nt,nu,k)
%  % create output
%       Pvec=reshape(P_t(:,:,i),25,1);
%     out(30*(i-1)+1:30*(i-1)+30) = [tn; te; L;vx;vy;Pvec];
    % process inputs
Ts=P.Tg;
    NN = 0;
    xhat_t=zeros(5,nt);
    P_t= zeros(5,5,nt);
        for i=1:nt
       xhat_t (:,i)      = in(NN+30*(i-1)+1:NN+30*(i-1)+5); % initial estimate is position of MAV
       Pvec=in(NN+30*(i-1)+6:NN+30*(i-1)+30);
        P_t (:,:,i)         = reshape(Pvec,5,5);
        end
        NN=NN+30*nt;
    
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
     %-------------------------------------------------------------------
    % implement continous-discrete EKF to estimate roll and pitch angles
   Q_t =0.1* diag([1,1,1,1,1]);
    R_t = diag([P.sigma_measurement_n, P.sigma_measurement_e, P.sigma_measurement_h]);
    J=zeros(nt,1);
    uinp=eye(nt);
   
    for tt=1:nt
    Ji=1;
%     Ji=1;
     for i=1:nt
         N = 10;
    % prediction step
        xu=[pn;pe;pd];
    for j=1:N,
        p_mav_dot = [Vg*cos(chi); Vg*sin(chi); 0];
        xu=xu+p_mav_dot*(Ts/N);
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
        A_t=eye(5)+(Ts/N)*A_t;
         xhat_t(:,i) = xhat_t(:,i) + (Ts/N)*f_t;
         %Q_t =1* diag([0.001,0.001,0.001,0.1,0.1]);
         P_t(:,:,i) = P_t(:,:,i) + (P.Ts/N)*(A_t*P_t(:,:,i) + P_t(:,:,i)*A_t' + Q_t);
        %P_t(:,:,i) = (A_t*P_t(:,:,i)*A_t') + Q_t*(P.Ts/N)^2;
    end
%     
    ell_i=[xhat_t(1:2,i);0]-xu;
    Ri=sqrt(ell_i'*ell_i);
    ell_i=ell_i/Ri;
    H = [[1,0;0,1;0,0], -ell_i, zeros(3,2)];
    Ji=Ji*det(P_t(:,:,i)^(-1)+uinp(tt,i)*(H'*R_t^(-1)*H));
   % Ji=Ji+det(P_t(:,:,i)^(-1)+uinp(tt,i)*(H'*R_t^(-1)*H));
   % Ji=Ji+det(P_t(:,:,i))*uinp(tt,i)*det(H'*R_t^(-1)*H);
     end
    J(tt)=Ji;
    end
%     J;
    [mymin,out]=max(J);
  
end
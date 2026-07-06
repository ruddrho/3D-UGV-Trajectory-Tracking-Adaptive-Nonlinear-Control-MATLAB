%% 3D UGV Trajectory Tracking Using Adaptive Nonlinear Control
% Complete MATLAB simulation file
% Model: bicycle/unicycle-type UGV tracking a 3D reference trajectory
% Author: Generated for academic simulation use

clc; clear; close all;

%% Simulation settings
dt = 0.01;                 % sampling time (s)
T  = 35;                   % total simulation time (s)
t  = 0:dt:T;
N  = length(t);

%% UGV parameters
L  = 0.55;                 % wheel base (m)
vmax = 1.5;                % maximum linear velocity (m/s)
wmax = 2.5;                % maximum yaw rate (rad/s)

%% Controller gains
kx = 1.60;                 % longitudinal error gain
ky = 3.20;                 % lateral error gain
kpsi = 2.60;               % heading error gain
kz = 1.80;                 % height tracking gain for 3D display/control

% Adaptive gain parameters
Gamma = diag([0.35 0.35 0.20 0.15]);  % adaptation rate
sigma = 0.05;                         % leakage term for bounded estimates
ahat = zeros(4,N);                    % estimated uncertainty parameters

%% Initial UGV state
% state = [x; y; z; psi]
x = zeros(4,N);
x(:,1) = [4.8; -0.8; 0.0; pi/2];

%% Reference trajectory allocation
xr = zeros(1,N);
yr = zeros(1,N);
zr = zeros(1,N);
psir = zeros(1,N);
vr = zeros(1,N);
wr = zeros(1,N);

%% Generate 3D reference trajectory
% A smooth circular/elliptical path with small vertical variation
R1 = 4.0;
R2 = 3.0;
omega = 0.22;
z0 = 0.50;
zamp = 0.25;

for k = 1:N
    xr(k) = R1*cos(omega*t(k));
    yr(k) = R2*sin(omega*t(k));
    zr(k) = z0 + zamp*sin(0.5*omega*t(k));
end

% Reference velocities and heading
xrdot = gradient(xr,dt);
yrdot = gradient(yr,dt);
zrdot = gradient(zr,dt);
psir = atan2(yrdot,xrdot);
vr = sqrt(xrdot.^2 + yrdot.^2);
wr = gradient(unwrap(psir),dt);

%% Data storage
u_v = zeros(1,N);          % actual linear velocity command
u_w = zeros(1,N);          % yaw-rate command
u_z = zeros(1,N);          % vertical command for 3D path display
ex_hist = zeros(1,N);
ey_hist = zeros(1,N);
ez_hist = zeros(1,N);
epsi_hist = zeros(1,N);
tracking_error = zeros(1,N);

%% Main simulation loop
for k = 1:N-1
    % Current states
    X = x(1,k); Y = x(2,k); Z = x(3,k); psi = x(4,k);
    
    % Position errors in inertial frame
    dx = xr(k) - X;
    dy = yr(k) - Y;
    dz = zr(k) - Z;
    
    % Transform errors to robot body frame
    ex =  cos(psi)*dx + sin(psi)*dy;
    ey = -sin(psi)*dx + cos(psi)*dy;
    ez = dz;
    epsi = wrapToPiLocal(psir(k) - psi);
    
    % Save errors
    ex_hist(k) = ex;
    ey_hist(k) = ey;
    ez_hist(k) = ez;
    epsi_hist(k) = epsi;
    tracking_error(k) = sqrt(dx^2 + dy^2 + dz^2);
    
    % Regressor for adaptive uncertainty compensation
    phi = [ex; ey; epsi; ez];
    uncertainty = ahat(:,k)'*phi;
    
    % Adaptive nonlinear controller
    v_cmd = vr(k)*cos(epsi) + kx*ex + uncertainty;
    w_cmd = wr(k) + ky*vr(k)*ey + kpsi*sin(epsi);
    z_cmd = zrdot(k) + kz*ez;
    
    % Saturation
    v_cmd = min(max(v_cmd,-vmax),vmax);
    w_cmd = min(max(w_cmd,-wmax),wmax);
    
    % Update adaptive parameters
    error_vector = [ex; ey; epsi; ez];
    ahat_dot = Gamma*error_vector - sigma*ahat(:,k);
    ahat(:,k+1) = ahat(:,k) + ahat_dot*dt;
    
    % UGV kinematics with small bounded disturbance
    d_v = 0.03*sin(0.7*t(k));
    d_w = 0.02*cos(0.5*t(k));
    
    Xdot = (v_cmd + d_v)*cos(psi);
    Ydot = (v_cmd + d_v)*sin(psi);
    Zdot = z_cmd;
    psidot = w_cmd + d_w;
    
    % Euler integration
    x(1,k+1) = X + Xdot*dt;
    x(2,k+1) = Y + Ydot*dt;
    x(3,k+1) = Z + Zdot*dt;
    x(4,k+1) = wrapToPiLocal(psi + psidot*dt);
    
    u_v(k) = v_cmd;
    u_w(k) = w_cmd;
    u_z(k) = z_cmd;
end

% final error values
ex_hist(N) = ex_hist(N-1);
ey_hist(N) = ey_hist(N-1);
ez_hist(N) = ez_hist(N-1);
epsi_hist(N) = epsi_hist(N-1);
tracking_error(N) = tracking_error(N-1);
u_v(N) = u_v(N-1);
u_w(N) = u_w(N-1);
u_z(N) = u_z(N-1);

%% Performance indices
RMSE = sqrt(mean(tracking_error.^2));
MAXE = max(tracking_error);
SS_error = mean(tracking_error(round(0.8*N):end));

fprintf('3D UGV Adaptive Nonlinear Trajectory Tracking Results\n');
fprintf('RMSE tracking error      = %.4f m\n',RMSE);
fprintf('Maximum tracking error   = %.4f m\n',MAXE);
fprintf('Steady-state mean error  = %.4f m\n',SS_error);

%% Figure 1: 3D trajectory tracking
figure('Color','w','Position',[100 100 900 650]);
plot3(xr,yr,zr,'r--','LineWidth',2); hold on;
plot3(x(1,:),x(2,:),x(3,:),'b','LineWidth',2);
plot3(x(1,1),x(2,1),x(3,1),'go','MarkerSize',8,'MarkerFaceColor','g');
plot3(x(1,end),x(2,end),x(3,end),'ko','MarkerSize',8,'MarkerFaceColor','k');
grid on; box on;
xlabel('x (m)','FontSize',12);
ylabel('y (m)','FontSize',12);
zlabel('z (m)','FontSize',12);
title('3D UGV Trajectory Tracking Using Adaptive Nonlinear Control','FontSize',13);
legend('Reference trajectory','UGV actual trajectory','Start','End','Location','best');
view(45,25);
axis equal;

%% Figure 2: 2D top view
figure('Color','w','Position',[150 120 900 650]);
plot(xr,yr,'r--','LineWidth',2); hold on;
plot(x(1,:),x(2,:),'b','LineWidth',2);
grid on; box on; axis equal;
xlabel('x (m)','FontSize',12);
ylabel('y (m)','FontSize',12);
title('Top View of UGV Path Tracking','FontSize',13);
legend('Reference path','Actual UGV path','Location','best');

%% Figure 3: Tracking errors
figure('Color','w','Position',[200 140 900 650]);
plot(t,ex_hist,'LineWidth',1.8); hold on;
plot(t,ey_hist,'LineWidth',1.8);
plot(t,ez_hist,'LineWidth',1.8);
plot(t,epsi_hist,'LineWidth',1.8);
grid on; box on;
xlabel('Time (s)','FontSize',12);
ylabel('Error','FontSize',12);
title('Body-Frame Tracking Errors','FontSize',13);
legend('e_x (m)','e_y (m)','e_z (m)','e_\psi (rad)','Location','best');

%% Figure 4: Total tracking error
figure('Color','w','Position',[250 160 900 650]);
plot(t,tracking_error,'k','LineWidth',2);
grid on; box on;
xlabel('Time (s)','FontSize',12);
ylabel('Tracking error (m)','FontSize',12);
title('Total 3D Position Tracking Error','FontSize',13);
legend('||e||','Location','best');

%% Figure 5: Control signals
figure('Color','w','Position',[300 180 900 650]);
plot(t,u_v,'LineWidth',1.8); hold on;
plot(t,u_w,'LineWidth',1.8);
plot(t,u_z,'LineWidth',1.8);
grid on; box on;
xlabel('Time (s)','FontSize',12);
ylabel('Control input','FontSize',12);
title('Adaptive Nonlinear Controller Outputs','FontSize',13);
legend('Linear velocity v (m/s)','Yaw rate \omega (rad/s)','Vertical command v_z (m/s)','Location','best');

%% Figure 6: Adaptive parameter estimates
figure('Color','w','Position',[350 200 900 650]);
plot(t,ahat(1,:),'LineWidth',1.8); hold on;
plot(t,ahat(2,:),'LineWidth',1.8);
plot(t,ahat(3,:),'LineWidth',1.8);
plot(t,ahat(4,:),'LineWidth',1.8);
grid on; box on;
xlabel('Time (s)','FontSize',12);
ylabel('Adaptive parameter estimate','FontSize',12);
title('Adaptive Gain/Uncertainty Parameter Estimates','FontSize',13);
legend({'$\hat{a}_1$','$\hat{a}_2$','$\hat{a}_3$','$\hat{a}_4$'},'Interpreter','latex','Location','best');

%% Simple UGV animation in 3D
figure('Color','w','Position',[400 220 900 650]);
for k = 1:80:N
    clf;
    plot3(xr,yr,zr,'r--','LineWidth',2); hold on;
    plot3(x(1,1:k),x(2,1:k),x(3,1:k),'b','LineWidth',2);
    drawUGV3D(x(1,k),x(2,k),x(3,k),x(4,k),0.55,0.35);
    grid on; box on;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title('3D UGV Trajectory Tracking Animation');
    legend('Reference','Tracked path','Location','best');
    axis equal;
    xlim([min(xr)-1 max(xr)+1]);
    ylim([min(yr)-1 max(yr)+1]);
    zlim([0 max(zr)+0.8]);
    view(45,25);
    drawnow;
end

%% Local functions
function ang = wrapToPiLocal(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end

function drawUGV3D(xc,yc,zc,psi,L,W)
    % Draw a simple rectangular UGV body with heading direction
    R = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    body = [ L/2  W/2;
             L/2 -W/2;
            -L/2 -W/2;
            -L/2  W/2]';
    pts = R*body + [xc; yc];
    fill3(pts(1,:),pts(2,:),zc*ones(1,4),'g','FaceAlpha',0.75,'EdgeColor','k','LineWidth',1.2);
    % wheels
    wheelOffsets = [ L/3  W/2; L/3 -W/2; -L/3 W/2; -L/3 -W/2]';
    wp = R*wheelOffsets + [xc; yc];
    plot3(wp(1,:),wp(2,:),zc*ones(1,4),'ko','MarkerFaceColor','k','MarkerSize',6);
    % heading arrow
    nose = R*[L/2+0.35;0] + [xc;yc];
    plot3([xc nose(1)],[yc nose(2)],[zc zc],'k','LineWidth',2);
end

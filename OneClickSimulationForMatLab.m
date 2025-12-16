function OneClickSimulationForMatLab()
    % hoverMMAControlXYPID – Full PID+MMA hover control with PID outer loops on x,y
    %
    % Four inner PID loops (altitude, roll, pitch, yaw) plus two PID outer loops
    % on x and y position to generate dynamic attitude references.
    % Motor Mixing Algorithm inverts [U1;U2;U3;U4] → rotor speeds.
    %
    % Usage:
    %   1) Save this file as hoverMMAControlXYPID.m
    %   2) Ensure quadNonlinearDynamics.m & loadQuadParams.m are on your path
    %   3) In MATLAB’s Command Window, type:
    %        hoverMMAControlXYPID

    %% 1) Load params & hover speed
    params      = loadQuadParams();
    kF          = params.kF;
    l           = params.l;
    B           = params.B;
    m           = params.m;
    g           = params.g;
    omega_hover = sqrt(m * g / (4*kF));
    
    duration = 10;                          %change simulation time

    %% 2) Inner-loop PID gains

    Kp_z     =  8.6;   Ki_z     = 10;    Kd_z     = 5.16;    % altitude U1

    Kp_phi   = 0.009;    Ki_phi   =0.0003;    Kd_phi   =0.001;  % roll     U2

    Kp_theta = 0.009;  Ki_theta =0.0003;    Kd_theta =0.001;   % pitch    U3

    Kp_psi   = 0.003;   Ki_psi   =0.0004;    Kd_psi   =1.8e-3; % yaw      U4


    %% 3) Outer-loop PID gains
    Kp_x   = 5;  Ki_x   = 2;  Kd_x   = 3.8;

    Kp_y   = Kp_x;  Ki_y   = Ki_x;  Kd_y   = Kd_x;

    %% 4) References

    % Change reference to change where the drone settles. Currently it
    % starts at 0 0 0 and then must move to 1 1 1 as you will see in the
    % initial conditions to follow. 

    x_ref     = 1;
    y_ref     = 1;
    z_ref     = 1;

    psi_ref   = 0;

    %% 5) Precompute mixer inverse

    M = [  kF,     kF,     kF,    kF;
           0,  -kF*l,      0,  kF*l;
        -kF*l,     0,   kF*l,      0;
          -B,      B,     -B,      B ];
    Minv = inv(M);

    %% 6) Initial conditions (12 states + 6 integrators)

   % ['x','y','z','ẋ','ẏ','ż','φ','θ','ψ','p','q','r']

   % Change initial conditions to change response. 

    x0 = zeros(12,1);

    % Position (m)
    x0(1) = 0;                   % x
    x0(2) = 0;                   % y
    x0(3) = 0;                   % z
    
    % Velocity (m/s)
    x0(4) = 1;                   % ẋ
    x0(5) = 1;                   % ẏ
    x0(6) = 1;                   % ż
    
    % Angular Position (rad)
    x0(7) = 60*pi/180;           % φ
    x0(8) = 60*pi/180;           % θ
    x0(9) = 75*pi/180;           % ψ
    
    % Angular velocity (rad/s)
    x0(10) = 10*pi/180;          % p
    x0(11) = 5*pi/180;          % q
    x0(12) = 15*pi/180;          % r

    % Moment of inertia 
    I0       = zeros(6,1);         % [iz; ip; it; ips; ix; iy]

    % Total initial conditions matrix
    X0_ext   = [x0; I0];

    %% 7) Simulation

    tspan = [0 duration];
    [t, Xext] = ode45(@(t,X) combinedDynXYPID(...
        t, X, params, omega_hover, Minv, ...
        Kp_z,Ki_z,Kd_z, ...
        Kp_phi,Ki_phi,Kd_phi, ...
        Kp_theta,Ki_theta,Kd_theta, ...
        Kp_psi,Ki_psi,Kd_psi, ...
        Kp_x,Ki_x,Kd_x, ...
        Kp_y,Ki_y,Kd_y, ...
        x_ref,y_ref, z_ref, psi_ref, g), ...
      tspan, X0_ext);

    %% 8) Plot physical states

    X = Xext(:,1:12);

 %% PLOT 1) Responses of all positions (angular and translational)

    figure;
    subplot(2,1,1);
    plot(t, X(:,1:3), 'LineWidth',1.2);
    legend('x','y','z');
    title('Position under PID+MMA Control');
    xlabel('Time (s)'); ylabel('m'); grid on;

    subplot(2,1,2);
    plot(t, rad2deg(X(:,7:9)), 'LineWidth',1.2);
    legend('\phi','\theta','\psi');
    title('Attitude under PID+MMA Control');
    xlabel('Time (s)'); ylabel('deg'); grid on;

    % Extract only the x,y,z columns:
x_traj = X(:,1);
y_traj = X(:,2);
z_traj = X(:,3);

%% PLOT 2) Trajectory map

figure;
plot3(x_traj, y_traj, z_traj, 'LineWidth', 1.5);
grid on;
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title('3D Trajectory of the Quadcopter');
view(45, 25);   % adjust az/el for a nice perspective
axis equal;     % keep aspect ratio so meters look like meters

% Optionally, mark start and end points:
hold on;
plot3(x_traj(1), y_traj(1), z_traj(1), 'go', 'MarkerSize',10, 'DisplayName','Start');
plot3(x_traj(end), y_traj(end), z_traj(end), 'r^', 'MarkerSize',10, 'DisplayName','End');
legend('Trajectory','Start','End');

end

%% Extended dynamics with full PID outer+inner loops
function dXext = combinedDynXYPID(t, Xext, params, omega_hover, Minv, ...
                                  Kp_z,Ki_z,Kd_z, ...
                                  Kp_phi,Ki_phi,Kd_phi, ...
                                  Kp_theta,Ki_theta,Kd_theta, ...
                                  Kp_psi,Ki_psi,Kd_psi, ...
                                  Kp_x,Ki_x,Kd_x, ...
                                  Kp_y,Ki_y,Kd_y, ...
                                  x_ref,y_ref, z_ref, psi_ref, g)
    % Split extended state
    xstate = Xext(1:12);
    iz     = Xext(13);
    ip     = Xext(14);
    it     = Xext(15);
    ips    = Xext(16);
    ix     = Xext(17);
    iy     = Xext(18);

    % Fixed PID update interval
    dt = 0.01;

    % Unpack vehicle state
    x     = xstate(1);  u = xstate(4);
    y     = xstate(2);  v = xstate(5);
    z     = xstate(3);  w = xstate(6);
    phi   = xstate(7);  p = xstate(10);
    theta = xstate(8);  q = xstate(11);
    psi   = xstate(9);  r = xstate(12);

    %% Outer loops → attitude refs
    ex       = x_ref - x;
    ix       = ix   + ex*dt;
    dex      = -u;   % derivative from x-velocity
    Ux       = Kp_x*ex + Ki_x*ix + Kd_x*dex;
    theta_ref= Ux / g;

    ey       = y_ref - y;
    iy       = iy   + ey*dt;
    dey      = -v;   % derivative from y-velocity
    Uy       = Kp_y*ey + Ki_y*iy + Kd_y*dey;
    phi_ref  = -Uy / g;

    %% Inner loops → U1…U4
    % Altitude U1
    ez    = z_ref - z;
    iz    = iz   + ez*dt;
    dez   = -w;
    U1    = Kp_z*ez + Ki_z*iz + Kd_z*dez;

    % Roll U2
    e_phi = phi_ref - phi;
    ip    = ip   + e_phi*dt;
    dep   = -p;
    U2    = Kp_phi*e_phi + Ki_phi*ip + Kd_phi*dep;

    % Pitch U3
    e_th  = theta_ref - theta;
    it    = it   + e_th*dt;
    det   = -q;
    U3    = Kp_theta*e_th + Ki_theta*it + Kd_theta*det;

    % Yaw U4
    e_ps  = psi_ref - psi;
    ips   = ips  + e_ps*dt;
    deps  = -r;
    U4    = Kp_psi*e_ps + Ki_psi*ips + Kd_psi*deps;

    %% Motor Mixing
    U    = [U1;U2;U3;U4];
    f    = Minv * U;
    f    = max(f, 0);
    omega= sqrt(f);

    % Compute dynamics
    params.omega = omega;
    dx        = quadNonlinearDynamics(t, xstate, params);

    % Integrator derivatives
    diz  = ez;
    dip  = e_phi;
    dit  = e_th;
    dips = e_ps;
    dix  = ex;
    diy  = ey;

    % Pack extended derivative
    dXext = [dx; diz; dip; dit; dips; dix; diy];
end

%% Helper Functions

% loadQuadParams.m
% --------------------------------------------------
% Helper function to load quadcopter parameters

function params = loadQuadParams()
    params.m   = 0.506;
    params.g   = 9.81;
    params.I_x = 8.11858e-5;
    params.I_y = 8.11858e-5;
    params.I_z = 6.12223e-5;
    params.l   = 0.235;
    params.kF  = 3.13e-5;
    params.B   = 75e-7;
end

function dx = quadNonlinearDynamics(~, x, params)
% quadNonlinearDynamics computes the state derivatives for a quadcopter
% using a detailed nonlinear model that calculates the thrusts and torques
% based on individual rotor speeds.
%
% The state vector x is:
%   x = [x; y; z; u; v; w; phi; theta; psi; p; q; r]
% where:
%   - (x, y, z) are the positions in the inertial frame,
%   - (u, v, w) are the linear velocities,
%   - (phi, theta, psi) are the Euler angles (roll, pitch, yaw),
%   - (p, q, r) are the angular velocities about the body axes.
%
% The model computes:
%   - For each rotor: T_i = kF * omega_i^2
%   - Total thrust: F_total = T1 + T2 + T3 + T4
%   - Roll torque:   tau_phi   = l * (T4 - T2)
%   - Pitch torque:  tau_theta = l * (T3 - T1)
%   - Yaw torque:    tau_psi   = kM * (omega1^2 - omega2^2 + omega3^2 - omega4^2)
%
% These are used to compute both the translational dynamics (by rotating the thrust
% into the inertial frame) and the rotational dynamics (using Euler's equations).

% Unpack state variables
pos   = x(1:3);     % Position [x; y; z]
vel   = x(4:6);     % Linear velocity [u; v; w]
phi   = x(7);       % Roll angle
theta = x(8);       % Pitch angle
psi   = x(9);       % Yaw angle
p     = x(10);      % Roll rate
q     = x(11);      % Pitch rate
r     = x(12);      % Yaw rate

% Unpack parameters
m   = params.m;
g   = params.g;
I_x = params.I_x;
I_y = params.I_y;
I_z = params.I_z;
l   = params.l;
kF  = params.kF;
B  = params.B;

% Compute individual rotor thrusts (T_i = kF * omega_i^2)
T1 = kF * (params.omega(1)^2);
T2 = kF * (params.omega(2)^2);
T3 = kF * (params.omega(3)^2);
T4 = kF * (params.omega(4)^2);

% Total thrust produced by all four rotors
F_total = T1 + T2 + T3 + T4;

% Compute torques:
% Roll torque: difference in thrust between right and left rotors.
tau_phi   = l * (T4 - T2);
% Pitch torque: difference in thrust between the front and back rotors.
tau_theta = l * (T3 - T1);
% Yaw torque: due to reactive drag torques generated by the rotors (with appropriate sign).
tau_psi   = B * (-params.omega(1)^2 + params.omega(2)^2 - params.omega(3)^2 + params.omega(4)^2);

% Preallocate state derivative vector
dx = zeros(12,1);

% -----------------------------
% Kinematics: Position derivatives
% -----------------------------
dx(1:3) = vel;  % The derivative of position is the velocity

% -----------------------------
% Translational Dynamics:
% The thrust is applied along the body-fixed z-axis.
% Use the Euler angles to project F_total into the inertial frame.
% -----------------------------
dx(4) = (F_total/m) * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
dx(5) = (F_total/m) * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
dx(6) = (F_total/m) * (cos(phi)*cos(theta)) - g;

% -----------------------------
% Euler Angle Kinematics:
% Assuming a Z-Y-X (yaw-pitch-roll) sequence.
% -----------------------------
dx(7) = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
dx(8) = q*cos(phi) - r*sin(phi);
dx(9) = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

% -----------------------------
% Rotational Dynamics (Euler Equations)
% -----------------------------
dx(10) = (tau_phi - (I_y - I_z)*q*r) / I_x;   % Roll rate derivative (p_dot)
dx(11) = (tau_theta - (I_z - I_x)*p*r) / I_y;  % Pitch rate derivative (q_dot)
dx(12) = (tau_psi - (I_x - I_y)*p*q) / I_z;      % Yaw rate derivative (r_dot)

end
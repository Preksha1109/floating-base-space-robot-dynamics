% dynamics.m 
% Computes the dynamic model of a floating-base robotic arm in microgravity.

% This script formulates the equations of motion using a Lagrangian
% approach, accounting for joint motion, link inertial properties,
% and base reaction effects due to conservation of momentum.

% Author: <Preksha Krishnan>

clc; clear; close all;

% Load Robot Model
robot = importrobot('mybot.urdf');
robot.DataFormat = 'row';

% Set gravity to zero (no gravity)
robot.Gravity = [0 0 0];
mu_earth = 3.986e14;  % m^3/s^2
r_earth = 6778000;    % m (example: 400 km orbit + Earth radius) 

% Floating Base
floatingbot = floatingBaseHelper("row");
addSubtree(floatingbot,"floating_base_RZ",robot,ReplaceBase=false);

nJoints = numel(homeConfiguration(robot));
nFloating = 6;  % base position + orientation
nDOF = nJoints + nFloating;

% Time setup 
tInterval = [0 5];  % motion from 0 to 5 sec
nSteps = 51;
tTimes = linspace(tInterval(1), tInterval(2), nSteps);
dt = (tInterval(2) - tInterval(1)) / (nSteps - 1);

% Define start and end base transforms (floating base)
T0 = trvec2tform([0 0 0]);
Tf = trvec2tform([1 2 3]) * eul2tform([pi/2 0 pi/4],'ZYX');

% Generate trapezoidal velocity profile scalar and its derivatives
[s, sd, sdd] = quinticpolytraj([0 1], tInterval, tTimes);

% Generate transforms and their first and second derivatives
[T, dT, ddT] = transformtraj(T0, Tf, tInterval, tTimes, 'TimeScaling', [s; sd; sdd]);

% Joint Trajectories: only joint 2 moves 0 -> 20 deg, others stay zero
q_start = zeros(1, nJoints);
q_end = zeros(1, nJoints);
q_end(2) = deg2rad(20); 

[q_traj, qd_traj, qdd_traj] = quinticpolytraj([q_start; q_end]', tInterval, tTimes);

q_traj = q_traj';     % nSteps x nJoints
qd_traj = qd_traj';   % nSteps x nJoints
qdd_traj = qdd_traj'; % nSteps x nJoints

% Initialize logging arrays as before
log_q = zeros(nSteps, nDOF);
log_CoM = zeros(nSteps, 3);
log_torque = zeros(nSteps, 3);
logAngles = zeros(nSteps, nJoints);
logVelocities = zeros(nSteps, nJoints);
logAccelerations = zeros(nSteps, nJoints);
logTorques = zeros(nSteps, nJoints);
logKE = zeros(nSteps, 1);
log_torque_gg = zeros(nSteps, 3);
log_torque_total = zeros(nSteps, 3);  % Base torque + gravity gradient
logData = [];

% Loop through time steps
qdd_errors = zeros(nSteps, 1);
for i = 1:nSteps
    % Calculate actual time in seconds
    t = tTimes(i) * (tInterval(2) - tInterval(1)) + tInterval(1);

    % Extract base pose from transformtraj output
    T_base = T(:,:,i);
    basePosition = tform2trvec(T_base);
    baseOrientation = rotm2eul(tform2rotm(T_base), 'ZYX');  % ZYX Euler angles

    % Base linear and angular velocities and accelerations from dT, ddT
    % dT and ddT are 6xN: [vx; vy; vz; wx; wy; wz]
    baseVel = dT(:,i)';
    baseAccel = ddT(:,i)';

    % Joint states at this time step
    joint_q = q_traj(i, :);
    joint_qd = qd_traj(i, :);
    joint_qdd = qdd_traj(i, :);

    % Compose full state vectors: base + joints
    q_floating = [basePosition baseOrientation joint_q];
    qd_floating = [baseVel joint_qd];
    qdd_floating = [baseAccel joint_qdd];

    M = massMatrix(floatingbot, q_floating);
    C = velocityProduct(floatingbot, q_floating, qd_floating);
    G = gravityTorque(floatingbot, q_floating);  
    tau = inverseDynamics(floatingbot, log_q(i,:), qd_floating, qdd_floating);

    % Save current state
    log_q(i,:) = q_floating;

    % Compute total center of mass
    totalMass = 0;
    com_total = zeros(1,3);
    for j = 1:numel(floatingbot.Bodies)
        body = floatingbot.Bodies{j};
        try
            Tbody = getTransform(floatingbot, q_floating, body.Name, floatingbot.BaseName);
        catch
            warning("Could not compute transform for %s", body.Name);
            continue;
        end
        m = body.Mass;
        com = body.CenterOfMass;
        com_world = (Tbody(1:3,1:3) * com(:))' + Tbody(1:3,4)';
        com_total = com_total + m * com_world;
        totalMass = totalMass + m;
    end
    CoM_world = com_total / totalMass;
    log_CoM(i,:) = CoM_world;

    % Store joint data
    logAngles(i,:) = rad2deg(joint_q);
    logVelocities(i,:) = rad2deg(joint_qd);
    logAccelerations(i,:) = rad2deg(joint_qdd);

    % Compute kinetic energy
    M = massMatrix(floatingbot, q_floating);
    KE = 0.5 * qd_floating * M * qd_floating';
    logKE(i) = KE;

    % Center of mass in world frame
    com_world = centerOfMass(floatingbot, q_floating);

    % Relative CoM in base frame
    rotm = eul2rotm(baseOrientation, 'ZYX');
    com_relative = (rotm') * (com_world' - basePosition');
    com_relative = com_relative';

    % Initialize total inertia tensor in inertial frame
    I_total_inertial = zeros(3,3);
    r_eci = [0; 0; 6778e3]; 
    R_BI = eul2rotm(baseOrientation, 'ZYX');  % Rotation from base to inertial
    R_IB = R_BI';  
for j = 1:numel(floatingbot.Bodies)
    body = floatingbot.Bodies{j};
    try
        T_body = getTransform(floatingbot, q_floating, body.Name);
    catch
        continue;
    end
    
    m = body.Mass;
    com_local = body.CenterOfMass(:);  % [x; y; z] in body frame
    R = T_body(1:3,1:3);
    p = T_body(1:3,4);  % Position in world frame
    
    % COM in world frame
    com_world = R * com_local + p;

    % Inertia matrix in body frame
    I_body_vector = body.Inertia;  % [Ixx Iyy Izz Ixy Ixz Iyz]
    I_body_mat = [ I_body_vector(1), -I_body_vector(4), -I_body_vector(5);
                  -I_body_vector(4),  I_body_vector(2), -I_body_vector(6);
                  -I_body_vector(5), -I_body_vector(6),  I_body_vector(3)];
    
    % Rotate to world frame
    I_world = R * I_body_mat * R';

    % Offset vector from total CoM to link CoM
    r_offset = com_world - CoM_world'; 
    
    % Parallel axis theorem
    I_total_inertial = I_total_inertial + I_world + m * (norm(r_offset)^2 * eye(3) - (r_offset * r_offset'));
end

    % Rotate inertia tensor from inertial to body frame
    I_body_com = R_IB' * I_total_inertial * R_IB;  % In body frame
    I_body_com = double(I_body_com);

    % Unit vector from satellite to Earth (in world frame)
    r_hat_inertial = (r_eci - CoM_world') / norm(r_eci - CoM_world');  % vector Earth->satellite in inertial frame
    r_hat_body = R_BI * r_hat_inertial;  % rotate to body frame

    % Gravity gradient torque in body frame
    tau_gg = 3 * mu_earth / norm(r_eci - CoM_world')^3 * cross(r_hat_body, I_body_com * r_hat_body);
   
    % Compute inverse dynamics torques
    tau_all = inverseDynamics(floatingbot, q_floating, qd_floating, qdd_floating);
    tau_base = tau_all(1:6);        % base forces/torques
    tau_joints = tau_all(7:end);    % joint torques

    logTorques(i,:) = tau_joints;
    log_torque(i,:) = (tau_base(4:6)' + tau_gg)';
    log_torque_total(i,:) = log_torque(i,:);

    % Compute qdd using forward dynamics
    qdd_fd = M \ (tau_all(:) - C(:) - G(:)); 
    error = norm(qdd_fd - qdd_floating');
    qdd_errors(i) = error;

    % Log all data for CSV export
     logData(end+1, :) = [...
        t, ...
        joint_q, ...
        joint_qd, ...
        joint_qdd, ...
        com_relative, ...
        KE, ...
        tau_base, ...
        tau_joints ...
    ];

    % Visualization (optional)
    show(floatingbot, q_floating, 'PreservePlot', false, 'Frames','off');
    view(135,20); axis equal; grid on;
    drawnow;
end

%% CSV Logging
headers = {'time', ...
           'q1','q2','q3','q4','q5', ...
           'qd1','qd2','qd3','qd4','qd5', ...
           'qdd1','qdd2','qdd3','qdd4','qdd5', ...
           'com_x','com_y','com_z', ...
           'KE', ...
           'Fx','Fy','Fz','Tx','Ty','Tz', ...
           'tau1','tau2','tau3','tau4','tau5'};
T = array2table(logData, 'VariableNames', headers);
T{:, 2:6} = rad2deg(T{:, 2:6});      % q
T{:, 7:11} = rad2deg(T{:, 7:11});    % qd
T{:, 12:16} = rad2deg(T{:, 12:16});  % qdd

writetable(T, 'robot_floating_log.csv');

%% Plot Results
time = (0:nSteps-1) * dt;
subplot(4,2,5); ylim padded; % adds margin
subplot(4,2,6); ylim padded;
figure;
subplot(3,1,1); plot(time, logAngles); title('Joint Angles'); ylabel('deg'); grid on;
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5');

subplot(3,1,2); plot(time, logVelocities); title('Joint Velocities'); ylabel('deg/s'); grid on;
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5');

subplot(3,1,3); plot(time, logAccelerations); title('Joint Accelerations'); ylabel('deg/s^2'); grid on;
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5');

xlabel('Time (s)');

figure;
plot(time, logTorques); title('Joint Torques'); ylabel('Nm'); xlabel('Time (s)'); grid on;
legend('Joint 1','Joint 2','Joint 3','Joint 4','Joint 5');

figure;
plot(time, logKE); title('Total Kinetic Energy'); ylabel('Joules'); xlabel('Time (s)'); grid on;
figure;
plot3(log_CoM(:,1), log_CoM(:,2), log_CoM(:,3), '-o');
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Total Center of Mass Trajectory in Space');

figure;
plot((0:nSteps-1)*dt, log_torque);
legend('X','Y','Z');
xlabel('Time [s]'); ylabel('Reaction Torque [Nm]');
title('Base Reaction Torques (Angular Momentum Conservation)');

plot(time, logTorques);
hold on;
yline(1.47, '--r', 'Rated Torque Limit');
ylim padded;
exceeds = any(logTorques > 1.47, 2);
plot(time(exceeds), logTorques(exceeds,:), 'x');  % Highlight exceeding points

figure;
plot(tTimes, qdd_errors);
title('Inverse Simulation Error (Forward vs Inverse Dynamics)');
xlabel('Time [s]');
ylabel('||qdd_{forward} - qdd_{given}||');
grid on;

figure;
plot3(log_torque(:,1), log_torque(:,2), log_torque(:,3), '-o');
title('Base Reaction Torque Trajectory (with Gravity Gradient)');
xlabel('Tx'); ylabel('Ty'); zlabel('Tz');
grid on; axis equal;

figure;
plot(time, log_torque_total);
title('Total Base Reaction Torque (with Gravity Gradient)');
xlabel('Time [s]');
ylabel('Torque [Nm]');
legend('Tx','Ty','Tz');
grid on;

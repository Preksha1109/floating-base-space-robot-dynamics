% simulate_robot.m
% Simulates a floating-base robotic arm in microgravity
% Author: <Preksha Krishnan>

clc; clear; close all;

%% Load Robot Model
robot = importrobot('dofbot.urdf');
robot.DataFormat = 'row';

%% Add Floating Base
floatingbot = floatingBaseHelper("row");
addSubtree(floatingbot,"floating_base_RZ",robot,ReplaceBase=false);

%% Define Floating Base + Joint Configurations
baseOrientation = [pi/4 pi/3 pi]; % ZYX Euler rotation order
basePosition = [1.0 0.5 0.75];
robotZeroConfig = zeros(1,5);

q = [basePosition baseOrientation zeros(1,5)];

%% Visualize Robot
show(floatingbot,q);
axis equal;
title("Robot Base ''Floating'' at Desired Position and Orientation")

function robot = floatingBaseHelper(df)
    arguments
        df = "column"
    end
robot = rigidBodyTree(DataFormat=df);
robot.BaseName = 'world';
jointaxname = {'PX','PY','PZ','RX','RY','RZ'};
jointaxval = [eye(3); eye(3)];
parentname = robot.BaseName;
for i = 1:numel(jointaxname)
    bname = ['floating_base_',jointaxname{i}];
    jname = ['floating_base_',jointaxname{i}];
    rb = rigidBody(bname);
    rb.Mass = 0;
    rb.Inertia = zeros(1,6);
    rbjnt = rigidBodyJoint(jname,jointaxname{i}(1));
    rbjnt.JointAxis = jointaxval(i,:);
    rbjnt.PositionLimits = [-inf inf];
    rb.Joint = rbjnt;
    robot.addBody(rb,parentname);
    parentname = rb.Name;
end
end

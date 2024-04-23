%% High level file to move to can and pick up

% Preliminary ROS setup
rosshutdown;
host_name = '192.168.56.128';
rosinit(host_name);

%% Subscribe to different ROS actions

% These are the global variables for joint_subscribers and trajectory
% actions
global pick_traj_act_client;
%global pointSub;
%global ImageSub;
global joint_state_sub;

% subscribe to the joint trajectory client
pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
%pointSub = rossubscriber('/camera/depth/points', 'sensor_msgs/PointCloud2', 'DataFormat', 'struct');
%ImageSub = rossubscriber('/camera/rgb/image_raw', 'sensor_msgs/Image', 'DataFormat','struct');
joint_state_sub = rossubscriber("/joint_states", 'DataFormat','struct');

% Set robot in home position
goHome;

%% Main functions that perform the different actions
[C_T_M, labels] = get_pose_of_targets_in_field_of_view;
R_T_G = getRTG();
R_T_M = convertCTM2RTM(C_T_M);

% Using the acquired R_T_M, send the robot to "pick" the object at that
% location
trajectoryResult = pick('topdown', R_T_M, R_T_G);

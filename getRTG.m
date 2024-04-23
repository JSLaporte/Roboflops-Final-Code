function [R_T_G] = getRTG()
   %----------------------------------------------------------------------
    % getRTG
    %----------------------------------------------------------------------
    % This function retrieves the transformation matrix from robot base to 
    % gripper tip
    %
    % The function will accomplish 3 steps:
    % 1) use getTransform to get quaternion transformation baseToGripper
    % 2) Record translation and rotation of baseToGripper
    % 3) Convert translation and rotation into transformation matrix
    %
    % Input arguments:
    % none
    %
    % Output:
    % R_T_G (4*4 matrix): robot to gripper transformation matrix
    %----------------------------------------------------------------------

% Initialization - Wait to start
tftree = rostf;
disp('Red Light');
pause(5);
disp('Green Light');

% Getting quaternion transformation from base to gripper
baseToGripper = getTransform(tftree, 'base_link', 'gripper_tip_link');

%Writing translation of rotation of baseToGripper to variables
X = baseToGripper.Transform.Rotation.X;
Y = baseToGripper.Transform.Rotation.Y;
Z = baseToGripper.Transform.Rotation.Z;
W = baseToGripper.Transform.Rotation.W;
x1 = baseToGripper.Transform.Translation.X;
y1 = baseToGripper.Transform.Translation.Y;
z1 = baseToGripper.Transform.Translation.Z;

%Converting baseToGripper parameters to transformation matrix
R_T_G = quat2tform([W,X,Y,Z]);
R_T_G(1,4) = x1;
R_T_G(2,4) = y1;
R_T_G(3,4) = z1;
R_T_G(4,4) = 1;

end
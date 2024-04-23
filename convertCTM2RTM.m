function [R_T_M] = convertCTM2RTM(C_T_M)
    %----------------------------------------------------------------------
    %convertCTM2RTM
    %----------------------------------------------------------------------
    %
    % convertCTM2RTM inputs camera to model XYZ cooridnates, outpus robot to
    % model coordinates
    %
    % The function will accomplish 3 steps:
    % 1) Obtain baseToCamera rotation and translation in quaternion form
    % 2) Convert rotation and translation into a transformation matrix
    % R_T_C
    % 3) Multiply baseToCamera transformation matrix R_T_C by camera to
    % model transformation matrix C_T_M
    %
    % Input arguments:
    % C_T_M (4*4 matrix): camera to model transformation matrix obtained
    % from point cloud data
    %
    % Output:
    % R_T_M (4*4 matrix): robot to model transformation matrix
    %----------------------------------------------------------------------

tftree = rostf;

% Getting baseToCamera translation & rotation
baseToCamera = getTransform(tftree, 'base_link', 'camera_link');
X = baseToCamera.Transform.Rotation.X;
Y = baseToCamera.Transform.Rotation.Y;
Z = baseToCamera.Transform.Rotation.Z;
W = baseToCamera.Transform.Rotation.W;
x1 = baseToCamera.Transform.Translation.X;
y1 = baseToCamera.Transform.Translation.Y;
z1 = baseToCamera.Transform.Translation.Z;
% quat = UnitQuaternion;
% []
% Creating transformation matrix R_T_C from quaternion
R_T_C = quat2tform([W,X,Y,Z]);
R_T_C(1,4) = x1;
R_T_C(2,4) = y1;
R_T_C(3,4) = z1;
R_T_C(4,4) = 1;

% Multiplying transformation matricies to get total transformation
R_T_M = R_T_C*C_T_M
end
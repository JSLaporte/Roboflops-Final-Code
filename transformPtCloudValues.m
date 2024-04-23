function transformedXYZPtCloud = transformPtCloudValues()


pointSub = rossubscriber('/camera/depth/points', 'DataFormat', 'struct');
ptcloud = receive(pointSub);

tftree       = rostf('DataFormat','struct');   
base         = 'base_link';
end_effector = ptcloud.Header.FrameId; %'camera_depth_link'; 

% this line will not work if you do not have 'ROS Toolbox Support Package for TurtleBot-Based Robots'
p = getTransform(tftree,base,end_effector);

pos = [ p.Transform.Translation.X, ... 
        p.Transform.Translation.Y, ...
        p.Transform.Translation.Z]; 

q = UnitQuaternion(p.Transform.Rotation.W, ...
                   [p.Transform.Rotation.X, ...
                    p.Transform.Rotation.Y, ...
                    p.Transform.Rotation.Z]);

q.T
T = transl(pos) * q.T;

% Extract Nx3 matrix of XYZ points. I changed this line from 'readXYZ' to
% 'rosReadXYZ' because 'readXYZ' was not recognized
xyz = rosReadXYZ(ptcloud);

% n rows by 3 cols with (x,y,z) tuples
[r,c]=size(xyz); 

% Transform each row of points, need homogeneous coords P=[x,y,z,1]'
pointsHomogeneous = [xyz, ones(size(xyz, 1), 1)];

% Apply the transformation
transformedPointsHomogeneous = pointsHomogeneous * T';

% Remove the homogeneous coordinate
transformedXYZ = transformedPointsHomogeneous(:, 1:3);

% Recreate the pointCloud object if needed for visualization or further processing
% The pointCloud object creates point cloud data from a set of points in 3-D coordinate system. The points generally represent the x,y, and z geometric coordinates for samples on a surface or of an environment. Each point can also be represented with additional information, such as the RGB color. The point cloud data is stored as an object with the properties listed in Properties. Use Object Functions to retrieve, select, and remove desired points from the point cloud data.
transformedXYZPtCloud = pointCloud(transformedXYZ);

%% Visualize
figure2 = figure;
axes2 = axes(Parent=figure2);
pcshow(transformedXYZPtCloud,Parent=axes2,AxesVisibility='on');
xlabel('X');
ylabel('Y');
zlabel('Z');
title({'Transformed Point Cloud'},FontSize=14)


% numPoints = size(transformedXYZ, 1);
% newData = zeros(numPoints * ptcloud.PointStep, 1, 'uint8');
% for i = 1:numPoints
%     % Each point's data starts at index base
%     base = (i - 1) * ptcloud.PointStep + 1;
% 
%     % Convert each X, Y, Z to single precision, then convert to bytes (1x4 uint8) and place them in the correct position
%     newData(base:base+3) = typecast(single(transformedXYZ(i, 1)), 'uint8');
%     newData(base+4:base+7) = typecast(single(transformedXYZ(i, 2)), 'uint8');
%     newData(base+8:base+11) = typecast(single(transformedXYZ(i, 3)), 'uint8');
% 
%     % Copy existing RGB data
%     % Assuming RGB starts at byte 16 and is 4 bytes long
%     rgbIndex = base + 16;
%     newData(rgbIndex:rgbIndex+3) = ptcloud.Data(rgbIndex:rgbIndex+3);   
% end
% ptcloud.Data = newData;

end
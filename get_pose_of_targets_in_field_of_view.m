function [C_T_M, labels] = get_pose_of_targets_in_field_of_view()
%--------------------------------------------------------------------------
% get_pose_of_targets_in_field_of_view
%--------------------------------------------------------------------------
% This function takes a rgb and ptcloud image from the current position,
% runs yolo for object detection,
% and uses the point cloud data to extrapilate an XYZ and theta of
% of all of the targets in the field of view.
% The XYZ and Thetas are then output in an array
%
% Input Arguments:
% None
%
% Outputs:
% C_T_M (4*4 matrix):camera to model transformation obtained from point
% cloud data
% labels: Array of labels of bounding boxes obtained from 
% YOLO neural network
%--------------------------------------------------------------------------

load('can_detector_20240412.mat'); % going to change to our own data set
yoyloScoreTh = 0.5; % threshold if bounding box is good. Scale of 0 (bad) to 1 (good)

%% Initialization of Subscribers
pointSub = rossubscriber('/camera/depth/points', 'sensor_msgs/PointCloud2', 'DataFormat', 'struct'); % this subscribes to get pointCloud2 images and data
ptcloudmsg = receive(pointSub); 
xyz_array = rosReadXYZ(ptcloudmsg); % convert the image to MATLAB readable format
pc = pointCloud(xyz_array); % stores 3-D point cloud
ImageSub = rossubscriber('/camera/rgb/image_raw', 'sensor_msgs/Image', 'DataFormat','struct'); % this subscribes to get the RGB image for bounding boxes
disp('Subscriptions Complete'); 
pause(2);

%% YOLO Data Collection
[bboxes, scores, labels, img] = takePhotoRgb(ImageSub.LatestMessage, detector); % setup bounding boxes on RGB image

showPartialOutputs = true;

%Check if there's any objects detected and display image
if isempty(bboxes)
    disp('No Objects Detected')
    
else 
    disp('Well Done! Object Found')
end

if(showPartialOutputs)
    detectedImg = insertObjectAnnotation(img,"Rectangle",bboxes,labels); % display boxes in image
    figure, imshow(detectedImg)
end

% Get information for bounding boxes above score threshold
valid_idx = scores > yoyloScoreTh;
bboxes = bboxes(valid_idx, :);
scores = scores(valid_idx);
labels = labels(valid_idx);
numObjects = size(bboxes,1);

% Parameters for table plane
PlanrThickness = 0.02;
normalvector = [0,0,1];
maxPlaneTilt = 5;
[param, planeIdx, nonPlaneIdx] = pcfitplane(pc, PlanrThickness, normalvector, maxPlaneTilt);

disp('Table Mask Set Up');

plane = select(pc, planeIdx); % setup plane mask
nonPlane = select(pc, nonPlaneIdx); % setup non-plane mask

% display image
if(showPartialOutputs)
    figure,pcshow(plane,'ViewPlane','XY');axis on;
end

% non-plane mask setup
[m,n,~] = size(img);
nonPlaneMask = zeros(m,n);
nonPlaneMask =nonPlaneMask(:);
nonPlaneMask(nonPlaneIdx)= 1;

%% Obtaining Poses from Point Cloud and Convert
gridDownsample = 0.01; 
[xyz,theta,ptCloud_vec,scene_pca_vec] = findObjectPoses(pc,img, bboxes, gridDownsample, nonPlaneMask);
C_T_M = [cos(rad),-sin(rad),0,xyz(1);...
             sin(rad),cos(rad), 0,xyz(2);...
             1,         1,      1,xyz(3);...
             0,         0,      0,1    ];

disp('Point Cloud Positions Calculated');

%Making the plot 
if(showPartialOutputs)
    figure;
    for idx = 1: numObjects
        U = scene_pca_vec{idx}.UVW(:,1);
        V = scene_pca_vec{idx}.UVW(:,2);
        W = scene_pca_vec{idx}.UVW(:,3);
        center = scene_pca_vec{idx}.centroid;
        nexttile;
        pcshow(ptCloud_vec{idx},'ViewPlane','XY');
        hold on;
        quiver3(center(1), center(2), center(3), U(1), V(1), W(1), 'r');
        quiver3(center(1), center(2), center(3), U(2), V(2), W(2), 'g');
        quiver3(center(1), center(2), center(3), U(3), V(3), W(3), 'b');
        hold off;
    end
end

end
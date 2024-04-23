
function [bboxes, scores, labels, img] = takePhotoRgb(ImageSub, detector)
%--------------------------------------------------------------------------
%takePhotoRgb
%--------------------------------------------------------------------------
% First, format image from subscriber
% Save image as .jpg in directory
% Read in image
% Generate boxes, score, and labels

% Format image from subscriber
imageFormatted = rosReadImage(ImageSub); 

% Save image as .jpg in directory
imwrite(imageFormatted, 'C:\Users\jjice\OneDrive\Documents\GitHub\matlab_ros_support_code\output.jpg');
 
% Read in image
img = imread("C:\Users\jjice\OneDrive\Documents\GitHub\matlab_ros_support_code\output.jpg"); 

% Generate boxes, score, and labels
[bboxes,scores,labels] = detect(detector,img); 

end

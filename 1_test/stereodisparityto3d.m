% disparity map

%% Camera calibration (see separate test program cameracalibration.m)

%Create a set of calibration images.
%images = imageDatastore(fullfile(toolboxdir('vision'), 'visiondata', ...
%    'calibration', 'mono'));
images = imageDatastore('phonecal');     %phone calibration checkerboards
imageFileNames = images.Files;

% Detect calibration pattern.
[imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);

% Generate world coordinates of the corners of the squares.
squareSize = 35; % millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
I = readimage(images, 1); 
imageSize = [size(I, 1), size(I, 2)];
[cameraParams, ~, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
                                     'ImageSize', imageSize);
                                 
%% Import images
% Load a pair of images into the workspace.

images = imageDatastore('littleprincestereo');
original1 = readimage(images, 1);
original2 = readimage(images, 2);
figure
imshowpair(original1, original2, 'montage'); 
title('Original Images');

%% Remove Lens Distortion
% Lens distortion can affect the accuracy of the final reconstruction. 
%You can remove the distortion from each of the images using the undistortImage function. 
%This process straightens the lines that are bent by the radial distortion of the lens.

I1 = undistortImage(original1, cameraParams);
I2 = undistortImage(original2, cameraParams);
figure 
imshowpair(I1, I2, 'montage');
title('Undistorted Images');

%% 1. Find Point Correspondences Between The Images (image 1)
% Detect good features to track. Reduce 'MinQuality' to detect fewer points, which would be more uniformly distributed throughout the image. 
%If the motion of the camera is not very large, then tracking using the KLT algorithm is a good way to establish point correspondences.

% Detect feature points in the first image
imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'MinQuality', 0.1);

% Visualize detected points
figure
imshow(I1, 'InitialMagnification', 50);
title('150 Strongest Corners from the First Image');
hold on
plot(selectStrongest(imagePoints1, 150));

%% 1b. Correspondances to image 2
% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);

% Track the points
[imagePoints2, validIdx] = step(tracker, I2);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

% Visualize correspondences
figure
showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
title('Tracked Features');

%% 2. Estimate the Fundamental Matrix (& use in filter)
%Use the estimateFundamentalMatrix function to compute the fundamental matrix and find the inlier points that meet the epipolar constraint.

% Estimate the fundamental matrix
[fMatrix, epipolarInliers] = estimateFundamentalMatrix(...
  matchedPoints1, matchedPoints2, 'Method', 'MSAC', 'NumTrials', 10000);

% Find epipolar inliers
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

% Display inlier matches
figure
showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
title('Epipolar Inliers');

%% 3. Compute the Camera Pose
%Compute the rotation and translation between the camera poses corresponding to the two images. 
%Note that t is a unit vector, because translation can only be computed up to scale.

[R, t] = cameraPose(fMatrix, cameraParams, inlierPoints1, inlierPoints2);

%% 4. Rectify stereo images

%Compute the fundamental matrix from the corresponding points.
f = estimateFundamentalMatrix(inlierPoints1,inlierPoints2,...
    'Method','Norm8Point');

%Compute the rectification transformations.
[t1, t2] = estimateUncalibratedRectification(f,inlierPoints1,...
    inlierPoints2,size(I2));

%Rectify the stereo images using projective transformations t1 and t2.
[I1Rect,I2Rect] = rectifyStereoImages(I1,I2,t1,t2);

%Display the stereo anaglyph, which can also be viewed with 3-D glasses.
figure;
imshow(stereoAnaglyph(I1Rect,I2Rect));
% SfM from 2 views (See MATLAB documentation example with the globes)

%% Procedure (this is all one comment)

%This example shows how to reconstruct a 3-D scene from a pair 2-D images taken with a camera calibrated using the Camera Calibrator app. The algorithm consists of the following steps:

%1 Match a sparse set of points between the two images. There are multiple ways of finding point correspondences between two images. This example detects corners in the first image using the detectMinEigenFeatures function, and tracks them into the second image using vision.PointTracker. Alternatively you can use extractFeatures followed by matchFeatures.

%2 Estimate the fundamental matrix using estimateFundamentalMatrix.

%3 Compute the motion of the camera using the cameraPose function.

%4 Match a dense set of points between the two images. Re-detect the point using detectMinEigenFeatures with a reduced 'MinQuality' to get more points. Then track the dense points into the second image using vision.PointTracker.

%5 Determine the 3-D locations of the matched points using triangulate.

%6 Detect an object of a known size. In this scene there is a globe, whose radius is known to be 10cm. Use pcfitsphere to find the globe in the point cloud.

%7 Recover the actual scale, resulting in a metric reconstruction.

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

%% 4. Reconstruct the 3-D Locations of Matched Points
%Re-detect points in the first image using lower 'MinQuality' to get more points. Track the new points into the second image. 
%Estimate the 3-D locations corresponding to the matched points using the triangulate function, 
%which implements the Direct Linear Transformation (DLT) algorithm [1]. 
%Place the origin at the optical center of the camera corresponding to the first image.

% Detect dense feature points
imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'MinQuality', 0.001);

% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);

% Track the points
[imagePoints2, validIdx] = step(tracker, I2);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

% Compute the camera matrices for each position of the camera
% The first camera is at the origin looking along the X-axis. Thus, its
% rotation matrix is identity, and its translation vector is 0.
camMatrix1 = cameraMatrix(cameraParams, eye(3), [0 0 0]);
camMatrix2 = cameraMatrix(cameraParams, R', -t*R');

% Compute the 3-D points
points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);

% Get the color of each reconstructed point
numPixels = size(I1, 1) * size(I1, 2);
allColors = reshape(I1, [numPixels, 3]);
colorIdx = sub2ind([size(I1, 1), size(I1, 2)], round(matchedPoints1(:,2)), ...
    round(matchedPoints1(:, 1)));
color = allColors(colorIdx, :);

% Create the point cloud
ptCloud = pointCloud(points3D, 'Color', color);

%% 5. Display the 3-D Point Cloud
%Use the plotCamera function to visualize the locations and orientations of the camera, and the pcshow function to visualize the point cloud.

% Visualize the camera locations and orientations
cameraSize = 0.3;
figure
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
hold on
grid on
plotCamera('Location', t, 'Orientation', R, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);

% Visualize the point cloud
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);

% Rotate and zoom the plot
camorbit(0, -30);
camzoom(1.5);

% Label the axes
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis')

title('Up to Scale Reconstruction of the Scene');

%% 6. Fit a Plane to the Point Cloud to Find the book
%Find the book in the point cloud by fitting a Plane to the 3-D points using the pcfitplane function.

% Detect the globe
book = pcfitplane(ptCloud, 0.01);

% Display the surface of the globe
plot(book);
title('Estimated Location and Size of the Globe');
hold off


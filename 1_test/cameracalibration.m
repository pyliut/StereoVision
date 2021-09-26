% Calibrate camera

%% Create a set of calibration images.
%images = imageDatastore(fullfile(toolboxdir('vision'), 'visiondata', ...
%    'calibration', 'mono'));
images = imageDatastore('phonecal');     %phone calibration checkerboards
imageFileNames = images.Files;

%% Detect calibration pattern.
[imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);

%% Generate world coordinates of the corners of the squares.
squareSize = 35; % millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

%% Calibrate the camera.
I = readimage(images, 1); 
imageSize = [size(I, 1), size(I, 2)];
[cameraParams, ~, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
                                     'ImageSize', imageSize);

%% Show extrinsice
figure;
showExtrinsics(cameraParams,'patternCentric');

% this follows a youtube video 'Computer Vision Made Easy' by 'MATLAB
% SOFTWARE'

%% Read images
book1 = imread('littleprince.jpeg');     % imports data from the saved jpeg file, in this case a picture of the Little Prince book
book2 = imread('littleprince3.jpeg');     % second image of book in cluttered setting

%imshow(book1)                           % shows the original image, different plots are avaiable under the plots tab if you select the variable in the workspace
grey1 = rgb2gray(book1);                  % rgb2gray() converts colour image to greyscale
grey2 = rgb2gray(book2);

%% ASIDE: display pictures
%figure;imshow(book1);title('Object')        % creates two images in separate screens
%figure;imshow(book2);title('Scene')

%% detect SURF features

points1 = detectSURFFeatures(grey1);        % detect interest points in image 1
points2 = detectSURFFeatures(grey2);

%% ASIDE: shows 20 strongest features on the original images - HWVR these are not the same in the 2 images
%imshow(book1); hold on;
%plot(points1.selectStrongest(20));
%imshow(grey2); hold on;
%plot(points2.selectStrongest(20));

%% Extract features
[feats1,valid1] = extractFeatures(grey1,points1);     %feature elements are 64 elements long
[feats2,valid2] = extractFeatures(grey2,points2);

% Locate object in an image by finding, extracting, matching features. 
%% Display features
figure;imshow(book1);hold on;plot(valid1,'showOrientation',true);    %showOrientation shows the orientation of the SURF features
title('Detected Features');

%% Match features
pairs = matchFeatures(feats1,feats2,'Prenormalized',true);
matches1 = valid1(pairs(:,1));                      %matched points
matches2 = valid2(pairs(:,2));
figure;showMatchedFeatures(book1,book2,matches1,matches2,'montage');
title('Initial Matches');

%% RANSAC to remove outliers while estimating geometric transform
[transform,inliers1,inliers2] = estimateGeometricTransform(matches1,matches2,'affine');
figure;showMatchedFeatures(book1,book2,inliers1,inliers2,'montage');title('FilteredMatches')

%% Locate object with estimated transform
% original object outline
box1 = [1,1;                        %top left
    size(book1,2), 1;               %top right
    size(book1,2), size(book1,1);   %bottom right
    1,size(book1,1);                %bottom left
    1,1];                           %top left to close

% new object location
box2 = transformPointsForward(transform,box1);
figure;imshow(book2);hold on;
line(box2(:,1),box2(:,2),'Color', 'r', 'LineWidth',5);
title('Detected object');
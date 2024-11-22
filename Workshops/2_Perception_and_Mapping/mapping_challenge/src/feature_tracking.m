function [inliers1,inliers2,outliers1,outliers2,E] = feature_tracking(I1,I2,camIntr)
% Perform feature tracking between consecutive frames

% Detect features
points1 = detectKAZEFeatures(I1);
points2 = detectKAZEFeatures(I2);

% Extract features, i.e., compute feature descriptors
[features1,valid_points1] = extractFeatures(I1,points1);
[features2,valid_points2] = extractFeatures(I2,points2);

% Match features between the two images
indexPairs = matchFeatures(features1,features2);
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

% Estimate the essential matrix describing the relative pose between the
% two camera views
[E, inliers] = estimateEssentialMatrix(matchedPoints1, matchedPoints2, camIntr,...
    "Confidence",99.99,"MaxDistance",0.1,"MaxNumTrials",1000);

% Reject outliers
inliers1 = matchedPoints1(inliers);
inliers2 = matchedPoints2(inliers);

outliers1 = matchedPoints1(~inliers);
outliers2 = matchedPoints2(~inliers);

end
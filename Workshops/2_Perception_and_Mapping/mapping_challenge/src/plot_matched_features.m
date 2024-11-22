function plot_matched_features(I1,I2,inliers1,inliers2,outliers1,outliers2)
% Plot inlier and outlier feature matches between two images

figure
showMatchedFeatures(I1,I2,inliers1,inliers2)
title('Inlier Features')

figure
showMatchedFeatures(I1,I2,outliers1,outliers2,"PlotOptions",{'ro','g+','r-'})
title('Outlier Features (Rejected)')

end
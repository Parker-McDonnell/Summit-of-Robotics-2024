function [point_cloud,inliers_prev,inliers_curr,outliers_prev,outliers_curr...
    ] = extract_point_cloud(img_prev, img_curr, cam_intr, rel_rot, rel_pos)
% Extract 3D point cloud using images from two different camera poses

% Perform feature tracking
[inliers_prev,inliers_curr,outliers_prev,outliers_curr] = ...
    feature_tracking(img_prev,img_curr,cam_intr);

% Triangulate keypoints to construct point cloud
% p_dlt = triangulate(inliers_prev,inliers_curr,cam_proj_prev,cam_proj_curr);
points_3d = triangulate_keypoints_DLT(inliers_prev.Location',inliers_curr.Location',...
    cam_intr, eye(3), rel_rot, [0;0;0], rel_pos);
point_cloud = pointCloud(points_3d');

end
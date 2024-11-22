function shape_model = shape_reconstruction(pclouds,pose_graph)
% Reconstruct a shape model starting form a set of camera-relative point
% clouds and the associated transformations from camera frame to world
% frame

n_pclouds = size(pclouds,2);
grid_step = 0.01;

% Initialize shape model with first point cloud
shape_model = pclouds{1};

% Loop through point clouds
for i = 2:n_pclouds

    tform_vec = nodeEstimates(pose_graph,i);
    tform = rigidtform3d(quat2rotm(tform_vec(4:7)),tform_vec(1:3));
    pcloud_world = pctransform(pclouds{i}, tform);
    shape_model = pcmerge(shape_model,pcloud_world,grid_step);

end

end
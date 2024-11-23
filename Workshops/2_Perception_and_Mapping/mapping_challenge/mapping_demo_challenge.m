% Asteroid mapping demo
% CU Robotics Retreat, Nov 22, 2024
% Author: Jacopo Villa, jacopo.villa@colorado.edu

close all
clear all
clc

disp('===================================================================')
disp('Asteroid Mapping')
disp('===================================================================')

% Add dependencies
addpath('src/')

% Set random seed for reproducibility
rng(123)

%% Simulation Setup

disp('Setting up simulation...')

% Define camera intrinsic parameters
fov_deg = 40; % [deg]
cam_res = 3000; % [pixels]
foc_length = cam_res/(2*tand(fov_deg/2)); % [pixels]
cam_intr = cameraIntrinsics([foc_length, foc_length],[cam_res/2,cam_res/2],...
    [cam_res, cam_res]);

% Load images
img_path = "images/";
img_data = imageDatastore(img_path);

% Load camera-pose data
load('rot_cam2ast.mat')
load('baselines.mat')

n_poses = size(baselines,2);

% Compute relative pose between consecutive camera views
rel_rot_cam = zeros(3,3,n_poses-1);
rel_pos = zeros(3,n_poses-1);
for i = 1:n_poses
    % Compute relative rotation
    rot_prev = rot_cam2ast(:,:,i);
    rot_curr = rot_cam2ast(:,:,i+1);
    rel_rot_cam(:,:,i) = rot_prev'*rot_curr;

    % Compute relative position
    rel_pos(:,i) = rot_prev'*baselines(:,i);
end

% Load true camera positions in the asteroid-fixed frame
% Note: (only for plotting purposes, not used for shape reconstruction!)
load('cam_pos_true.mat')
pos_true_camframe = zeros(3,n_poses);
for i = 2:n_poses
    pos_true_camframe(:,i) = rot_cam2ast(:,:,1)' * ...
        (cam_pos_true(:,i) - cam_pos_true(:,1));
end

%% Initialize Mapping

disp('Initializing mapping...')

% Initialize figures
h_pc_curr = figure; % point cloud figure
h_pc_compare = figure; % point-cloud comparison figure
h_poses = figure; % camera-pose figure

% Initialize pose graph object
pose_graph = poseGraph3D;
tform_tot = rigidtform3d;

% Read image pair
i_pose = 1;
img_prev = readimage(img_data,i_pose);
img_curr = readimage(img_data,i_pose+1);

% Extract point cloud
[pclouds{i_pose}, inl_prev, inl_curr, outl_prev, outl_curr] = ...
    extract_point_cloud(img_prev, img_curr, cam_intr, ...
    rel_rot_cam(:,:,i_pose), rel_pos(:,i_pose));

% Plot matched features
plot_matched_features(img_prev,img_curr,inl_prev,inl_curr,outl_prev,outl_curr)

% Initialize asteroid map
pc_merged = pclouds{i_pose};

% Plot current asteroid map
plot_point_cloud(pc_merged,h_pc_curr);

%% Run Mapping Loop

disp('Running asteroid mapping...')

% Loop through images
for i_pose = 2:n_poses

    disp(['Processing pose no. ' num2str(i_pose)])

    % Read image pair
    img_prev = readimage(img_data,i_pose);
    img_curr = readimage(img_data,i_pose+1);

    % Extract point cloud
    pclouds{i_pose} = extract_point_cloud(img_prev, img_curr, cam_intr, ...
    rel_rot_cam(:,:,i_pose), rel_pos(:,i_pose));

    % Register point clouds
    inlier_ratio = 0.6;
    [tform,~,rmse] = pcregistericp(pclouds{i_pose},pclouds{i_pose-1},...
        'InlierRatio',inlier_ratio,'MaxIterations',1000,...
        'InitialTransform',rigidtform3d,'Metric','planeToPlane',...
        'Tolerance',[1e-8, 1e-8]);

    % Transform current point cloud to align it with previous point cloud
    pc_curr_tformed = pctransform(pclouds{i_pose},tform);

    % Compare point clouds before and after registration
    plot_pclouds_before_after_reg(pclouds{i_pose-1},pclouds{i_pose},...
        pc_curr_tformed,h_pc_compare)

    % Add relative pose to pose graph
    rel_pose_vec = [tform.Translation rotm2quat(tform.R)];
    info_mat = [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1];
    addRelativePose(pose_graph,rel_pose_vec,info_mat,i_pose-1,i_pose);

    % Merge this point cloud with previous ones to update the shape model
    pc_merged = shape_reconstruction(pclouds,pose_graph);

    % Plot current asteroid map
    plot_point_cloud(pc_merged,h_pc_curr);
    
    % Plot camera poses
    figure(h_poses)
    clf
    show(pose_graph)
    hold on
    plot3(pos_true_camframe(1,1:i_pose),pos_true_camframe(2,1:i_pose),...
        pos_true_camframe(3,1:i_pose),'bo')
    title('Estimated Camera Trajectory')
    legend('Estimated Rel. Pos.','Est. Pos.','True Pos.')

end

% Transform reconstructed point cloud to asteroid-fixed (world) frame
pc_merged_bodyframe = (rot_cam2ast(:,:,1) * pc_merged.Location' + cam_pos_true(:,1))' ;
pc_merged_bodyframe = pointCloud(pc_merged_bodyframe);

% Load ground-truth point cloud
mesh_true = readSurfaceMesh('Eros Gaskell 200k poly.obj');
pc_true = pointCloud(mesh_true.Vertices);

% Compare reconstructed point cloud with ground truth
figure
pcshowpair(pc_true,pc_merged_bodyframe)
title('Reconstructed Asteroid Map','Color','w')
legend('True','Estimated','TextColor','w')

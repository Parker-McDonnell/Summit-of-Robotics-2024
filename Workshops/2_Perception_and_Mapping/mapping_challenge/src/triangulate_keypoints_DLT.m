function p = triangulate_keypoints_DLT(uv1,uv2,cam_intr, ...
    rot_prev, rot_curr, pos_prev, pos_curr)
% Triangulate keypoints using the Direct Linear Transform (DLT)

foc_length = cam_intr.FocalLength(1); % Assuming same focal length for u and v

% Initialize point cloud using triangulation based on a-priori camera poses
los1 = [uv1 - cam_intr.PrincipalPoint'; foc_length*ones(1,size(uv1,2))];
los1 = los1./vecnorm(los1,2,1);
los1 = rot_prev*los1;
los2 = [uv2 - cam_intr.PrincipalPoint'; foc_length*ones(1,size(uv2,2))];
los2 = los2./vecnorm(los2,2,1);
los2 = rot_curr*los2;

n_p = size(los1,2);
p = zeros(3,n_p);
for i = 1:n_p
    p(:,i) = DLT([los1(:,i), los2(:,i)],[pos_prev, pos_curr]);
end

end

function r = DLT(los,p)
% Direct Linear Transform (DLT) triangulation algorithm

n_obs = size(los,2);

% Loop through LOS observations
H = zeros(n_obs*3,3);
y = zeros(n_obs*3,1);
for i = 1:n_obs

    idx_mat_i = 3*i-2:3*i;
    S_los_i = skew_symm_mat(los(:,i));
    H(idx_mat_i,:) = S_los_i;
    y(idx_mat_i) = S_los_i * p(:,i);

end

% Estimate point position
r = (H'*H)\H'*y;

function S_x = skew_symm_mat(x)
    % Evaluate skew-symmetric matrix

    S_x = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

end

end

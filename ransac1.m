function [glob_eqn_params, three_pts, glob_inlier_idx] = ransac1(pts_3d, minimal_pts, threshold)
%function [glob_inlier_pts,glob_outlier_pts,glob_eqn_params] = ransac1(pts_3d, minimal_pts, threshold)
%[B, P, inliers1] = ransacfitplane(pts_xyz', 0.5);

% Returns:
%           B - 4x1 array of plane coefficients in the form
%               b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0
%               The magnitude of B is 1.
%               This plane is obtained by a least squares fit to all the
%               points that were considered to be inliers, hence this
%               plane will be slightly different to that defined by P below.
%           P - The three points in the data set that were found to
%               define a plane having the most number of inliers.
%               The three columns of P defining the three points.
%           inliers - The indices of the points that were considered
%                     inliers to the fitted plane.
%

glob_inlier_cnt = 0;
%x = [min(pts_3d,1) - threshold(1) : 0.1 : max(pts_3d,1) + threshold(1)];
%y = [min(pts_3d,2) - threshold(2) : 0.1 : max(pts_3d,2) + threshold(2)];
%z = [min(pts_3d,3) - threshold(3) : 0.1 : max(pts_3d,3) + threshold(3)];
N = 3;
for i = 1 : N
    k = randperm(size(pts_3d,2));
    indices = k(1:minimal_pts);
    pts = pts_3d(:,indices);
    eqn_params = least_sq_fit(pts);
    a = eqn_params(1);
    b = eqn_params(2);
    c = eqn_params(3);
    d = eqn_params(4);
    
    eq = [a; b; c; d];
    
    inlier_pts = [];
    outlier_pts = [];
    
    for j = 1 : size(pts_3d,2)
        pt = [pts_3d(:,j);1];
        dist = (pt' * eq)/norm([a,b,c],2);
        if dist <= threshold
            inlier_pts = [inlier_pts ; j];
        else
            outlier_pts = [outlier_pts ; j];
        end
    end
    if size(inlier_pts,1) > glob_inlier_cnt
        glob_inlier_cnt = size(inlier_pts,1);
        glob_inlier_idx = inlier_pts;
        glob_eqn_params = eqn_params;
        glob_outlier_pts = outlier_pts;
        three_pts = pts;
    end
end

end
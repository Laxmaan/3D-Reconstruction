function [zbuffer] = get_Z_buffer(points_3d,points_2d,K,R_t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% points 3d of shape (4,N)
R = R_t(:,1:3);
T = R_t(:,4);

pt = mean(points_2d,2); %use one point to get u
u = R'*pinv(K)* (pt/norm(pt));

u_hat = u / norm(u);

zbuffer = u_hat' * points_3d(1:3,:);
end


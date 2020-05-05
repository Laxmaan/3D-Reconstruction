function params = least_sq_fit(set_pts)
partA = set_pts(1:2,:);
B = set_pts(3,:);
A_T = [partA ; ones(1,size(partA,2))];
A = A_T';
%params = inv(A' * A) * A' * B';
param = inv(A)*B';
params = [param(1), param(2), -1, param(3)];
end
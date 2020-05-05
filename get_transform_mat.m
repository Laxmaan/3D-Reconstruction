function [local_transform_mat] = get_transform_mat(alpha,beta,gamma,tx,ty,tz,mode)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
T = [tx ty tz];
if mode=="rad"
alpha = rad2deg(alpha);
beta  = rad2deg(beta);
gamma = rad2deg(gamma);
end

R = rotx(alpha)*roty(beta)*rotz(gamma); 
local_transform_mat =   trvec2tform(T)*rotm2tform(R);
end

